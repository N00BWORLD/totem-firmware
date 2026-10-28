// 토템 나침반 (Totem Compass)
// 토템 두 대가 LoRa로 서로의 위치를 주고받아 OLED에 상대 방향과 거리를 보여 주고,
// BLE로 폰 지도 앱(totem-map)과 연결됩니다. 하드웨어와 통신 규격은 README.md 참고.
#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <RadioLib.h>
#include <U8g2lib.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#if defined(CONFIG_BLUEDROID_ENABLED)
#include <BLE2902.h>
#endif
#include <esp_mac.h>
#include "mbedtls/gcm.h"

#include "config.h"
#include "protocol.h"
#if !__has_include("secrets.h")
#error "secrets.h 가 없습니다. secrets.example.h 를 secrets.h 로 복사하고 키를 넣어 주세요 (README.md 참고)."
#endif
#include "secrets.h"
#include "bno055.h"
#include "gps.h"

#define FONT_KO u8g2_font_gulim11_t_korean2  // 한글 11px
#define FONT_SMALL u8g2_font_5x8_tf
#define FONT_BIG u8g2_font_helvB14_tf

SX1262 radio = new Module(LORA_NSS, LORA_DIO1, LORA_RST, LORA_BUSY);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C oled(U8G2_R0, PIN_OLED_RST, PIN_OLED_SCL, PIN_OLED_SDA);
Bno055 bno;
NmeaParser gps;
Preferences prefs;
BLECharacteristic *statusChar;
QueueHandle_t cmdQueue;

// ---- 내 기기 ----
uint16_t myId;
char bleName[16];
uint8_t netId;
uint32_t bootNonce, txCounter;

// ---- 하드웨어 상태 ----
bool loraOk, bnoOk;
volatile bool bleConnected;
volatile bool radioIrq;
float headingDeg;
uint8_t calStat;
bool calSaved;
float batV;
uint8_t batPct;

// ---- 내 위치 (GPS, 안 잡히면 폰이 빌려준 위치) ----
bool myFix, usingPhone;
int32_t myLat, myLon;
bool phHave;
int32_t phLat, phLon;
uint16_t phAcc;
uint32_t phMs;

// ---- 상대 토템 ----
bool frSeen, frFix, frPhone;
uint16_t frId;
uint32_t frMs;
int16_t frRssi;
int32_t frLat, frLon;
uint8_t frSats, frBat;

// ---- 같이 쓰는 만남 장소 ----
bool meetSet;
int32_t meetLat, meetLon;
uint16_t meetVer;

// ---- 메시지 ----
uint8_t txMsgSeq, txMsgType, pingTx;  // pingTx: 0 없음, 1 보내는 중, 2 전달됨
uint8_t rxMsgSeq, pingRx, msgRx = 1;

// ---- 설정 (NVS "totem") ----
uint16_t farM;
uint8_t screenMin;

// ---- 화면 ----
enum { PAGE_FRIEND, PAGE_MEET, PAGE_GPS, PAGE_COMPASS, PAGE_SYSTEM, NPAGES };
const char *PAGE_NAMES[] = {"상대", "만남 장소", "GPS", "나침반", "시스템"};
int page = PAGE_FRIEND;
bool screenOn = true;
uint32_t lastActivity;
uint8_t alert;  // 0 없음, 1~3 받은 메시지, 10 너무 멀어짐
uint32_t alertMs;
bool farAlerted;
char toast[40];
uint32_t toastMs;
uint32_t nextTxMs;

void IRAM_ATTR onRadioIrq() { radioIrq = true; }

// ================= BLE =================
class ServerCb : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { bleConnected = true; }
  void onDisconnect(BLEServer *) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

// BLE 콜백은 다른 태스크에서 돌아서, 명령은 큐에 넣고 loop()에서 처리
class CmdCb : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    Cmd cmd;
    size_t n = c->getLength();
    if (n == 0) return;
    cmd.len = min(n, sizeof(cmd.d));
    memcpy(cmd.d, c->getData(), cmd.len);
    xQueueSend(cmdQueue, &cmd, 0);
  }
};

void setupBle() {
  BLEDevice::init(bleName);
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCb());
  BLEService *svc = server->createService(BLE_SVC_UUID);
  statusChar = svc->createCharacteristic(BLE_STATUS_UUID, BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
#if defined(CONFIG_BLUEDROID_ENABLED)
  statusChar->addDescriptor(new BLE2902());
#endif
  BLECharacteristic *cmd = svc->createCharacteristic(BLE_CMD_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmd->setCallbacks(new CmdCb());
  svc->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData ad, sr;
  ad.setFlags(0x06);
  ad.setCompleteServices(BLEUUID(BLE_SVC_UUID));
  sr.setName(bleName);  // 128비트 UUID와 이름이 한 패킷에 안 들어가서 이름은 스캔 응답에
  adv->setAdvertisementData(ad);
  adv->setScanResponseData(sr);
  BLEDevice::startAdvertising();
}

static void put16(uint8_t *b, uint16_t v) { memcpy(b, &v, 2); }
static void put32(uint8_t *b, int32_t v) { memcpy(b, &v, 4); }

// 앱이 읽는 42바이트 상태 (형식은 protocol.h / README.md)
void updateStatus() {
  uint8_t b[STATUS_LEN] = {0};
  uint32_t age = frSeen ? (millis() - frMs) / 1000 : 0;
  b[0] = STATUS_VER;
  b[1] = (myFix ? SF_MYFIX : 0) | (frSeen ? SF_SEEN : 0) | (frFix ? SF_FRFIX : 0) | (meetSet ? SF_MEET : 0) |
         (bnoOk ? SF_COMPASS : 0) | (usingPhone ? SF_PHONE : 0);
  put32(b + 2, myLat);
  put32(b + 6, myLon);
  put16(b + 10, (uint16_t)lroundf(headingDeg) % 360);
  b[12] = gps.sats;
  b[13] = batPct;
  put32(b + 14, frLat);
  put32(b + 18, frLon);
  put16(b + 22, min(age, 65535UL));
  put16(b + 24, (uint16_t)frRssi);
  b[26] = frSats;
  b[27] = frBat;
  put32(b + 28, meetLat);
  put32(b + 32, meetLon);
  b[36] = pingRx;
  b[37] = pingTx;
  b[38] = msgRx;
  put16(b + 39, farM);
  b[41] = screenMin;
  statusChar->setValue(b, STATUS_LEN);
  if (bleConnected) statusChar->notify();
}

// ================= LoRa =================
static void makeIv(const uint8_t *frame, uint8_t *iv) {
  memset(iv, 0, 12);
  memcpy(iv, frame + 1, 10);
}

bool sealFrame(const Payload &p, uint8_t *frame) {
  frame[0] = netId;
  frame[1] = myId >> 8;
  frame[2] = myId & 0xFF;
  txCounter++;
  memcpy(frame + 3, &bootNonce, 4);
  memcpy(frame + 7, &txCounter, 4);
  uint8_t iv[12];
  makeIv(frame, iv);
  mbedtls_gcm_context g;
  mbedtls_gcm_init(&g);
  mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, TOTEM_KEY, 128);
  int r = mbedtls_gcm_crypt_and_tag(&g, MBEDTLS_GCM_ENCRYPT, sizeof(Payload), iv, 12, frame, FRAME_HDR,
                                    (const uint8_t *)&p, frame + FRAME_HDR, FRAME_TAG, frame + FRAME_HDR + sizeof(Payload));
  mbedtls_gcm_free(&g);
  return r == 0;
}

bool openFrame(const uint8_t *frame, Payload &p) {
  uint8_t iv[12];
  makeIv(frame, iv);
  mbedtls_gcm_context g;
  mbedtls_gcm_init(&g);
  mbedtls_gcm_setkey(&g, MBEDTLS_CIPHER_ID_AES, TOTEM_KEY, 128);
  int r = mbedtls_gcm_auth_decrypt(&g, sizeof(Payload), iv, 12, frame, FRAME_HDR, frame + FRAME_HDR + sizeof(Payload), FRAME_TAG,
                                   frame + FRAME_HDR, (uint8_t *)&p);
  mbedtls_gcm_free(&g);
  return r == 0;
}

void urgentTx() { nextTxMs = millis() + 200 + random(400); }

void sendBeacon() {
  Payload p{};
  p.magic = 'T';
  p.ver = 1;
  p.flags = (myFix ? PF_FIX : 0) | (meetSet ? PF_MEET : 0) | (usingPhone ? PF_PHONE : 0);
  p.sats = gps.sats;
  p.bat = batPct;
  p.lat = myLat;
  p.lon = myLon;
  p.meetVer = meetVer;
  p.mlat = meetLat;
  p.mlon = meetLon;
  p.msgSeq = pingTx == 1 ? txMsgSeq : 0;
  p.msgType = txMsgType;
  p.ackSeq = rxMsgSeq;
  uint8_t f[FRAME_LEN];
  if (!sealFrame(p, f)) return;
  for (int i = 0; i < 3; i++) {  // 보내기 전에 채널이 비었는지 확인 (LBT)
    if (radio.scanChannel() != RADIOLIB_LORA_DETECTED) break;
    delay(random(40, 160));
  }
  radio.transmit(f, FRAME_LEN);
  radioIrq = false;
  radio.startReceive();
}

void raiseAlert(uint8_t kind);
void showToast(const char *s);

void onFriend(uint16_t sid, const Payload &p, float rssi) {
  frSeen = true;
  frId = sid;
  frMs = millis();
  frRssi = lroundf(rssi);
  frFix = p.flags & PF_FIX;
  frPhone = p.flags & PF_PHONE;
  if (frFix) {
    frLat = p.lat;
    frLon = p.lon;
  }
  frSats = p.sats;
  frBat = p.bat;

  // 만남 장소: 버전이 더 새로운 쪽을 따름
  if ((int16_t)(p.meetVer - meetVer) > 0) {
    meetVer = p.meetVer;
    meetSet = p.flags & PF_MEET;
    meetLat = p.mlat;
    meetLon = p.mlon;
    showToast(meetSet ? "만남 장소 지정됨" : "만남 장소 지워짐");
  }
  if (p.msgSeq && p.msgSeq != rxMsgSeq) {
    rxMsgSeq = p.msgSeq;
    msgRx = (p.msgType >= 1 && p.msgType <= 3) ? p.msgType : 1;
    pingRx++;
    raiseAlert(msgRx);
    urgentTx();  // 받았다고 바로 알려 줌
  }
  if (pingTx == 1 && p.ackSeq == txMsgSeq) {
    pingTx = 2;
    showToast("전달됐어요");
  }
}

void handleRadio() {
  if (!radioIrq) return;
  radioIrq = false;
  uint8_t f[256];
  size_t n = radio.getPacketLength();
  int st = radio.readData(f, min(n, sizeof(f)));
  float rssi = radio.getRSSI();
  radio.startReceive();
  if (st != RADIOLIB_ERR_NONE || n != FRAME_LEN || f[0] != netId) return;
  uint16_t sid = f[1] << 8 | f[2];
  if (sid == myId) return;
  Payload p;
  if (!openFrame(f, p) || p.magic != 'T') return;  // 키가 다르면 여기서 버려짐
  onFriend(sid, p, rssi);
}

// ================= 센서 =================
void readBattery() {
  digitalWrite(PIN_BAT_CTRL, HIGH);
  delay(5);
  uint32_t mv = 0;
  for (int i = 0; i < 8; i++) mv += analogReadMilliVolts(PIN_BAT_ADC);
  digitalWrite(PIN_BAT_CTRL, LOW);
  batV = mv / 8.0f * BAT_DIVIDER / 1000.0f;
  static const float V[] = {3.30, 3.50, 3.60, 3.70, 3.75, 3.80, 3.85, 3.90, 4.00, 4.10, 4.20};
  static const uint8_t P[] = {0, 5, 10, 20, 30, 40, 50, 60, 75, 90, 100};
  if (batV <= V[0]) batPct = 0;
  else if (batV >= V[10]) batPct = 100;
  else
    for (int i = 1; i < 11; i++)
      if (batV < V[i]) {
        batPct = P[i - 1] + (P[i] - P[i - 1]) * (batV - V[i - 1]) / (V[i] - V[i - 1]);
        break;
      }
}

void saveCalibration() {
  uint8_t cal[Bno055::CAL_LEN];
  if (!bnoOk || !bno.save(cal)) return;
  prefs.begin("bnocal", false);
  prefs.putBytes("cal", cal, sizeof(cal));
  prefs.end();
  calSaved = true;
}

void readCompass() {
  if (!bnoOk) return;
  static uint32_t calMs;
  float h = bno.heading();
  if (h >= 0) headingDeg = fmodf(h + MAG_DECLINATION + HEADING_OFFSET + 720.0f, 360.0f);
  if (millis() - calMs > 500) {
    calMs = millis();
    calStat = bno.calib();
    if (calStat == 0xFF && !calSaved && millis() > 30000) saveCalibration();  // 완전히 보정되면 부팅마다 한 번 저장
  }
}

void updatePosition() {
  if (gps.valid()) {
    myFix = true;
    usingPhone = false;
    myLat = gps.lat;
    myLon = gps.lon;
  } else if (phHave && millis() - phMs < PHONE_LOC_TTL_MS) {
    myFix = true;
    usingPhone = true;
    myLat = phLat;
    myLon = phLon;
  } else {
    myFix = usingPhone = false;
  }
}

// a에서 b까지 거리(m)와 방위(진북 기준 도)
void distBearing(int32_t la1, int32_t lo1, int32_t la2, int32_t lo2, float &d, float &brg) {
  double p1 = la1 * 1e-7 * DEG_TO_RAD, p2 = la2 * 1e-7 * DEG_TO_RAD;
  double dl = ((double)lo2 - (double)lo1) * 1e-7 * DEG_TO_RAD;
  double x = dl * cos((p1 + p2) / 2), y = p2 - p1;
  d = sqrt(x * x + y * y) * 6371000.0;
  brg = atan2(sin(dl) * cos(p2), cos(p1) * sin(p2) - sin(p1) * cos(p2) * cos(dl)) * RAD_TO_DEG;
  if (brg < 0) brg += 360;
}

bool friendDistance(float &d, float &brg) {
  if (!myFix || !frSeen || !frFix || (millis() - frMs) / 1000 > FRIEND_VALID_S) return false;
  distBearing(myLat, myLon, frLat, frLon, d, brg);
  return true;
}

void checkFar() {
  float d, brg;
  if (!farM || !friendDistance(d, brg)) return;
  if (d > farM && !farAlerted) {
    farAlerted = true;
    raiseAlert(10);
  } else if (d < farM * 0.8f) {
    farAlerted = false;
  }
}

// ================= 앱에서 온 명령 =================
void sendMessage(uint8_t type) {
  txMsgType = type;
  txMsgSeq = txMsgSeq % 255 + 1;
  pingTx = 1;
  urgentTx();
  showToast("보내는 중...");
}

void handleCmd(const Cmd &c) {
  switch (c.d[0]) {
    case CMD_MEET:
      if (c.len < 9) break;
      memcpy(&meetLat, c.d + 1, 4);
      memcpy(&meetLon, c.d + 5, 4);
      meetSet = true;
      meetVer++;
      urgentTx();
      showToast("만남 장소 지정됨");
      break;
    case CMD_MEET_CLEAR:
      meetSet = false;
      meetVer++;
      urgentTx();
      showToast("만남 장소 지워짐");
      break;
    case CMD_MSG:
      sendMessage(c.len >= 2 ? c.d[1] : 1);
      break;
    case CMD_SETTINGS:
      if (c.len < 4) break;
      farM = c.d[1] | c.d[2] << 8;
      screenMin = c.d[3];
      prefs.begin("totem", false);
      prefs.putUShort("farM", farM);
      prefs.putUChar("scrMin", screenMin);
      prefs.end();
      farAlerted = false;
      showToast("설정 저장됨");
      break;
    case CMD_PHONE_LOC:
      if (c.len < 11) break;
      memcpy(&phLat, c.d + 1, 4);
      memcpy(&phLon, c.d + 5, 4);
      phAcc = c.d[9] | c.d[10] << 8;
      phMs = millis();
      phHave = true;
      break;
  }
}

// ================= 화면 =================
void wakeScreen() {
  lastActivity = millis();
  if (!screenOn) {
    oled.setPowerSave(0);
    screenOn = true;
  }
}

void raiseAlert(uint8_t kind) {
  alert = kind;
  alertMs = millis();
  wakeScreen();
}

void showToast(const char *s) {
  strlcpy(toast, s, sizeof(toast));
  toastMs = millis();
}

void onShortPress() {
  if (alert) {
    alert = 0;
    return;
  }
  page = (page + 1) % NPAGES;
}

void onLongPress() {
  if (page == PAGE_FRIEND) sendMessage(1);
  else if (page == PAGE_COMPASS) {
    saveCalibration();
    showToast(calSaved ? "보정값 저장됨" : "저장 실패");
  }
}

void handleButton() {
  static bool down, longFired, swallow;
  static uint32_t t0;
  bool pressed = digitalRead(PIN_BTN) == LOW;
  if (pressed && !down) {
    down = true;
    t0 = millis();
    longFired = false;
    swallow = !screenOn;  // 화면이 꺼져 있으면 첫 누름은 켜기만
    wakeScreen();
  }
  if (pressed && down && !longFired && !swallow && millis() - t0 > 800) {
    longFired = true;
    onLongPress();
  }
  if (!pressed && down) {
    down = false;
    if (!longFired && !swallow && millis() - t0 > 30) onShortPress();
  }
}

void fmtDist(char *s, size_t n, float m) {
  if (m < 1000) snprintf(s, n, "%dm", (int)lroundf(m));
  else snprintf(s, n, "%.1fkm", m / 1000);
}

void ko(int x, int y, const char *s) {
  oled.setFont(FONT_KO);
  oled.drawUTF8(x, y, s);
}

// 화면 위쪽 기준 시계 방향 deg 각도로 화살표
void drawArrow(int cx, int cy, int r, float deg) {
  float a = deg * DEG_TO_RAD;
  auto px = [&](float ang, float rr) { return cx + (int)lroundf(rr * sinf(ang)); };
  auto py = [&](float ang, float rr) { return cy - (int)lroundf(rr * cosf(ang)); };
  float side = 2.6f;  // 약 150도
  oled.drawCircle(cx, cy, r + 2);
  oled.drawTriangle(px(a, r), py(a, r), px(a + side, r * 0.6f), py(a + side, r * 0.6f), px(a, -r * 0.2f), py(a, -r * 0.2f));
  oled.drawTriangle(px(a, r), py(a, r), px(a - side, r * 0.6f), py(a - side, r * 0.6f), px(a, -r * 0.2f), py(a, -r * 0.2f));
}

// 맨 위 줄: 화면 이름 | BLE, 폰 위치, 배터리
void drawHeader() {
  char s[24];
  ko(0, 11, PAGE_NAMES[page]);
  oled.setFont(FONT_SMALL);
  snprintf(s, sizeof(s), "%s%s %d%%", bleConnected ? "BLE " : "", usingPhone ? "PH " : "", batPct);
  oled.drawStr(128 - oled.getStrWidth(s), 9, s);
  oled.drawHLine(0, 13, 128);
}

const char *signalWord(int rssi) { return rssi > -80 ? "강함" : rssi > -100 ? "보통" : "약함"; }

void drawFriendPage() {
  char s[40];
  uint32_t age = (millis() - frMs) / 1000;
  float d, brg;
  if (!frSeen) {
    ko(0, 32, "아직 상대가 없어요");
    ko(0, 48, "신호 기다리는 중...");
  } else if (friendDistance(d, brg)) {
    drawArrow(28, 40, 19, brg - (bnoOk ? headingDeg : 0));
    fmtDist(s, sizeof(s), d);
    oled.setFont(FONT_BIG);
    oled.drawStr(58, 36, s);
    snprintf(s, sizeof(s), "%lu초 전", (unsigned long)age);
    ko(58, 50, s);
    oled.setFont(FONT_SMALL);
    snprintf(s, sizeof(s), "%ddBm", frRssi);
    oled.drawStr(58, 61, s);
    if (!bnoOk) ko(92, 61, "북=위");
  } else {
    // 한쪽이라도 GPS가 없으면 신호 세기로 찾기
    ko(0, 28, age < 20 ? "신호 세기" : "마지막 신호");
    ko(0, 44, signalWord(frRssi));
    int bars = constrain(map(frRssi, -120, -40, 0, 5), 0, 5);
    for (int i = 0; i < 5; i++) {
      int h = 4 + i * 5;
      if (i < bars) oled.drawBox(86 + i * 8, 46 - h, 6, h);
      else oled.drawFrame(86 + i * 8, 46 - h, 6, h);
    }
    snprintf(s, sizeof(s), "%lu초 전 %s", (unsigned long)age, !myFix ? "내 GPS 없음" : "상대 GPS 없음");
    ko(0, 61, s);
  }
}

void drawMeetPage() {
  char s[32];
  if (!meetSet) {
    ko(0, 32, "만남 장소 없음");
    ko(0, 48, "앱에서 정해 주세요");
    return;
  }
  if (!myFix) {
    ko(0, 32, "만남 장소 있음");
    ko(0, 48, "내 위치 찾는 중...");
    return;
  }
  float d, brg;
  distBearing(myLat, myLon, meetLat, meetLon, d, brg);
  drawArrow(28, 40, 19, brg - (bnoOk ? headingDeg : 0));
  fmtDist(s, sizeof(s), d);
  oled.setFont(FONT_BIG);
  oled.drawStr(58, 40, s);
}

void drawGpsPage() {
  char s[40];
  ko(0, 26, gps.valid() ? "GPS 잡힘" : usingPhone ? "폰 위치 사용 중" : "GPS 찾는 중...");
  snprintf(s, sizeof(s), "위도 %.5f", myLat * 1e-7);
  ko(0, 39, s);
  snprintf(s, sizeof(s), "경도 %.5f", myLon * 1e-7);
  ko(0, 51, s);
  snprintf(s, sizeof(s), "위성 %d개  HDOP %.1f", gps.sats, gps.hdop);
  ko(0, 63, s);
}

void drawCompassPage() {
  char s[32];
  if (!bnoOk) {
    ko(0, 34, "나침반 센서 없음");
    ko(0, 50, "BNO055 배선 확인");
    return;
  }
  drawArrow(28, 40, 19, -headingDeg);
  ko(23, 26, "북");
  oled.setFont(FONT_BIG);
  snprintf(s, sizeof(s), "%d", (int)lroundf(headingDeg) % 360);
  oled.drawStr(58, 32, s);
  oled.setFont(FONT_SMALL);
  snprintf(s, sizeof(s), "S%d G%d A%d M%d", calStat >> 6, (calStat >> 4) & 3, (calStat >> 2) & 3, calStat & 3);
  oled.drawStr(58, 43, s);
  ko(58, 61, calSaved ? "보정 저장됨" : "길게:보정저장");
}

void drawSystemPage() {
  char s[40];
  snprintf(s, sizeof(s), "배터리 %.2fV %d%%", batV, batPct);
  ko(0, 26, s);
  snprintf(s, sizeof(s), "LoRa %s  상대 %d%%", loraOk ? "정상" : "오류", frSeen ? frBat : 0);
  ko(0, 39, s);
  snprintf(s, sizeof(s), "%s %s", bleName, bleConnected ? "연결됨" : "대기");
  ko(0, 51, s);
  snprintf(s, sizeof(s), "펌웨어 %s", FW_VERSION);
  ko(0, 63, s);
}

void drawAlert() {
  static const char *MSG[] = {"", "보고싶어!", "어디야?", "곧 갈게!"};
  char s[32];
  oled.drawFrame(0, 0, 128, 64);
  oled.drawFrame(2, 2, 124, 60);
  if (alert == 10) {
    float d, brg;
    ko(10, 22, "너무 멀어졌어요");
    if (friendDistance(d, brg)) {
      fmtDist(s, sizeof(s), d);
      oled.setFont(FONT_BIG);
      oled.drawStr(10, 42, s);
    }
  } else {
    ko(10, 22, "상대가 보냈어요");
    oled.setFont(u8g2_font_gulim16_t_korean2);
    oled.drawUTF8(10, 42, MSG[alert]);
  }
  ko(10, 58, "버튼을 누르면 닫혀요");
}

void render() {
  oled.clearBuffer();
  if (alert) {
    drawAlert();
  } else {
    drawHeader();
    switch (page) {
      case PAGE_FRIEND: drawFriendPage(); break;
      case PAGE_MEET: drawMeetPage(); break;
      case PAGE_GPS: drawGpsPage(); break;
      case PAGE_COMPASS: drawCompassPage(); break;
      case PAGE_SYSTEM: drawSystemPage(); break;
    }
    if (millis() - toastMs < 2000) {
      oled.setDrawColor(0);
      oled.drawBox(0, 51, 128, 13);
      oled.setDrawColor(1);
      oled.drawHLine(0, 51, 128);
      oled.setFont(FONT_KO);
      oled.drawUTF8(64 - oled.getUTF8Width(toast) / 2, 63, toast);
    }
  }
  oled.sendBuffer();
}

void updateLed() {
  bool on = alert && millis() - alertMs < 10000 && (millis() / 250) % 2;
  digitalWrite(PIN_LED, on);
}

// 시리얼 모니터용 한 줄 상태 (영문 유지: 터미널 인코딩과 무관하게 보이도록)
void printStatus() {
  Serial.printf("hdg=%.0f fix=%d sats=%d | friend seen=%d valid=%d rssi=%d | bat=%.2fV\n", headingDeg, myFix, gps.sats, frSeen,
                frSeen && frFix && (millis() - frMs) / 1000 <= FRIEND_VALID_S, frRssi, batV);
}

// ================= setup / loop =================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_VEXT, OUTPUT);
  digitalWrite(PIN_VEXT, LOW);
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_BAT_CTRL, OUTPUT);
  analogReadResolution(12);
  delay(100);

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);
  myId = mac[4] << 8 | mac[5];
  snprintf(bleName, sizeof(bleName), "TOTEM-%02X%02X", mac[4], mac[5]);
  netId = TOTEM_KEY[0] ^ TOTEM_KEY[5] ^ TOTEM_KEY[11] ^ 0xA5;
  bootNonce = esp_random();
  txMsgSeq = esp_random() % 255 + 1;
  randomSeed(esp_random());

  oled.begin();
  oled.clearBuffer();
  ko(0, 14, "토템 나침반 v" FW_VERSION);
  ko(0, 32, "시작하는 중...");
  oled.sendBuffer();

  prefs.begin("totem", true);
  farM = prefs.getUShort("farM", 0);
  screenMin = prefs.getUChar("scrMin", 0);
  prefs.end();

  int st = radio.begin(LORA_FREQ, LORA_BW, LORA_SF, LORA_CR, LORA_SYNC, LORA_POWER, LORA_PREAMBLE, LORA_TCXO, false);
  loraOk = st == RADIOLIB_ERR_NONE;
  if (loraOk) {
    radio.setCurrentLimit(140);
    radio.setDio1Action(onRadioIrq);
    radio.startReceive();
  }

  Wire1.begin(PIN_BNO_SDA, PIN_BNO_SCL, 100000);
  bnoOk = bno.begin(Wire1, BNO_ADDR);
  if (bnoOk) {
    uint8_t cal[Bno055::CAL_LEN];
    prefs.begin("bnocal", true);
    bool have = prefs.getBytes("cal", cal, sizeof(cal)) == sizeof(cal);
    prefs.end();
    if (have) bno.restore(cal);
    else bno.start();
  }

  Serial1.begin(GPS_BAUD, SERIAL_8N1, PIN_GPS_RX, -1);
  readBattery();

  cmdQueue = xQueueCreate(8, sizeof(Cmd));
  setupBle();

  Serial.printf("Totem Compass v%s\n", FW_VERSION);
  Serial.printf("LoRa=%d(0=OK)  BNO055=%s\n", st, bnoOk ? "OK" : "FAIL");
  Serial.printf("BLE name=%s\n", bleName);
  lastActivity = millis();
  nextTxMs = millis() + random(TX_INTERVAL_MS);
}

void loop() {
  uint32_t now = millis();
  static uint32_t compassMs, batMs, statusMs, drawMs, printMs;

  while (Serial1.available()) gps.feed(Serial1.read());
  if (loraOk) handleRadio();
  Cmd c;
  while (xQueueReceive(cmdQueue, &c, 0)) handleCmd(c);
  handleButton();

  if (now - compassMs >= 50) {
    compassMs = now;
    readCompass();
  }
  if (now - batMs >= 10000) {
    batMs = now;
    readBattery();
  }
  updatePosition();
  checkFar();

  if (loraOk && (int32_t)(now - nextTxMs) >= 0) {
    sendBeacon();
    nextTxMs = millis() + TX_INTERVAL_MS + random(TX_JITTER_MS);
  }
  if (now - statusMs >= 500) {
    statusMs = now;
    updateStatus();
  }
  if (alert && alert != 10 && now - alertMs > ALERT_TIMEOUT_MS) alert = 0;
  if (screenOn && screenMin && !alert && now - lastActivity > screenMin * 60000UL) {
    oled.setPowerSave(1);
    screenOn = false;
  }
  if (screenOn && now - drawMs >= 200) {
    drawMs = now;
    render();
  }
  updateLed();
  if (now - printMs >= 2000) {
    printMs = now;
    printStatus();
  }
}

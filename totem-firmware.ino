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
char myName[NAME_LEN + 1];  // 화면 맨 위에 보이는 내 이름 (앱에서 정함)
char frName[NAME_LEN + 1];  // 상대가 보내 준 이름
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
bool charging;

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
bool frCharging;

// ---- 같이 쓰는 만남 장소 ----
bool meetSet;
int32_t meetLat, meetLon;
uint16_t meetVer;

// ---- 메시지 ----
uint8_t txMsgSeq, txMsgType, pingTx;  // pingTx: 0 없음, 1 보내는 중, 2 전달됨
uint8_t rxMsgSeq, pingRx, msgRx = 1;

// ---- 실내 찾기 ----
uint32_t findUntil;          // 0이면 꺼짐
bool frFinding;              // 상대도 찾는 중
uint16_t myAcc, frAcc;       // 위치 오차 (m)
float rssiAvg;               // 상대 신호 평균 (dBm)
int8_t trend;                // 1 가까워짐, -1 멀어짐, 0 그대로
int16_t rssiHist[6];         // 최근 신호 6개 (오래된 것부터)
uint8_t rssiHistN;
uint32_t rssiHistMs;         // 마지막으로 넣은 시각
float scanSum[SCAN_BINS];    // 방향별 신호 합 (진북 기준 30도 칸)
uint8_t scanCnt[SCAN_BINS];
bool scanReady;
uint16_t scanDir;
uint8_t scanSpread, scanCover;

// ---- 설정 (NVS "totem") ----
uint16_t farM;
uint8_t screenMin;

// ---- 화면 ----
enum { PAGE_FRIEND, PAGE_FIND, PAGE_MEET, PAGE_GPS, PAGE_COMPASS, PAGE_SYSTEM, NPAGES };
const char *PAGE_NAMES[] = {"상대", "실내 찾기", "만남 장소", "GPS", "나침반", "시스템"};
int page = PAGE_FRIEND;
bool screenOn = true;
uint32_t lastActivity;
uint8_t alert;  // 0 없음, 1~4 받은 메시지 (4 깜박여 줘), 10 너무 멀어짐
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
  adv->setMinInterval(BLE_ADV_MIN);  // 광고를 덜 자주 (배터리 절약, 폰이 찾는 데 1초 안팎)
  adv->setMaxInterval(BLE_ADV_MAX);
  BLEDevice::startAdvertising();
}

bool finding() { return findUntil && (int32_t)(millis() - findUntil) < 0; }
bool friendFinding() { return frFinding && frSeen && millis() - frMs < 10000; }

static void put16(uint8_t *b, uint16_t v) { memcpy(b, &v, 2); }
static void put32(uint8_t *b, int32_t v) { memcpy(b, &v, 4); }

// 앱이 읽는 42바이트 상태 (형식은 protocol.h / README.md)
void updateStatus() {
  uint8_t b[STATUS_LEN] = {0};
  uint32_t age = frSeen ? (millis() - frMs) / 1000 : 0;
  b[0] = STATUS_VER;
  b[1] = (myFix ? SF_MYFIX : 0) | (frSeen ? SF_SEEN : 0) | (frFix ? SF_FRFIX : 0) | (meetSet ? SF_MEET : 0) |
         (bnoOk ? SF_COMPASS : 0) | (usingPhone ? SF_PHONE : 0) | (charging ? SF_CHARGING : 0);
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
  // 42~: 실내 찾기
  b[42] = (finding() ? FF_FINDING : 0) | (friendFinding() ? FF_FRIEND_FINDING : 0) | (scanReady ? FF_SCAN_READY : 0) |
          (frSeen && frCharging ? FF_FRIEND_CHARGING : 0);
  put16(b + 43, (uint16_t)(int16_t)lroundf(rssiAvg));
  b[45] = (uint8_t)trend;
  put16(b + 46, scanDir);
  b[48] = scanSpread;
  b[49] = scanCover;
  put16(b + 50, myAcc);
  put16(b + 52, frAcc);
  memcpy(b + 54, myName, NAME_LEN);
  memcpy(b + 66, frName, NAME_LEN);
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
  p.ver = PAYLOAD_VER;
  p.flags = (myFix ? PF_FIX : 0) | (meetSet ? PF_MEET : 0) | (usingPhone ? PF_PHONE : 0) | (finding() ? PF_FIND : 0) |
            (charging ? PF_CHARGING : 0);
  p.acc = myAcc;
  memcpy(p.name, myName, NAME_LEN);
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

// ---- 실내 찾기: 신호 평균, 가까워지는지, 방향 재기 ----
void resetScan() {
  memset(scanSum, 0, sizeof(scanSum));
  memset(scanCnt, 0, sizeof(scanCnt));
  scanReady = false;
  scanCover = scanSpread = 0;
}

// rescan: 찾는 중이면 방향 재기를 처음부터. 아니면 시간만 연장
void startFind(bool rescan) {
  bool was = finding();
  if (!was || rescan) resetScan();
  if (!was) showToast("실내 찾기 시작");
  else if (rescan) showToast("방향 다시 재기");
  findUntil = millis() + FIND_DURATION_MS;
  urgentTx();
}

void stopFind() {
  findUntil = 0;
  showToast("실내 찾기 끝");
}

void addSignal(float rssi) {
  rssiAvg = (rssiHistN == 0 || millis() - rssiHistMs > 30000) ? rssi : rssiAvg * 0.6f + rssi * 0.4f;
  if (millis() - rssiHistMs > 30000) rssiHistN = 0;  // 오래 끊겼으면 처음부터
  rssiHistMs = millis();
  if (rssiHistN == 6) memmove(rssiHist, rssiHist + 1, 5 * sizeof(int16_t));
  else rssiHistN++;
  rssiHist[rssiHistN - 1] = lroundf(rssi);

  // 최근 3개 평균과 그 전 3개 평균 비교
  trend = 0;
  if (rssiHistN == 6) {
    float older = (rssiHist[0] + rssiHist[1] + rssiHist[2]) / 3.0f;
    float newer = (rssiHist[3] + rssiHist[4] + rssiHist[5]) / 3.0f;
    if (newer - older >= 3) trend = 1;
    else if (older - newer >= 3) trend = -1;
  }

  // 방향 재기: 찾는 중에 몸을 돌리면, 몸이 전파를 가리지 않는 쪽(상대 쪽)이 세게 잡힘
  if (!finding() || !bnoOk) return;
  int bin = (int)(headingDeg / (360 / SCAN_BINS)) % SCAN_BINS;
  scanSum[bin] += rssi;
  scanCnt[bin]++;
  int best = -1;
  float bestV = -999, worstV = 999;
  scanCover = 0;
  for (int i = 0; i < SCAN_BINS; i++) {
    if (!scanCnt[i]) continue;
    scanCover++;
    float v = scanSum[i] / scanCnt[i];
    if (v > bestV) bestV = v, best = i;
    if (v < worstV) worstV = v;
  }
  scanReady = scanCover >= SCAN_BINS * 2 / 3;
  if (scanReady) {
    scanDir = best * (360 / SCAN_BINS) + 180 / SCAN_BINS;
    scanSpread = min(99, (int)lroundf(bestV - worstV));
  }
}

void onFriend(uint16_t sid, const Payload &p, float rssi) {
  addSignal(rssi);
  frFinding = p.flags & PF_FIND;
  frAcc = p.acc;
  memcpy(frName, p.name, NAME_LEN);
  frName[NAME_LEN] = 0;
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
  frCharging = p.flags & PF_CHARGING;

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
    msgRx = (p.msgType >= 1 && p.msgType <= 4) ? p.msgType : 1;
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
  float v = mv / 8.0f * BAT_DIVIDER / 1000.0f;
  uint32_t now = millis();

  // 충전기를 꽂으면 전압이 갑자기 올라가고(충전 전압), 빼면 갑자기 내려감.
  // 충전 중 전압으로 %를 매기면 뺀 순간 확 닳은 것처럼 보이므로 충전 중엔 %를 안 믿음.
  static float avg = 0;
  static uint32_t settleUntil = 0;
  if (avg == 0) avg = v;
  if (v > avg + BAT_JUMP_V) charging = true;
  else if (v < avg - BAT_JUMP_V) {
    charging = false;
    settleUntil = now + BAT_SETTLE_MS;  // 뺀 직후 전압이 자리 잡을 때까지 기다림
  }
  // 다 찬 배터리에서 뺄 때는 전압이 조금만 내려감. 충전 중 전압은 계속 오르기만 하므로
  // 최고값보다 확실히 낮은 상태가 30초 이어지면 뺀 것으로 봄
  static float peak = 0;
  static uint8_t lower = 0;
  if (!charging) peak = 0, lower = 0;
  else {
    peak = max(peak, v);
    lower = v < peak - BAT_UNPLUG_V ? lower + 1 : 0;
    if (lower >= 3) {
      charging = false;
      settleUntil = now + BAT_SETTLE_MS;
    }
  }
  if (fabsf(v - avg) > BAT_JUMP_V) avg = v;  // 꽂거나 뺀 순간엔 평균을 새로 시작
  else avg += (v - avg) * 0.2f;              // 무선 송신 등으로 잠깐 처지는 것은 평균으로 무시
  batV = avg;
  if (charging || (int32_t)(now - settleUntil) < 0) return;  // %는 마지막 값 유지

  // 다 찬 배터리도 충전기를 빼면 4.15V 안팎으로 내려와 쉼 → 4.15V 이상이면 100%
  static const float V[] = {3.30, 3.50, 3.60, 3.70, 3.75, 3.80, 3.85, 3.90, 3.98, 4.07, 4.15};
  static const uint8_t P[] = {0, 5, 10, 20, 30, 40, 50, 60, 75, 90, 100};
  uint8_t pct = 0;
  if (batV >= V[10]) pct = 100;
  else if (batV > V[0])
    for (int i = 1; i < 11; i++)
      if (batV < V[i]) {
        pct = P[i - 1] + (P[i] - P[i - 1]) * (batV - V[i - 1]) / (V[i] - V[i - 1]);
        break;
      }
  // 내려갈 땐 바로, 올라갈 땐 5% 넘게 높은 값이 30초 이어질 때만 (오르락내리락 방지).
  // 충전기를 꽂은 채로 켜서 '충전 중'을 못 알아챈 경우에도 결국 올라감
  static bool known = false;
  static uint8_t higher = 0;
  higher = pct > batPct + 5 ? higher + 1 : 0;
  if (!known || settleUntil || pct < batPct || higher >= 3) {
    batPct = pct;
    higher = 0;
  }
  known = true;
  settleUntil = 0;
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
    myAcc = max(3L, lroundf(gps.hdop * 5));  // 대략적인 추정
  } else if (phHave && millis() - phMs < PHONE_LOC_TTL_MS) {
    myFix = true;
    usingPhone = true;
    myLat = phLat;
    myLon = phLon;
    myAcc = phAcc;
  } else {
    myFix = usingPhone = false;
    myAcc = 0;
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
    case CMD_NAME:
      memset(myName, 0, sizeof(myName));
      memcpy(myName, c.d + 1, min(c.len - 1, NAME_LEN));
      prefs.begin("totem", false);
      prefs.putString("name", myName);
      prefs.end();
      urgentTx();
      showToast("이름 저장됨");
      break;
    case CMD_FIND: {
      uint8_t arg = c.len >= 2 ? c.d[1] : 1;
      if (arg == 0) stopFind();
      else startFind(arg == 2);  // 1: 켜기/연장, 2: 방향 다시 재기
      break;
    }
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
  showToast(PAGE_NAMES[page]);  // 맨 위에는 이름이 있으니 화면 이름은 잠깐 보여 줌
}

void onLongPress() {
  if (page == PAGE_FRIEND) sendMessage(1);
  else if (page == PAGE_FIND) startFind(true);
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

// 방향이 대략일 때: 화살표 대신 ±half도 부채꼴 (넓을수록 방향이 불확실)
void drawWedge(int cx, int cy, int r, float deg, float half) {
  if (half < WEDGE_MIN_DEG) {
    drawArrow(cx, cy, r, deg);
    return;
  }
  half = min(half, 90.0f);
  oled.drawCircle(cx, cy, r + 2);
  int n = max(2, (int)ceilf(half * 2 / 10));  // 10도마다 삼각형 하나
  float a0 = (deg - half) * DEG_TO_RAD, step = half * 2 / n * DEG_TO_RAD;
  for (int i = 0; i < n; i++) {
    float a = a0 + step * i, b = a + step;
    oled.drawTriangle(cx, cy, cx + lroundf(r * sinf(a)), cy - lroundf(r * cosf(a)),
                      cx + lroundf(r * sinf(b)), cy - lroundf(r * cosf(b)));
  }
}

// 거리 d에서 위치 오차가 err이면 방향은 ±asin(err/d)만큼 틀릴 수 있음
float spreadDeg(float err, float d) { return err >= d ? 90 : asinf(err / d) * RAD_TO_DEG; }

// 맨 위 줄: 내 이름 | 찾는 중, BLE, 폰 위치, 배터리
void drawHeader() {
  char s[24];
  ko(0, 11, myName);
  oled.setFont(FONT_SMALL);
  snprintf(s, sizeof(s), "%s%s%s %d%%", finding() || friendFinding() ? "FIND " : "", bleConnected ? "BLE " : "",
           usingPhone ? "PH " : "", batPct);
  if (charging) snprintf(s, sizeof(s), "%s%s%sCHG", finding() || friendFinding() ? "FIND " : "", bleConnected ? "BLE " : "",
                         usingPhone ? "PH " : "");
  oled.drawStr(128 - oled.getStrWidth(s), 9, s);
  oled.drawHLine(0, 13, 128);
}

const char *friendLabel() { return frName[0] ? frName : "상대"; }

const char *signalWord(int rssi) { return rssi > -80 ? "강함" : rssi > -100 ? "보통" : "약함"; }

void drawFriendPage() {
  char s[40];
  uint32_t age = (millis() - frMs) / 1000;
  float d, brg;
  bool located = frSeen && friendDistance(d, brg);
  bool fuzzy = located && myAcc + frAcc > d;  // 위치 오차가 거리보다 크면 화살표를 믿을 수 없음 (주로 실내 폰 위치)
  if (!frSeen) {
    ko(0, 32, "아직 상대가 없어요");
    ko(0, 48, "신호 기다리는 중...");
  } else if (fuzzy) {
    snprintf(s, sizeof(s), "근처예요 (약 %dm)", (int)lroundf(d));
    ko(0, 28, s);
    snprintf(s, sizeof(s), "신호 %s %ddBm", signalWord(frRssi), frRssi);
    ko(0, 43, s);
    ko(0, 58, "실내 찾기로 찾아보세요");
  } else if (located) {
    drawWedge(28, 40, 19, brg - (bnoOk ? headingDeg : 0), spreadDeg(myAcc + frAcc, d));
    fmtDist(s, sizeof(s), d);
    oled.setFont(FONT_BIG);
    oled.drawStr(58, 36, s);
    ko(58, 50, friendLabel());
    oled.setFont(FONT_SMALL);
    snprintf(s, sizeof(s), "%lus %ddBm", (unsigned long)age, frRssi);
    oled.drawStr(58, 61, s);
    if (!bnoOk) ko(92, 61, "북=위");
  } else {
    // 한쪽이라도 GPS가 없으면 신호 세기로 찾기
    ko(0, 28, age < 40 ? "신호 세기" : "마지막 신호");
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

void drawFindPage() {
  char s[40];
  uint32_t age = (millis() - frMs) / 1000;
  if (!finding()) {
    ko(0, 28, "길게 눌러 시작");
    ko(0, 43, "1초마다 신호, 방향 재기");
    if (friendFinding()) ko(0, 58, "상대가 찾는 중이에요");
    else if (frSeen) {
      snprintf(s, sizeof(s), "지금 신호 %ddBm", frRssi);
      ko(0, 58, s);
    }
    return;
  }
  if (!frSeen || age > 15) {
    ko(0, 32, "상대 신호가 없어요");
    ko(0, 48, "기다리는 중...");
    return;
  }

  // 왼쪽: 방향을 알면 화살표, 아니면 잰 방향 표시 (12칸)
  if (scanReady && scanSpread >= SCAN_MIN_SPREAD) {
    drawWedge(28, 40, 19, scanDir - headingDeg, SCAN_WEDGE_DEG);  // 신호로 잰 방향은 늘 대략
  } else {
    oled.drawCircle(28, 40, 3);
    for (int i = 0; i < SCAN_BINS; i++) {
      float a = (i * 30 + 15 - headingDeg) * DEG_TO_RAD;
      int x = 28 + lroundf(19 * sinf(a)), y = 40 - lroundf(19 * cosf(a));
      if (scanCnt[i]) oled.drawDisc(x, y, 3);
      else oled.drawCircle(x, y, 2);
    }
  }

  // 오른쪽: 신호 평균, 가까워지는지
  oled.setFont(FONT_BIG);
  snprintf(s, sizeof(s), "%d", (int)lroundf(rssiAvg));
  oled.drawStr(58, 32, s);
  int w = oled.getStrWidth(s);
  oled.setFont(FONT_SMALL);
  oled.drawStr(60 + w, 32, "dBm");
  ko(58, 46, trend > 0 ? "가까워져요" : trend < 0 ? "멀어져요" : "그대로");
  if (!bnoOk) ko(58, 60, "나침반 없음");
  else if (!scanReady) {
    snprintf(s, sizeof(s), "한 바퀴 %d/%d", scanCover, SCAN_BINS);
    ko(58, 60, s);
  } else ko(58, 60, scanSpread >= SCAN_MIN_SPREAD ? "화살표 쪽이 셈" : "방향 모르겠음");
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
  if (charging) snprintf(s, sizeof(s), "배터리 %.2fV 충전 중", batV);
  else snprintf(s, sizeof(s), "배터리 %.2fV %d%%", batV, batPct);
  ko(0, 26, s);
  if (frSeen && frCharging) snprintf(s, sizeof(s), "LoRa %s  상대 충전 중", loraOk ? "정상" : "오류");
  else snprintf(s, sizeof(s), "LoRa %s  상대 %d%%", loraOk ? "정상" : "오류", frSeen ? frBat : 0);
  ko(0, 39, s);
  snprintf(s, sizeof(s), "%s %s", bleName, bleConnected ? "연결됨" : "대기");
  ko(0, 51, s);
  snprintf(s, sizeof(s), "펌웨어 %s", FW_VERSION);
  ko(0, 63, s);
}

void drawAlert() {
  static const char *MSG[] = {"", "보고싶어!", "어디야?", "곧 갈게!"};
  char s[48];
  if (alert == 4) {  // 깜박여 줘: 멀리서도 보이게 화면 전체를 번쩍임
    bool inv = (millis() / 400) % 2;
    if (inv) oled.drawBox(0, 0, 128, 64);
    oled.setDrawColor(inv ? 0 : 1);
    oled.setFont(u8g2_font_gulim16_t_korean2);
    oled.drawUTF8(18, 26, "여기예요!");
    snprintf(s, sizeof(s), "%s에게서 찾는 중", friendLabel());
    ko(10, 44, s);
    ko(10, 60, "누르면 닫혀요");
    oled.setDrawColor(1);
    return;
  }
  oled.drawFrame(0, 0, 128, 64);
  oled.drawFrame(2, 2, 124, 60);
  if (alert == 10) {
    float d, brg;
    snprintf(s, sizeof(s), "멀어짐: %s", friendLabel());
    ko(10, 22, s);
    if (friendDistance(d, brg)) {
      fmtDist(s, sizeof(s), d);
      oled.setFont(FONT_BIG);
      oled.drawStr(10, 42, s);
    }
  } else {
    snprintf(s, sizeof(s), "%s에게서", friendLabel());
    ko(10, 22, s);
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
      case PAGE_FIND: drawFindPage(); break;
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
  bool on = alert == 4 ? (millis() / 150) % 2  // 깜박여 줘: 계속 빠르게
                      : alert && millis() - alertMs < 10000 && (millis() / 250) % 2;
  digitalWrite(PIN_LED, on);
}

// 시리얼 모니터용 한 줄 상태 (영문 유지: 터미널 인코딩과 무관하게 보이도록)
void printStatus() {
  Serial.printf("hdg=%.0f fix=%d sats=%d | friend seen=%d valid=%d rssi=%d | bat=%.2fV\n", headingDeg, myFix, gps.sats, frSeen,
                frSeen && frFix && (millis() - frMs) / 1000 <= FRIEND_VALID_S, frRssi, batV);
}

// ================= setup / loop =================
void setup() {
  setCpuFrequencyMhz(CPU_MHZ);  // 240 → 80 MHz: 할 일이 많지 않아서 충분하고 전류가 크게 줆
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
  screenMin = prefs.getUChar("scrMin", SCREEN_MIN_DEFAULT);
  snprintf(myName, sizeof(myName), "토템%02X%02X", mac[4], mac[5]);  // 이름을 정하기 전 기본값
  if (prefs.isKey("name")) prefs.getString("name", myName, sizeof(myName));
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

  if (now - compassMs >= (screenOn || finding() ? 50 : 500)) {  // 화면이 꺼져 있으면 나침반은 가끔만
    compassMs = now;
    readCompass();
  }
  if (now - batMs >= 10000) {
    batMs = now;
    readBattery();
  }
  updatePosition();
  checkFar();

  if (findUntil && !finding()) stopFind();  // 3분 지남
  if (loraOk && (int32_t)(now - nextTxMs) >= 0) {
    sendBeacon();
    bool fast = finding() || friendFinding();  // 한쪽이라도 찾는 중이면 둘 다 빠르게
    nextTxMs = millis() + (fast ? FIND_TX_INTERVAL_MS + random(400) : TX_INTERVAL_MS + random(TX_JITTER_MS));
  }
  if (now - statusMs >= 500) {
    statusMs = now;
    updateStatus();
  }
  if (alert && alert != 10 && now - alertMs > (alert == 4 ? 30000UL : ALERT_TIMEOUT_MS)) alert = 0;
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
  delay(LOOP_IDLE_MS);  // 쉬는 동안 CPU가 잠깐씩 잠 (GPS 9600bps는 UART 버퍼가 받아 둠)
}

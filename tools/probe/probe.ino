// 토템 하드웨어 찾기: BNO055 I2C 핀, GPS UART 핀/속도, 배터리 ADC 방식을 찾고
// 주변 LoRa 패킷을 들어 봅니다. 결과는 시리얼(115200)로 출력.
// 배선이 README와 다를 때 이걸 먼저 올려서 config.h 값을 정하세요.
#include <Wire.h>
#include <RadioLib.h>

SX1262 radio = new Module(8, 14, 12, 13);  // NSS, DIO1, RST, BUSY

const int CAND[] = {2, 3, 4, 5, 6, 7, 19, 20, 26, 33, 34, 38, 39, 40, 41, 42, 45, 46, 47, 48, 17, 18};  // not 43/44: USB-UART console
const int NC = sizeof(CAND) / sizeof(CAND[0]);

volatile bool rxFlag = false;
void IRAM_ATTR onRx() { rxFlag = true; }

int bnoAt(int sda, int scl) {
  Wire.end();
  if (!Wire.begin(sda, scl, 100000)) return 0;
  Wire.setTimeOut(20);
  for (int a = 0x28; a <= 0x29; a++) {
    Wire.beginTransmission(a);
    Wire.write(0x00);
    if (Wire.endTransmission(false) != 0) continue;
    if (Wire.requestFrom(a, 1) == 1 && Wire.read() == 0xA0) return a;
  }
  return 0;
}

void scanI2C() {
  Serial.println("== I2C scan (BNO055 chip id 0xA0)");
  for (int i = 0; i < NC; i++)
    for (int j = 0; j < NC; j++) {
      if (i == j) continue;
      int a = bnoAt(CAND[i], CAND[j]);
      if (a) Serial.printf("BNO055 FOUND addr=0x%02X SDA=%d SCL=%d\n", a, CAND[i], CAND[j]);
    }
  // Also list all devices on the OLED bus
  Wire.end();
  Wire.begin(17, 18);
  Serial.print("bus 17/18 devices:");
  for (int a = 1; a < 127; a++) { Wire.beginTransmission(a); if (Wire.endTransmission() == 0) Serial.printf(" 0x%02X", a); }
  Serial.println();
  Wire.end();
}

void scanGPS() {
  Serial.println("== GPS UART scan");
  const long BAUDS[] = {9600, 38400, 115200};
  for (int i = 0; i < NC; i++) {
    int p = CAND[i];
    if (p == 43 || p == 44) continue;  // USB-UART console
    for (long b : BAUDS) {
      Serial1.begin(b, SERIAL_8N1, p, -1);
      unsigned long t = millis();
      String buf;
      while (millis() - t < 1300) {
        while (Serial1.available()) { char c = Serial1.read(); if (buf.length() < 400) buf += c; }
      }
      Serial1.end();
      int k = buf.indexOf("$G");
      if (k >= 0) {
        int e = buf.indexOf('\n', k);
        Serial.printf("GPS FOUND RX=%d baud=%ld sample=%s\n", p, b, buf.substring(k, e > 0 ? e : k + 80).c_str());
      }
    }
  }
}

void battery() {
  Serial.println("== Battery ADC (GPIO1, ctrl GPIO37)");
  analogReadResolution(12);
  for (int lvl = 0; lvl <= 1; lvl++) {
    pinMode(37, OUTPUT);
    digitalWrite(37, lvl);
    delay(50);
    uint32_t mv = 0;
    for (int i = 0; i < 16; i++) mv += analogReadMilliVolts(1);
    mv /= 16;
    Serial.printf("ctrl=%d adc=%lumV vbat~%.2fV (x4.9)\n", lvl, (unsigned long)mv, mv * 4.9 / 1000.0);
  }
  pinMode(37, INPUT);
}

void sniffLoRa() {
  Serial.println("== LoRa sniff");
  const float FREQS[] = {922.0, 923.0, 921.9};
  const uint8_t SYNCS[] = {0x12, 0x34};
  for (float f : FREQS)
    for (uint8_t sw : SYNCS)
      for (int sf = 7; sf <= 12; sf++)
        for (float bw : {125.0f}) {
          int st = radio.begin(f, bw, sf, 5, sw, 10, 8, 1.8, false);
          if (st != 0) { Serial.printf("radio.begin err %d\n", st); return; }
          radio.setDio1Action(onRx);
          rxFlag = false;
          radio.startReceive();
          unsigned long dwell = sf >= 11 ? 7000 : 4500;
          unsigned long t = millis();
          while (millis() - t < dwell) {
            if (rxFlag) {
              rxFlag = false;
              uint8_t b[256];
              size_t n = radio.getPacketLength();
              int r = radio.readData(b, n);
              Serial.printf("PKT f=%.1f bw=%.0f sf=%d sw=0x%02X len=%u rssi=%.0f snr=%.1f st=%d :", f, bw, sf, sw,
                            (unsigned)n, radio.getRSSI(), radio.getSNR(), r);
              for (size_t i = 0; i < n; i++) Serial.printf(" %02X", b[i]);
              Serial.println();
              if (r == 0) {  // found the settings, collect a few more then stop
                for (int k = 0; k < 4; k++) {
                  rxFlag = false;
                  radio.startReceive();
                  unsigned long t2 = millis();
                  while (!rxFlag && millis() - t2 < 8000) delay(5);
                  if (!rxFlag) break;
                  n = radio.getPacketLength();
                  radio.readData(b, n);
                  Serial.printf("PKT len=%u rssi=%.0f :", (unsigned)n, radio.getRSSI());
                  for (size_t i = 0; i < n; i++) Serial.printf(" %02X", b[i]);
                  Serial.println();
                }
                return;
              }
              radio.startReceive();
            }
            delay(2);
          }
        }
  Serial.println("no LoRa packets heard");
}

void setup() {
  Serial.begin(115200);
  pinMode(36, OUTPUT);
  digitalWrite(36, LOW);  // Vext on (OLED + external sensors)
  pinMode(21, OUTPUT);
  digitalWrite(21, HIGH);
  delay(1500);
  Serial.println("\n#### TOTEM PROBE START");
  battery();
  scanI2C();
  scanGPS();
  sniffLoRa();
  Serial.println("#### TOTEM PROBE DONE");
}

void loop() { delay(1000); }

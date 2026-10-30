#pragma once
#include <Wire.h>

// 간단한 BNO055 드라이버: NDOF 융합 방위, 보정 상태, 보정값 저장/복원
class Bno055 {
 public:
  static const int CAL_LEN = 22;

  bool begin(TwoWire &w, uint8_t address) {
    wire = &w;
    addr = address;
    if (read8(0x00) != 0xA0) {
      delay(700);  // 전원 켜고 약 650ms 뒤부터 응답
      if (read8(0x00) != 0xA0) return false;
    }
    configMode();
    write8(0x3F, 0x20);  // SYS_TRIGGER: 리셋
    delay(700);
    for (int i = 0; i < 20 && read8(0x00) != 0xA0; i++) delay(50);
    write8(0x3E, 0x00);  // 일반 전원 모드
    write8(0x07, 0x00);  // 레지스터 페이지 0
    write8(0x3B, 0x00);  // 도 단위, 시계 방향으로 커짐
    write8(0x3F, 0x00);
    delay(10);
    return true;
  }

  void start() {
    write8(0x3D, 0x0C);  // NDOF
    delay(25);
  }

  void restore(const uint8_t *cal) {
    configMode();
    writeN(0x55, cal, CAL_LEN);
    start();
  }

  bool save(uint8_t *cal) {
    configMode();
    bool ok = readN(0x55, cal, CAL_LEN);
    start();
    return ok;
  }

  // 화면이 꺼져 있을 때: 약 12mA → 0.04mA. 깨우면 NDOF로 돌아감 (보정값은 유지)
  void suspend() {
    configMode();
    write8(0x3E, 0x02);
  }
  void resume() {
    for (int i = 0; i < 3; i++) {  // 잠든 칩은 첫 I2C에 응답하지 않을 수 있어 확인하며 다시 씀
      write8(0x3E, 0x00);
      delay(10);
      if (read8(0x3E) == 0x00) break;
    }
    configMode();
    start();
  }

  // 0~360, 자북 기준
  float heading() {
    uint8_t b[2];
    if (!readN(0x1A, b, 2)) return -1;
    return (int16_t)(b[0] | b[1] << 8) / 16.0f;
  }

  // 비트: 시스템 7-6, 자이로 5-4, 가속도 3-2, 지자기 1-0 (각 0~3)
  uint8_t calib() { return read8(0x35); }

 private:
  TwoWire *wire = nullptr;
  uint8_t addr = 0x29;

  void configMode() {
    write8(0x3D, 0x00);
    delay(25);
  }
  void write8(uint8_t reg, uint8_t v) { writeN(reg, &v, 1); }
  void writeN(uint8_t reg, const uint8_t *v, int n) {
    wire->beginTransmission(addr);
    wire->write(reg);
    wire->write(v, n);
    wire->endTransmission();
  }
  uint8_t read8(uint8_t reg) {
    uint8_t v = 0;
    readN(reg, &v, 1);
    return v;
  }
  bool readN(uint8_t reg, uint8_t *v, int n) {
    wire->beginTransmission(addr);
    wire->write(reg);
    if (wire->endTransmission(false) != 0) return false;
    if (wire->requestFrom((int)addr, n) != n) return false;
    for (int i = 0; i < n; i++) v[i] = wire->read();
    return true;
  }
};

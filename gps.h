#pragma once
#include <Arduino.h>

// u-blox 절전 모드 (UBX-CFG-RXM lpMode=1). 보드 TX가 GPS RX에 이어져 있어야 함
inline void gpsPowerSave(Stream &s) {
  uint8_t m[] = {0xB5, 0x62, 0x06, 0x11, 0x02, 0x00, 0x08, 0x01, 0, 0};
  for (int i = 2; i < 8; i++) {
    m[8] += m[i];
    m[9] += m[8];
  }
  s.write(m, sizeof(m));
}

// 작은 NMEA 파서: RMC(위치, 수신 여부)와 GGA(위성 수, HDOP)만 읽음
class NmeaParser {
 public:
  int32_t lat = 0, lon = 0;  // 1e-7도
  uint8_t sats = 0;
  float hdop = 99.9f;
  uint32_t lastFixMs = 0;
  bool everFixed = false;

  bool valid() const { return everFixed && millis() - lastFixMs < 3000; }

  void feed(char c) {
    if (c == '$') n = 0;
    if (c == '\r') return;
    if (c == '\n') {
      buf[n] = 0;
      if (n > 6 && checksumOk()) parse();
      n = 0;
      return;
    }
    if (n < sizeof(buf) - 1) buf[n++] = c;
  }

 private:
  char buf[100];
  size_t n = 0;
  const char *f[20];
  int nf = 0;

  bool checksumOk() {
    char *star = strchr(buf, '*');
    if (buf[0] != '$' || !star) return false;
    uint8_t x = 0;
    for (char *p = buf + 1; p < star; p++) x ^= *p;
    return strtol(star + 1, nullptr, 16) == x;
  }

  void split() {
    nf = 0;
    char *p = buf;
    *strchr(buf, '*') = 0;
    f[nf++] = p;
    while ((p = strchr(p, ',')) && nf < 20) {
      *p++ = 0;
      f[nf++] = p;
    }
  }

  static bool coord(const char *v, const char *hemi, int degDigits, int32_t &out) {
    if (!*v || !*hemi) return false;
    char d[4] = {0};
    memcpy(d, v, degDigits);
    double deg = atoi(d) + atof(v + degDigits) / 60.0;
    if (*hemi == 'S' || *hemi == 'W') deg = -deg;
    out = (int32_t)llround(deg * 1e7);
    return true;
  }

  void parse() {
    bool rmc = !strncmp(buf + 3, "RMC", 3), gga = !strncmp(buf + 3, "GGA", 3);
    if (!rmc && !gga) return;
    split();
    if (rmc && nf > 6 && f[2][0] == 'A') {
      int32_t la, lo;
      if (coord(f[3], f[4], 2, la) && coord(f[5], f[6], 3, lo)) {
        lat = la;
        lon = lo;
        lastFixMs = millis();
        everFixed = true;
      }
    } else if (gga && nf > 8) {
      sats = atoi(f[7]);
      if (*f[8]) hdop = atof(f[8]);
    }
  }
};

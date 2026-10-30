#pragma once
#include <stdint.h>

#define NAME_LEN 12  // 이름 최대 바이트 (한글 4글자)

// ---- LoRa: 토템 <-> 토템 ----
// 프레임: [netId 1][senderId 2][bootNonce 4][counter 4] [Payload, AES-128-GCM 암호화] [tag 8]
// 앞 11바이트 헤더는 인증만 하고(AAD), 그중 뒤 10바이트 + 0 두 바이트가 IV.
struct __attribute__((packed)) Payload {
  uint8_t magic;    // 'T'
  uint8_t ver;      // 1
  uint8_t flags;    // PF_*
  uint8_t sats;
  uint8_t bat;      // %
  int32_t lat, lon; // 1e-7도, PF_FIX일 때만 유효
  uint16_t meetVer; // 더 새로운 버전을 양쪽이 따름
  int32_t mlat, mlon;
  uint8_t msgSeq;   // 상대가 받았다고 할 때까지 0이 아닌 값
  uint8_t msgType;  // 1 보고싶어, 2 어디야, 3 곧 갈게, 4 깜박여 줘 (찾는 중)
  uint8_t ackSeq;   // 상대에게서 마지막으로 받은 msgSeq
  uint16_t acc;     // 위치 오차 (m). 폰 위치는 폰이 알려 준 값, GPS는 HDOP로 추정
  char name[NAME_LEN]; // 내 이름 (UTF-8, 꽉 차면 끝에 0 없음)
};
static_assert(sizeof(Payload) == 40, "payload size");

#define PAYLOAD_VER 2
enum { PF_FIX = 1, PF_MEET = 2, PF_PHONE = 4, PF_FIND = 8, PF_CHARGING = 16 };  // PF_FIND: 실내 찾기 중 (상대도 1초마다 송신), PF_CHARGING: 충전 중

#define FRAME_HDR 11
#define FRAME_TAG 8
#define FRAME_LEN (FRAME_HDR + sizeof(Payload) + FRAME_TAG)

// ---- LoRa 장거리 매너 모드: 짧은 프레임 (한 번 송신 0.4초 안쪽) ----
// [netId 1][senderId 2][bootCnt 2][counter 2] [본문, AES-128-GCM] [tag 4]
// 본문 길이로 종류를 구분: 비컨 13, 만남 장소 11, 이름 12
#define MF_HDR 7
#define MF_TAG 4
struct __attribute__((packed)) MBeacon {
  uint8_t flags;    // MBF_* | (msgType-1) << 4 | 오차 등급 << 6
  uint8_t bat;
  int32_t lat, lon;
  int8_t snr;       // 상대 신호를 내가 받은 SNR (dB). MB_NO_SNR = 못 받음 → 상대 출력 조절용
  uint8_t msgSeq, ackSeq;
};
struct __attribute__((packed)) MMeet {
  uint16_t meetVer;
  int32_t mlat, mlon;
  uint8_t set;
};
static_assert(sizeof(MBeacon) == 13 && sizeof(MMeet) == 11 && NAME_LEN == 12, "manner frame sizes");
enum { MBF_FIX = 1, MBF_PHONE = 2, MBF_FIND = 4, MBF_CHARGING = 8 };
#define MB_NO_SNR (-128)

// ---- BLE: 토템 <-> totem-map 앱 ----
#define BLE_SVC_UUID "6e7a0001-7c3a-4b8e-9f2d-544f54454d31"
#define BLE_STATUS_UUID "6e7a0002-7c3a-4b8e-9f2d-544f54454d31"  // read/notify, STATUS_LEN 바이트
#define BLE_CMD_UUID "6e7a0003-7c3a-4b8e-9f2d-544f54454d31"     // write

// 0~41은 예전 앱과 같음. 42~53 실내 찾기, 54~65 내 이름, 66~77 상대 이름 (앱은 byteLength로 구분)
#define STATUS_LEN 78
#define STATUS_VER 4

// 찾기 플래그 (42번 바이트)
enum { FF_FINDING = 1, FF_FRIEND_FINDING = 2, FF_SCAN_READY = 4, FF_FRIEND_CHARGING = 8 };

// 상태 플래그 (1번 바이트)
enum { SF_MYFIX = 1, SF_SEEN = 2, SF_FRFIX = 4, SF_MEET = 8, SF_COMPASS = 16, SF_PHONE = 64, SF_CHARGING = 128 };

// 명령 (첫 바이트)
enum {
  CMD_MEET = 1,      // + lat i32 + lon i32
  CMD_MEET_CLEAR = 2,
  CMD_MSG = 3,       // + type u8
  CMD_SETTINGS = 4,  // + farM u16 + screenMin u8
  CMD_PHONE_LOC = 5, // + lat i32 + lon i32 + 정확도 u16 (m)
  CMD_FIND = 6,      // + 0 끄기 / 1 켜기(3분) / 2 방향 다시 재기
  CMD_NAME = 7       // + 이름 UTF-8 (최대 NAME_LEN 바이트)
};

struct Cmd {
  uint8_t len;
  uint8_t d[20];
};

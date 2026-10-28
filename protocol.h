#pragma once
#include <stdint.h>

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
  uint8_t msgType;  // 1 보고싶어, 2 어디야, 3 곧 갈게
  uint8_t ackSeq;   // 상대에게서 마지막으로 받은 msgSeq
};
static_assert(sizeof(Payload) == 26, "payload size");

enum { PF_FIX = 1, PF_MEET = 2, PF_PHONE = 4 };

#define FRAME_HDR 11
#define FRAME_TAG 8
#define FRAME_LEN (FRAME_HDR + sizeof(Payload) + FRAME_TAG)

// ---- BLE: 토템 <-> totem-map 앱 ----
#define BLE_SVC_UUID "6e7a0001-7c3a-4b8e-9f2d-544f54454d31"
#define BLE_STATUS_UUID "6e7a0002-7c3a-4b8e-9f2d-544f54454d31"  // read/notify, 42바이트
#define BLE_CMD_UUID "6e7a0003-7c3a-4b8e-9f2d-544f54454d31"     // write

#define STATUS_LEN 42
#define STATUS_VER 3

// 상태 플래그 (1번 바이트)
enum { SF_MYFIX = 1, SF_SEEN = 2, SF_FRFIX = 4, SF_MEET = 8, SF_COMPASS = 16, SF_PHONE = 64 };

// 명령 (첫 바이트)
enum {
  CMD_MEET = 1,      // + lat i32 + lon i32
  CMD_MEET_CLEAR = 2,
  CMD_MSG = 3,       // + type u8
  CMD_SETTINGS = 4,  // + farM u16 + screenMin u8
  CMD_PHONE_LOC = 5  // + lat i32 + lon i32 + 정확도 u16 (m)
};

struct Cmd {
  uint8_t len;
  uint8_t d[20];
};

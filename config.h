#pragma once
// 토템 나침반 하드웨어: Heltec WiFi LoRa 32 V3.1/V3.2 (ESP32-S3 + SX1262)
// 아래 핀은 실제 보드에서 tools/probe 스케치로 확인한 값입니다.
// 배선이 다르면 tools/probe 를 올려서 핀을 찾은 뒤 여기만 고치면 됩니다.

#define FW_VERSION "4.6"

// 보드
#define PIN_VEXT 36       // LOW = Vext 켜짐 (OLED, GPS, BNO055 전원)
#define PIN_LED 35
#define PIN_BTN 0         // PRG 버튼, 누르면 LOW
#define PIN_BAT_ADC 1
#define PIN_BAT_CTRL 37   // HIGH일 때 배터리 전압을 읽음 (V3.1 이상. LOW면 0V로 나옴)
#define BAT_DIVIDER 4.9f
#define BAT_JUMP_V 0.08f          // 이만큼 갑자기 오르면 충전기 꽂음, 내리면 뺌
#define BAT_SETTLE_MS 60000       // 충전기를 뺀 뒤 전압이 자리 잡는 시간
#define BAT_UNPLUG_V 0.04f        // 충전 중 최고 전압보다 이만큼 낮게 30초면 충전기 뺀 것

// 보드 내장 OLED (SSD1306 128x64, I2C 0x3C)
#define PIN_OLED_SDA 17
#define PIN_OLED_SCL 18
#define PIN_OLED_RST 21

// BNO055 나침반 (두 번째 I2C)
#define PIN_BNO_SDA 4
#define PIN_BNO_SCL 5
#define BNO_ADDR 0x29     // 모듈에 따라 0x28

// GPS 모듈 (NMEA 출력)
#define PIN_GPS_RX 6      // 보드 RX <- GPS TX
#define GPS_BAUD 9600

// SX1262
#define LORA_NSS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13

// 쓰는 나라에 맞게 하나만 고르세요. 두 토템이 같아야 서로 통신됨.
#define REGION_US  // 미국 (FCC Part 15)
// #define REGION_KR  // 한국 (KR920)

#if defined(REGION_US)
// 902~928 MHz. 한 채널에 고정하려면 대역폭 500 kHz(디지털 변조 규정) → 최대 30 dBm.
// SF11/BW500은 SF9/BW125와 송신 시간이 같고 감도는 약 1 dB 낮지만, 출력이 9 dB 높아 더 멀리 감.
// 919.0 MHz: LoRaWAN US915 채널(902.3~914.9 올림, 923.3~927.5 내림) 사이의 빈 곳
#define LORA_FREQ 919.0f
#define LORA_BW 500.0f
#define LORA_SF 11
#define LORA_POWER 17     // dBm. 22(최대)에서 버튼 누를 때 재시작이 있어 낮춤 (FCC 한도 30 dBm)
#elif defined(REGION_KR)
// 920.9~923.3 MHz, 200 kHz 간격. 922.1 MHz는 채널 위에 있음.
#define LORA_FREQ 922.1f
#define LORA_BW 125.0f
#define LORA_SF 9
#define LORA_POWER 13     // dBm, KR920 한도(14 dBm) 아래로
#endif
#define LORA_CR 5
#define LORA_SYNC 0x12    // 사설 네트워크
#define LORA_PREAMBLE 8
#define LORA_TCXO 1.8f

#define TX_INTERVAL_MS 15000      // 배터리 절약. 찾는 중에는 FIND_TX_INTERVAL_MS
#define TX_JITTER_MS 3000         // 두 토템이 같은 순간에 보내지 않게

// 배터리 절약
#define CPU_MHZ 80                // BLE가 되는 최저 속도
#define LOOP_IDLE_MS 10
#define SCREEN_MIN_DEFAULT 1      // 화면 자동 꺼짐 (분, 0 = 안 꺼짐). 앱에서 바꾼 값이 있으면 그 값
#define BLE_ADV_MIN 800           // x0.625 ms = 0.5초
#define BLE_ADV_MAX 1600          // 1초

// 나침반
#define MAG_DECLINATION -8.5f  // 한국은 편각 약 8.5도 서편: 진북 = 자북 + 편각
#define HEADING_OFFSET 0.0f    // 보드를 돌려서 달았으면 보정

// 시간
#define PHONE_LOC_TTL_MS 15000   // 앱이 5초마다 다시 보냄
#define FRIEND_VALID_S 60
#define ALERT_TIMEOUT_MS 60000

// 실내 찾기
#define FIND_TX_INTERVAL_MS 1200  // 찾는 중에는 양쪽 다 이 간격으로 송신
#define FIND_DURATION_MS 180000   // 3분 뒤 자동으로 끝 (앱이 켜 두면 계속 연장)
#define SCAN_BINS 12              // 방향 재기: 30도씩 12칸
#define WEDGE_MIN_DEG 10          // 방향 오차가 이보다 작으면 부채꼴 대신 화살표
#define SCAN_WEDGE_DEG 30         // 신호 세기로 잰 방향의 부채꼴 폭 (±도)
#define SCAN_MIN_SPREAD 4        // 가장 센 쪽과 약한 쪽 차이가 이 dB 이상이어야 화살표 표시

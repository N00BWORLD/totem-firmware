#pragma once
// 토템 나침반 하드웨어: Heltec WiFi LoRa 32 V3.1/V3.2 (ESP32-S3 + SX1262)
// 아래 핀은 실제 보드에서 tools/probe 스케치로 확인한 값입니다.
// 배선이 다르면 tools/probe 를 올려서 핀을 찾은 뒤 여기만 고치면 됩니다.

#define FW_VERSION "4.18"

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
// 남은 시간·충전 시간 추정 (실측 전 추정값). 배터리 용량은 USB로 "bat 2000"처럼 바꿈
#define BAT_MAH_DEFAULT 1000
#define BAT_AVG_MA 70             // 평균 소비 전류 추정. 20분 넘게 쓰면 실제 줄어든 %로 다시 계산
#define CHARGE_MA 300             // 보드 충전 전류 추정

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
#define PIN_GPS_TX -1     // 보드 TX -> GPS RX. 선을 이으면 GPS 절전 명령을 보냄 (-1 = 선 없음)
#define GPS_BAUD 9600

// SX1262
#define LORA_NSS 8
#define LORA_DIO1 14
#define LORA_RST 12
#define LORA_BUSY 13

// 하나만 고르세요. 두 토템이 같아야 서로 통신됨.
#define REGION_LONG  // 기본: 922.7 MHz / 62.5 kHz / 22 dBm / LBT / 1회 약 4초 (장거리 고정)
// #define REGION_US  // 미국 (FCC Part 15)
// #define REGION_KR  // 한국 (KR920, EIRP 14 dBm)

#if defined(REGION_LONG)
// 한국 KR920 채널(922.7 MHz)과 LBT·1회 4초 규칙을 따르되 출력은 칩 최대 22 dBm.
// 주의: 22 dBm은 한국 한도(EIRP 14 dBm)를 넘고, 미국에서 62.5 kHz 고정 채널은 30 dBm 규정(500 kHz 이상) 대상이 아님.
// 늘 장거리 모드(짧은 프레임). 시스템 화면 길게 누르기로 끌 수 없음
#define MANNER_ONLY
#define LORA_FREQ 922.7f
#define LORA_BW 62.5f     // 시작할 때만 쓰임 (늘 장거리 모드)
#define LORA_SF 11
#define LORA_POWER 22
// SF12/BW62.5/CR4/8: 비컨(24바이트) 1회 약 3.95초 (4초 이내), 만남 장소·이름은 그보다 짧음
#define MANNER_BW 62.5f
#define MANNER_SF 12
#define MANNER_CR 8
#define MANNER_POWER_MAX 22
#define MANNER_POWER_MIN 22           // 출력 고정 (상대가 잘 들려도 낮추지 않음)
#define MANNER_TX_INTERVAL_MS 20000   // 20초 + 0~4초 랜덤 (TX_JITTER_MS). 한 번에 약 4초
#define MANNER_FIND_INTERVAL_MS 10000 // 실내 찾기 중
#define MANNER_DUTY_PCT 30            // 한 시간 송신 시간 상한 (30%. 22초 간격 비컨 약 18% + GPS 없을 때 와이파이 약 6%)
#define MANNER_EXTRA_EVERY 16         // 한 번에 4초라 다시 보내기는 드물게
#elif defined(REGION_US)
// 902~928 MHz. 한 채널에 고정하려면 대역폭 500 kHz(디지털 변조 규정) → 최대 30 dBm.
// SF11/BW500은 SF9/BW125와 송신 시간이 같고 감도는 약 1 dB 낮지만, 출력이 9 dB 높아 더 멀리 감.
// 919.0 MHz: LoRaWAN US915 채널(902.3~914.9 올림, 923.3~927.5 내림) 사이의 빈 곳
#define LORA_FREQ 919.0f
#define LORA_BW 500.0f
#define LORA_SF 11
#define LORA_POWER 22     // dBm, 칩 최대 (FCC 한도 30 dBm). v4.6의 "버튼 재시작"은 화면 꺼짐 버그였음(v4.7에서 고침)
// 장거리 매너 모드: FCC가 허용하는 범위에서 가장 멀리. SF12/BW500/CR4/8, 칩 최대 22dBm (FCC 한도 30dBm)
// 한 번 송신 약 0.43초 (짧은 프레임). 상대가 잘 들리면 출력을 스스로 낮춤
#define MANNER_BW 500.0f
#define MANNER_SF 12
#define MANNER_CR 8
#define MANNER_POWER_MAX 22
#define MANNER_TX_INTERVAL_MS 25000   // 앱은 40초 넘으면 신호 없음으로 봄
#define MANNER_FIND_INTERVAL_MS 4000  // 실내 찾기 중
#define MANNER_DUTY_PCT 2             // 한 시간 송신 시간 상한 (2% = 72초)
#define MANNER_EXTRA_EVERY 8          // 만남 장소는 비컨 8번마다, 이름은 16번마다 다시 보냄
#elif defined(REGION_KR)
// 920.9~923.3 MHz, 200 kHz 간격. 922.1 MHz는 채널 위에 있음.
#define LORA_FREQ 922.7f  // 922.1~922.5는 LoRaWAN 기본 채널이라 붐빔, 923.1은 Z-Wave
#define LORA_BW 125.0f
#define LORA_SF 9
#define LORA_POWER 11     // dBm. 스틱 안테나 3dBi를 더해도 EIRP 14dBm (KR920 한도)
// 장거리 매너 모드: SF12/BW62.5, 한 번 송신 약 3초 (짧은 프레임).
// 고시 제8조④6호 나목: LBT(5ms 이상, -65dBm 이하)를 하면 1회 4초 이내 + 50ms 이상 휴지. 0.4초는 LBT 안 쓰는 기기 기준
#define MANNER_BW 62.5f
#define MANNER_SF 12
#define MANNER_CR 5
#define MANNER_POWER_MAX 11
#define MANNER_TX_INTERVAL_MS 60000   // 한 번에 3초라 간격을 넓힘 (앱은 40초 넘으면 "마지막 신호"로 표시)
#define MANNER_FIND_INTERVAL_MS 10000 // 실내 찾기 중
#define MANNER_DUTY_PCT 6             // 한 시간 송신 시간 상한 (6% = 216초, 1분 간격 비컨이 약 5%)
#define MANNER_EXTRA_EVERY 16         // 한 번에 3초라 다시 보내기는 드물게
#endif
#define LORA_CR 5
#define LORA_SYNC 0x12    // 사설 네트워크
#define LORA_PREAMBLE 8
#define LORA_TCXO 1.8f

// 장거리 매너 모드: 남이 쓰고 있으면 안 보내고, 짧게, 가끔, 필요한 만큼만 세게
#define MANNER_LBT_DBM -80            // 이보다 센 신호가 있으면 채널 사용 중 (한국 기준 -65보다 엄격)
#ifndef MANNER_POWER_MIN
#define MANNER_POWER_MIN 0            // 상대가 잘 들리면 이만큼까지 낮춤
#endif

#define TX_INTERVAL_MS 15000      // 배터리 절약. 찾는 중에는 FIND_TX_INTERVAL_MS
#define TX_JITTER_MS 4000         // 0~4초 랜덤: 두 토템이 같은 순간에 보내 서로 묻히지 않게

// 배터리 절약
#define CPU_MHZ 80                // BLE가 되는 최저 속도
#define LOOP_IDLE_MS 10
#define SCREEN_MIN_DEFAULT 1      // 화면 자동 꺼짐 (분, 0 = 안 꺼짐). 앱에서 바꾼 값이 있으면 그 값
#define BLE_ADV_MIN 800           // x0.625 ms = 0.5초
#define BLE_ADV_MAX 1600          // 1초

// 와이파이 위치 (GPS가 안 잡힐 때)
#define WIFI_SCAN_MS 60000        // 스캔 간격
#define WIFI_EVERY 3              // 비컨 3번에 한 번 와이파이 프레임을 덧붙임

// 나침반
#define MAG_DECLINATION -8.5f  // 한국은 편각 약 8.5도 서편: 진북 = 자북 + 편각
#define HEADING_OFFSET 0.0f    // 보드를 돌려서 달았으면 보정

// 시간
#define PHONE_LOC_TTL_MS 15000   // 앱이 5초마다 다시 보냄
#define FRIEND_VALID_S 60  // 매너 모드는 비컨 간격의 2.5배 (friendValidS)
#define ALERT_TIMEOUT_MS 60000

// 실내 찾기
#define FIND_TX_INTERVAL_MS 1200  // 찾는 중에는 양쪽 다 이 간격으로 송신
#define FIND_DURATION_MS 180000   // 3분 뒤 자동으로 끝 (앱이 켜 두면 계속 연장)
#define SCAN_BINS 12              // 방향 재기: 30도씩 12칸
#define WEDGE_MIN_DEG 10          // 방향 오차가 이보다 작으면 부채꼴 대신 화살표
#define SCAN_WEDGE_DEG 30         // 신호 세기로 잰 방향의 부채꼴 폭 (±도)
#define SCAN_MIN_SPREAD 4        // 가장 센 쪽과 약한 쪽 차이가 이 dB 이상이어야 화살표 표시

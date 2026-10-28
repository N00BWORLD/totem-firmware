#pragma once
// 토템 나침반 하드웨어: Heltec WiFi LoRa 32 V3.1/V3.2 (ESP32-S3 + SX1262)
// 아래 핀은 실제 보드에서 tools/probe 스케치로 확인한 값입니다.
// 배선이 다르면 tools/probe 를 올려서 핀을 찾은 뒤 여기만 고치면 됩니다.

#define FW_VERSION "4.1"

// 보드
#define PIN_VEXT 36       // LOW = Vext 켜짐 (OLED, GPS, BNO055 전원)
#define PIN_LED 35
#define PIN_BTN 0         // PRG 버튼, 누르면 LOW
#define PIN_BAT_ADC 1
#define PIN_BAT_CTRL 37   // HIGH일 때 배터리 전압을 읽음 (V3.1 이상. LOW면 0V로 나옴)
#define BAT_DIVIDER 4.9f

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

// 한국 KR920 대역 (920.9~923.3 MHz, 200 kHz 간격). 922.1 MHz는 채널 위에 있음.
// 다른 나라에서 쓰면 그 나라에서 허용된 주파수와 출력으로 바꾸세요.
#define LORA_FREQ 922.1f
#define LORA_BW 125.0f
#define LORA_SF 9
#define LORA_CR 5
#define LORA_SYNC 0x12    // 사설 네트워크
#define LORA_POWER 13     // dBm, KR920 한도(14 dBm) 아래로
#define LORA_PREAMBLE 8
#define LORA_TCXO 1.8f

#define TX_INTERVAL_MS 5000
#define TX_JITTER_MS 1500

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
#define SCAN_MIN_SPREAD 4         // 가장 센 쪽과 약한 쪽 차이가 이 dB 이상이어야 화살표 표시

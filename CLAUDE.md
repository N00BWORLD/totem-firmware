# 작업 메모 (Claude / 기여자용)

사용자용 설명, 배선, 통신 규격은 README.md에 있음. 여기는 작업할 때 알아야 할 것만.

## 규칙
- 화면 문구와 주석은 한국어. OLED 한글은 `FONT_KO`(u8g2 gulim11_t_korean2)로 `drawUTF8`/`ko()` 사용. 화살표 같은 특수문자는 폰트에 없을 수 있음
- 시리얼 상태 줄(`hdg=... fix=... | friend ... | bat=...`)은 영문 형식 유지
- BLE STATUS/CMD 형식은 [totem-map](https://github.com/N00BWORLD/totem-map) `index.html`의 `parse()`/`send()`와 같아야 함. 바꾸면 양쪽 같이
- LoRa `Payload`를 바꾸면 `Payload.ver`를 올리고 두 대 다 업데이트
- `secrets.h`는 절대 커밋하지 않음 (.gitignore)

## 빌드
- core `esp32:esp32` 3.3.12, RadioLib 7.8.1, U8g2 2.36.19에서 확인 (2026-10)
- `arduino-cli compile --fqbn esp32:esp32:heltec_wifi_lora_32_V3 --upload -p <포트> .`
- 업로드 후 시리얼 115200에서 `LoRa=0(0=OK)  BNO055=OK`, `BLE name=TOTEM-xxxx` 확인

## 구조
- `loop()`이 모든 것을 차례로 처리. BLE 콜백은 다른 태스크라 명령은 `cmdQueue`로 넘김
- 위치: GPS 우선, 없으면 앱이 보낸 폰 위치(15초 유효)
- 만남 장소는 `meetVer`가 큰 쪽이 이김. 메시지는 ack(`ackSeq`) 올 때까지 `msgSeq`를 실어 보냄
- NVS: `totem`(farM, scrMin, name), `bnocal`(cal 22바이트)
- 실내 찾기: `findUntil`이 지나면 꺼짐. 받을 때마다 `addSignal()`이 평균·추세·방향별(30도 12칸) 신호를 갱신

## 다음 할 일
- [ ] 야외에서 GPS·방향 화살표·거리 실측, 장착 방향에 따라 `HEADING_OFFSET` 확인
- [ ] 배터리 절약 (화면 꺼짐 때 GPS/CPU 절전)
- [x] 신호 세기로 방향 찾기 (v4.1 실내 찾기)

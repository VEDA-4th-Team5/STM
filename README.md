# STM — 주차 감지 노드 펌웨어

주차장 감시 및 제어 시스템의 **STM32 펌웨어**입니다.
주차면 점유(홀 센서)와 화재(불꽃 센서)를 감지해 LoRa 로 라즈베리파이 게이트웨이에 보고합니다.

VEDA 4기 5팀 프로젝트. 관련 저장소: [Pi_Server](https://github.com/VEDA-4th-Team5/Pi_Server) · [Qt_Client](https://github.com/VEDA-4th-Team5/Qt_Client)

---

## 시스템 구성

```
  ┌──────────────┐                        ┌─────────────────┐
  │  STM1 노드    │ ──┐                    │  Raspberry Pi   │
  │  홀2 + 화재1  │   │                    │   Pi_Server     │
  └──────────────┘   ├─ LoRa 920MHz ────▶ │        │        │
  ┌──────────────┐   │    (양방향)          └────────┼────────┘
  │  STM2 노드    │ ──┘                             ▼
  │  홀2          │                            Qt_Client
  └──────────────┘
```

노드 두 대는 **같은 펌웨어**입니다. 헤더 한 줄로 STM1/STM2 를 고릅니다.

| | 홀 센서 | 불꽃 센서 | 센서 ID |
|---|---|---|---|
| STM1 | 2 | 1 | HALL01, HALL02, FLAME01 |
| STM2 | 2 | – | HALL03, HALL04 |

센서 ID 는 노드를 가로질러 전역 고유합니다. Pi 의 `config/parking_slots.json` 과 맞춘 값입니다.

---

## 현재 구현

- **홀 센서** — D0 오픈컬렉터 GPIO 직결(LOW = OCCUPIED), EXTI 인터럽트. 채널별 쿨다운으로 채터링 억제
- **불꽃 센서** — DFR0076 ADC + CMSIS-DSP FFT. 1~20Hz flicker 에너지 **또는** DC 상승(delta) 하이브리드 판정, 5창 중 3 다수결 디바운스
- **baseline 추종** — 깨끗한 창에서만 천천히 따라가 조명 변화·센서 드리프트를 흡수 (EVDA-218)
- **LoRa 전송** — EBYTE E220-900T30D, 920MHz. 페이로드 v1.1 (프레임 version `0x02`)
- **duty cycle 리미터** — 전파법 한도 내 송신 보장. 초과분은 버림
- **하트비트** — 10초 주기, 노드별로 반 주기 어긋내 half-duplex 충돌 회피 (EVDA-46)
- **화재 유실 대책** — DETECTED 를 같은 seq 로 2회 더 재전송 (EVDA-124)
- **번호판 조명 LED** — 하행 명령 수신부 (EVDA-194)

### 전송 포맷

```
SENSOR:<node>:<sensor>:<state>:<seq>
FIRE:<node>:<sensor>:<state>:<seq>:<energy>

SENSOR:STM1:HALL01:OCCUPIED:142
FIRE:STM1:FLAME01:DETECTED:144:43.70
```

`<seq>` 는 **노드 단일 카운터**입니다. 홀·화재가 같은 스트림을 씁니다.
디바운스는 STM32 가 책임지고 Pi 는 재디바운스하지 않습니다(팀 계약).

---

## 소스 구조

```
STM/                            STM32CubeIDE 프로젝트
  Core/Inc/board_config.h       ★ STM1 / STM2 선택
  Core/Inc/sensor_protocol.h    페이로드 v1.1 규격, 주기 상수
  Core/Inc/lora_frame.h         LoRa 바이너리 프레임 규격
  Core/Src/freertos.c           태스크 (홀 / 화재 FFT / LoRa 송수신)
  Core/Src/sensor_protocol.c    문자열 인코딩, seq 관리
  README.md                     클럭·핀맵·신호처리·LoRa 상세
docs/                           프로토콜 / 전파법 / Pi 연동 문서
misc/
```

---

## 빌드

STM32CubeIDE 2.2.0 으로 `STM/` 를 엽니다. GCC 툴체인, CMSIS-DSP 링크.

대상 보드는 `STM/Core/Inc/board_config.h` 에서 고릅니다.

```c
#define BOARD_SELECT  BOARD_STM1     /* 또는 BOARD_STM2 */
```

> ⚠️ **CubeMX 로 코드를 재생성하면 `MX_FREERTOS_Init()` 의 `osThreadNew` 호출이 사라집니다.**
> CubeMX 2.2.0 에서 재현되는 알려진 버그입니다. 재생성 직후 반드시 해당 함수를 확인하고
> 태스크·큐·세마포어 생성 코드를 복원하세요. 자세한 건 [`STM/README.md`](STM/README.md) "알려진 CubeMX 버그" 절.

---

## 실행

전원을 넣으면 USART2(ST-Link VCP, **115200 8N1**)로 부팅 배너가 나옵니다.
어느 보드로 구워졌는지 여기서 확인합니다.

```
# STM2  build Aug 14 2026 16:20:11
# payload v1.1 (frame ver 0x02) / node STM2
```

전송 경로는 `sensor_protocol.h` 의 두 플래그로 고릅니다. 둘 다 1이면 같은 이벤트가
유선·무선으로 동시에 나가 A/B 비교가 됩니다.

```c
#define SENSOR_TX_UART 1   /* USART2 콘솔 / 직결 */
#define SENSOR_TX_LORA 1   /* E220 LoRa 프레임 (Pi 링크) */
```

불꽃 임계값을 재튜닝할 때는 `FLAME_DEBUG_ENERGY` 를 켜면 창마다
`raw / baseline / energy / delta / vote` 가 주석 줄로 나옵니다.

---

## 현재 한계

- **Pi 시퀀스 가드** — 노드가 켜진 지 약 1시간 이내에 리셋되면 그 노드의 프레임이 전부 버려집니다. Pi 쪽 수정 필요 → [`docs/PI_SERVER_SEQ_GUARD_REBOOT.md`](docs/PI_SERVER_SEQ_GUARD_REBOOT.md)
- **불꽃 판정 히스테리시스 없음** — 신호가 임계 근처에 걸리면 DETECTED/CLEARED 가 반복될 수 있습니다. 진입·해제 문턱 분리 필요
- **태양광 환경 오탐 테스트 미완** — 실내 조건(형광등·모니터·손 흔들기)까지만 검증됨
- **TDOA 마이크** — 미착수

---

## 문서

| 문서 | 내용 |
|---|---|
| [`STM/README.md`](STM/README.md) | 클럭·핀맵·신호 처리·태스크·LoRa 상세 |
| [`docs/LORA_PAYLOAD_V1_1_FOR_PI.md`](docs/LORA_PAYLOAD_V1_1_FOR_PI.md) | 센서 페이로드 v1.1 규격 |
| [`docs/LORA_RF_COMPLIANCE_KR.md`](docs/LORA_RF_COMPLIANCE_KR.md) | 920MHz 전파법 duty cycle 검토 |
| [`docs/PI_LORA_SETUP_GUIDE.md`](docs/PI_LORA_SETUP_GUIDE.md) | Pi 쪽 LoRa 구축 가이드 |
| [`docs/PI_SERVER_LORA_V11_PATCH.md`](docs/PI_SERVER_LORA_V11_PATCH.md) | Pi_Server v1.1 수신 대응 요청 |
| [`docs/PI_SERVER_SEQ_GUARD_REBOOT.md`](docs/PI_SERVER_SEQ_GUARD_REBOOT.md) | Pi 시퀀스 가드 재부팅 차단 문제 |
| [`docs/LORA_FIXES_2026-08-06.md`](docs/LORA_FIXES_2026-08-06.md) | LoRa 송신 문제 수정 내역 |
| [`docs/STM1_flame_sensor_validation_2026-07-23.md`](docs/STM1_flame_sensor_validation_2026-07-23.md) | 불꽃센서 실물 검증 기록 |

---

## 브랜치

`develop` 이 기본 브랜치입니다. 작업은 `feat/EVDA-000-요약` 으로 따서 PR 로 머지하고,
커밋 메시지에 지라 이슈 키(EVDA-000)를 붙입니다.

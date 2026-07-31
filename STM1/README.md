# STM1 — 홀센서 + 불꽃센서 보드

VEDA 4기 5팀 주차장 감시/제어 시스템의 STM1 보드 펌웨어. NUCLEO-F401RE 기반으로
홀센서 4채널(GPIO 직결)과 불꽃센서(ADC+FFT)를 담당하고, 결과를 UART로
Raspberry Pi에 전달한다.

## 개발 환경

- STM32CubeIDE 2.2.0 (CubeMX 통합), GCC 툴체인
- 처음엔 Keil로 시작했으나 평가판 코드 크기 제한(32KB)에 걸려 CubeIDE로 전환함
  (CMSIS-DSP 라이브러리 링크 시 94KB 필요)

## 클럭 / 핀맵

- 클럭: HSE(bypass, ST-Link MCO 8MHz) 기반 PLL로 84MHz (PLLM=8, PLLN=336, PLLP=÷4)
- 홀센서 D0 4개를 GPIO에 직결(내부 pull-up). D0가 오픈컬렉터라 **LOW = OCCUPIED**:
  - `PA0`(HALL1_D0) / `PA1`(HALL2_D0) / `PA4`(HALL3_D0) / `PB0`(HALL4_D0)
- `PC0`(FLAME_A0): 불꽃센서 ADC1 입력 (`PA3`는 USART2 RX라 사용 불가)
- `PA2`/`PA3`: USART2 (ST-Link VCP, 115200 8N1)

## 신호 처리

- TIM2가 64Hz(PSC=1249, ARR=1049)로 ADC1(PC0, `ADC_SAMPLETIME_480CYCLES`)을 TRGO
  트리거, circular DMA로 `adc_buf[64]`에 저장
- 1초(64샘플) 단위로 `HAL_ADC_ConvCpltCallback`이 `adcBufReadySem` 세마포어를 release
- 같은 64샘플 윈도우에서 두 가지 신호를 뽑아 **OR로 결합**하는 하이브리드 판정
  (실물 DFR0076 + 라이터로 실측 검증, 2026-07-23):
  - **FFT flicker 에너지**(`energy`, 1~20Hz bin 합산): 평상시 주력 신호. baseline이
    1 미만인데 점화 시 40~90대로 뛰고 지속 연소 중에도 대체로 24~93을 유지한다.
    다만 센서가 완전 포화되면 윈도우 내내 값이 평평해져 AC 성분이 수학적으로 전부
    사라지면서 **정확히 `0.0000`**이 되는데, 실측에서 9창(9초) 연속 이어진 구간이 있었다
  - **raw DC 레벨**(`raw_avg`, baseline 대비 delta): 반응은 느리지만 지속적인 IR
    세기를 안정적으로 반영 — 위 포화 구간의 백업
  - `raw_verdict = (energy >= FLAME_ENERGY_THRESHOLD=5.0) || (delta >= FLAME_DELTA_THRESHOLD=300.0)`
  - 임계값 실측 근거(2026-07-30): 평상시 raw 46~69 / delta 0~12 / energy 0.15~0.61,
    불꽃 raw 1480~4095(ADC 풀스케일 포화) / delta 1427~4042 / energy 0~103
  - 짧고 산발적인 dip은 K-of-N 다수결 디바운스로 흡수(최근 5윈도우 중 3개 이상 hit)
    — `TaskHallSensor`의 연속 디바운스와 달리 sliding window 방식. 단 9창 연속 0은
    5창 중 3창 조건으로 못 버티므로 그 구간은 DC delta 항이 커버함
  - baseline은 부팅 직후 첫 3창을 버리고(센서/ADC settle) 다음 3창 평균으로 확정.
    첫 창을 그대로 쓰면 값이 낮게 잡혔을 때 영구 DETECTED로 고착되는 문제가 있었음
  - `energy`는 UART로 내보내지 않는다(Pi 파서에 대응 필드 없음). 임계값 재튜닝이
    필요할 때만 `FLAME_DEBUG_ENERGY`로 확인
  - 손 흔들기(빛 가림)/형광등/모니터 오탐 테스트 통과 확인. 태양광 테스트는 미실시

## FreeRTOS 태스크

| 태스크 | 주기/트리거 | 역할 |
|---|---|---|
| `TaskHallSensor` | 50ms 폴링 | 홀센서 4개 GPIO 직접 읽기, 채널별 250ms 디바운스 후 상태변화 시에만 이벤트 발행 |
| `TaskFlameSensor` | ADC DMA 완료 세마포어 | `adc_buf` 정규화 → FFT → 에너지 계산 → 판정 → 이벤트 발행 |
| `TaskPacketTX` | `sensorEventQueue` 블로킹 대기 | 이벤트를 받아 UART 포맷으로 인코딩 후 송신 |
| `defaultTask` | 1s | 현재는 별다른 역할 없음 |

태스크 간 데이터 공유는 뮤텍스 대신 `osMessageQueue`(`sensorEventQueue`, `SensorEvent_t`)
하나로 처리 — 자연히 직렬화되어 레이스 컨디션이 없다. 정의는
[`Core/Inc/sensor_queue.h`](Core/Inc/sensor_queue.h).

## UART 프로토콜

Raspberry Pi 팀이 이미 구현해둔 파서 형식을 그대로 따른다(당초 계획했던
`[STX][SRC_ID][MSG_TYPE][LEN][PAYLOAD][CRC-8][ETX]` 바이너리 프레임 대신 —
실제로 소비하는 쪽이 이 문자열 포맷이라 채택). 인코딩은
[`Core/Src/sensor_protocol.c`](Core/Src/sensor_protocol.c) 참고.

```
SENSOR:HALL01:OCCUPIED|VACANT:<seq>\r\n     (HALL01~HALL04)
FIRE:FLAME01:DETECTED|CLEARED:<seq>\r\n
```

Pi 파서(`SensorProtocolParser`)가 기대하는 형식은 각각
`SENSOR:<sensor_id>:<OCCUPIED|VACANT>[:seq][:ts]`, `FIRE:<sensor_id>:DETECTED|CLEARED[:seq[:unix_ms]]`.
센서 ID는 Pi의 `config/parking_slots.json`(HALL01~04)과
`.env.fire.local`의 `FIRE_SENSOR_SLOT_MAP=FLAME01=EV01`에 맞춘 값이다.

`<seq>`는 스트림별 독립 카운터(홀 4채널 공용 / 화염 별도)로, Pi가 유실·순서뒤바뀜을
판별하는 데 쓴다. 화염 energy 값은 Pi 파서에 대응 필드가 없어 **전송하지 않는다** —
임계값 재튜닝이 필요하면 `FLAME_DEBUG_ENERGY`를 켜서 별도 주석 줄로 확인할 것.

### PuTTY 디버그 UI 모드

[`Core/Inc/sensor_protocol.h`](Core/Inc/sensor_protocol.h)의 `SENSOR_DEBUG_UI`를 `1`로
바꾸면, 위 원본 프로토콜 라인 대신 ANSI escape 기반으로 화면을 제자리에서 갱신하는
사람이 보기 좋은 상태 테이블(홀센서 4채널 OCCUPIED/VACANT, 화염센서 DETECTED/CLEARED +
energy, 색상 강조)을 PuTTY에 출력한다. **Pi가 파싱할 수 없는 화면이므로 로컬 PuTTY
테스트 전용** — Pi 연동 시에는 반드시 `0`으로 되돌릴 것(기본값은 `0`).

## 알려진 CubeMX 버그

CubeMX 2.2.0에서 FreeRTOS 코드를 재생성할 때마다 `freertos.c`의
`MX_FREERTOS_Init()`에서 `osThreadNew(...)` 호출들이 빠지고 빈 헤더 주석
블록으로 대체되는 버그가 재현된다. **코드 재생성 직후 반드시 이 함수를 확인**하고,
필요하면 4개 태스크의 `osThreadNew` 호출과 큐/세마포어 생성 코드를 복원할 것.

## 현재 상태 (2026-07-23)

실물 DFR0076 불꽃센서로 점화→지속연소→소화 전체 사이클과 오탐(손 흔들기/형광등/
모니터) 테스트를 마쳤고, 하이브리드 판정 알고리즘(위 "신호 처리" 참고)까지 확정했다.
초기 파이프라인 스모크 테스트(센서 미장착 상태, 손가락 접촉으로 ADC 반응만 확인)
기록은 [`../docs/STM1_pipeline_test_2026-07-21.md`](../docs/STM1_pipeline_test_2026-07-21.md)에
남아있으나 이후 실물 센서 테스트로 대체됨.

LoRa(SX1262)는 현재 STM1 통신 경로에 포함되어 있지 않다 — Raspberry Pi가 소비하는
파서가 UART 문자열 포맷이라 UART만으로 확정. 필요해지면 별도로 추가.

남은 것:
- 태양광 환경에서 불꽃센서 오탐 테스트
- 홀센서 4개 실장착 후 채널별 독립 동작 재확인 (CD4051 멀티플렉서는 4채널 브링업에서 OCCUPIED 고착 문제로 제거하고 GPIO 직결로 전환함)
- Raspberry Pi 파서와 실제 연동 테스트

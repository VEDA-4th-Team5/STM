# STM1 — 홀센서 + 불꽃센서 보드

VEDA 4기 5팀 주차장 감시/제어 시스템의 STM1 보드 펌웨어. NUCLEO-F401RE 기반으로
홀센서 4채널(CD4051 멀티플렉서)과 불꽃센서(ADC+FFT)를 담당하고, 결과를 UART로
Raspberry Pi에 전달한다.

## 개발 환경

- STM32CubeIDE 2.2.0 (CubeMX 통합), GCC 툴체인
- 처음엔 Keil로 시작했으나 평가판 코드 크기 제한(32KB)에 걸려 CubeIDE로 전환함
  (CMSIS-DSP 라이브러리 링크 시 94KB 필요)

## 클럭 / 핀맵

- 클럭: HSE(bypass, ST-Link MCO 8MHz) 기반 PLL로 84MHz (PLLM=8, PLLN=336, PLLP=÷4)
- `PA0`(MUX_A) / `PA1`(MUX_B): CD4051 채널 선택 출력
- `PA4`(MUX_COM, pull-down): 멀티플렉서 출력 읽기
- `PC0`(FLAME_A0): 불꽃센서 ADC1 입력 (`PA3`는 USART2 RX라 사용 불가)
- `PA2`/`PA3`: USART2 (ST-Link VCP, 115200 8N1)

## 신호 처리

- TIM2가 64Hz(PSC=1249, ARR=1049)로 ADC1을 TRGO 트리거, circular DMA로 `adc_buf[64]`에 저장
- 1초(64샘플) 단위로 `HAL_ADC_ConvCpltCallback`이 `adcBufReadySem` 세마포어를 release
- 64포인트 FFT(CMSIS-DSP `arm_rfft_fast_f32`), 1~20Hz(bin 1~20) magnitude 합산을
  "에너지값"으로 사용. 화재 판정 임계값은 STM32가 확정하지 않고 Raspberry Pi로
  넘겨서(재플래싱 없이 튜닝 가능하도록) 결정한다 — 코드의 임계값(5.0)은 로컬
  placeholder일 뿐.

## FreeRTOS 태스크

| 태스크 | 주기/트리거 | 역할 |
|---|---|---|
| `TaskHallSensor` | 50ms 폴링 | CD4051 4채널 순회, 채널별 250ms 디바운스 후 상태변화 시에만 이벤트 발행 |
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
SENSOR:sensor_0N:OCCUPIED|VACANT:<seq>\r\n
FLAME:flame_01:ALERT|CLEAR:<seq>:<energy>\r\n
```

## 알려진 CubeMX 버그

CubeMX 2.2.0에서 FreeRTOS 코드를 재생성할 때마다 `freertos.c`의
`MX_FREERTOS_Init()`에서 `osThreadNew(...)` 호출들이 빠지고 빈 헤더 주석
블록으로 대체되는 버그가 재현된다. **코드 재생성 직후 반드시 이 함수를 확인**하고,
필요하면 4개 태스크의 `osThreadNew` 호출과 큐/세마포어 생성 코드를 복원할 것.

## 현재 상태 (2026-07-21)

홀센서/불꽃센서/LoRa 모듈 실물이 배송 지연으로 아직 없어서, STM1 보드만으로
파이프라인 전체(ADC→FFT→에너지계산→판정→UART, 홀센서 디바운스→이벤트→UART)를
검증했다. 상세 내용은 [`../docs/STM1_pipeline_test_2026-07-21.md`](../docs/STM1_pipeline_test_2026-07-21.md).

남은 것:
- 실제 불꽃센서(DFR0076) 도착 후 태양광/형광등/라이터 3종 실측 → Pi 쪽 임계값 초기값 산정
- 실제 CD4051 + 홀센서 4개 장착 후 채널별 독립 동작 재확인
- Raspberry Pi 파서와 실제 연동 테스트

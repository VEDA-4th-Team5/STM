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
| `TaskHallSensor` | 50ms 폴링 | 홀센서 4개 GPIO 직접 읽기, 채널별 250ms 디바운스 후 1초마다 4채널 상태 발행 |
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

전송 주기는 스트림별로 다르다:

- **홀센서**: 상태 변화와 무관하게 **1초마다 4채널 전부** 전송. Pi가 1~2초 폴링을
  전제하고 중복을 멱등 처리하므로 안전하며, 동시에 보드 liveness 신호 역할을 한다
- **화염센서**: **상태가 실제로 바뀔 때만** 전송(엣지 트리거). 부팅 직후 첫 확정값은
  내부 기준값으로만 저장하고 발행하지 않는다(콜드부팅 스팸 방지)

디바운스는 STM32가 책임지고 Pi는 재디바운스하지 않는다는 게 팀 계약(`references/contracts.md`).

## LoRa 무선 전송 (EVDA-46)

같은 센서 문자열을 UART 대신(또는 동시에) LoRa로 보낼 수 있다. 전송 경로는
[`Core/Inc/sensor_protocol.h`](Core/Inc/sensor_protocol.h)의 두 플래그로 고른다.

```c
#define SENSOR_TX_UART 1   /* USART2 -> 콘솔/직결 */
#define SENSOR_TX_LORA 1   /* E22 LoRa 프레임 (Pi 링크) */
```

둘 다 1로 두면 같은 이벤트가 유선·무선 양쪽으로 나가서 A/B 비교가 된다. 문자열
생성은 `sensor_protocol.c`의 한 곳에만 있고 실제 송신은 `emit_line()`이 분기하므로
태스크 로직(`freertos.c`)은 건드릴 필요가 없다.

### 하드웨어

EBYTE **E22-900T22S(1B)** — E220이 **아니다**. M0/M1 인코딩과 레지스터 맵이 다르다
(E220 기준 `M1=1,M0=1`은 E22에서 딥슬립이라 아무 응답이 없다). 드라이버가 USART1과
M0/M1 GPIO를 자체 초기화하므로 `.ioc` 변경이 필요 없다.

| E22 패드 | 방향 | Nucleo 핀 | 실크 | 커넥터 |
|---|---|---|---|---|
| VCC | — | 3V3 | `3V3` | CN6-4 |
| GND | — | GND | `GND` | CN6-6 |
| **TXD** | 모듈 → 보드 | **PA10** (USART1_**RX**) | `D2` | CN9-3 |
| **RXD** | 보드 → 모듈 | **PA9** (USART1_**TX**) | `D8` | CN5-1 |
| M0 | 보드 → 모듈 | PC7 | `D9` | CN5-2 |
| M1 | 보드 → 모듈 | PB6 | `D10` | CN5-3 |
| AUX | 모듈 → 보드 | PA7 | `D11` | CN5-4 |

**TXD/RXD는 교차다.** 모듈의 TXD(출력)가 보드의 RX(PA10)로, 모듈의 RXD(입력)가
보드의 TX(PA9)로 간다. 위 표는 교차가 이미 반영된 상태이므로 **표대로 꽂으면 된다.**
`E22 TXD ↔ PA10`, `E22 RXD ↔ PA9`.

`D8`~`D11`이 CN5에 연속으로 붙어 있어 네 가닥이 한 줄로 정리된다.

M0/M1/AUX는 원래 `PB4`/`PB5`/`PA8`이었으나, 번호판 LED 작업(EVDA-194)이 홀센서를
그 자리로 옮기게 되어 비켜났다. 셋 다 평범한 GPIO라 어디로 가도 되지만 USART1은
`PA9`/`PA10`(대체 `PB6`/`PB7`)뿐이라 옮길 여지가 없다.

AUX는 모듈이 바쁠 때 LOW, idle이면 HIGH인 출력이다. 이걸 봐야 송신이 실제로
끝났는지 알 수 있고, 그래야 연속 송신 때 앞 프레임이 아직 공중에 있는 상태로
다음 프레임을 밀어 넣는 일이 없어진다. 입력에 풀업을 걸어두므로 **선이 안 붙어
있으면 항상 HIGH로 읽혀 예전의 고정 딜레이 동작으로 조용히 되돌아간다** — 배선이
틀려도 퇴행은 없다. 실제 연결 여부는 송신 로그의 `aux=ok` / `aux=미배선?` 로 본다.

### 프레임 포맷

Pi_Server `docs/UART_LORA_PROTOCOL.md` 규격을 그대로 따른다. 새로 설계한 것이 아니라
Pi에 이미 있는 `LoRaDriver` / `parking-link-tool`이 기대하는 형식이다. 구현은
[`Core/Src/lora_frame.c`](Core/Src/lora_frame.c).

```
AA 55 | 01  | 01  | 00 00 00 01 | 00 18 | SENSOR:HALL01:OCCUPIED:1 | D3 F6
SOF   | ver | typ | transport seq| len   | payload                  | CRC16
```

- 모든 다중 byte 정수는 big-endian
- CRC16-CCITT (init `0xFFFF`, poly `0x1021`), 범위는 **version부터 payload 끝까지**
- payload는 기존 센서 문자열 그대로, **CRLF는 붙이지 않는다**(길이 필드가 있으므로 불필요)
- 오버헤드 12 byte. 센서 문자열 기준 총 36~40 byte로 E22 서브패킷 240 byte 한도 내
- transport sequence는 payload 안의 seq와 같은 값을 쓴다(Pi 문서 권장)

### 무선 설정과 전파법

KC RF 시험성적서 원문(`E22-900T22S_1B-KC-RF보고`) 대조로 확정한 값이다.

| 항목 | 값 | 근거 |
|---|---|---|
| 채널 | **30** (922.9 MHz) | `f = 916.9 + CH × 0.2`, 채널 20~32는 200 mW 허용 |
| 출력 | **10 dBm** (10 mW) | 한도의 1/20. E22 최저 단계 |
| LBT | **on** | 송신 전 5 ms 센싱, −65 dBm 임계 |
| 에어레이트 | **62.5k** | 공중 점유가 2.4k의 약 1/15 |
| UART | **115200** | Pi 기본값(`SENSOR_UART_BAUD`)과 일치 |
| 전송 모드 | **고정점** | 투명 모드는 양쪽 주소가 같아야 해서 노드 구분이 불가능 |
| 노드 주소 | STM1 `0x0001` / Pi `0x0010` | STM2 `0x0002` 예약 |
| duty 실측 | **약 1.3%** | 한도 2% |

- **공장 기본값은 위법이다.** 채널 2(917.3MHz)는 RFID 리더 전용인데 기본 출력이
  22 dBm(158 mW)이라 일반 데이터 링크 한도를 크게 넘는다. 반드시 재설정할 것
- **에어레이트는 양쪽 노드가 같아야 한다.** 한쪽만 바꾸면 통신이 끊긴다.
  UART baud는 서로 달라도 무방하다
- **UART baud는 duty와 무관하다.** 공중 점유는 에어레이트로만 정해진다
- Pi 쪽 모듈은 MCU가 없어 스스로 설정하지 못한다. 이 보드에 잠깐 물려
  `persist=true`(C0)로 한 번 구워서 넘겨야 한다

> ⚠️ **안테나**: KC 인증 지정 안테나는 Chengdu Ziisor `TX915-JK-11`(Bendable Rubber,
> 917~923.5MHz **2.5 dBi**)이다. 현재는 재고 문제로 HELTEC 스틱 안테나(**3 dBi**)를
> 쓰고 있어 EIRP가 인증값 대비 0.5 dB 높다. 그 외 기술기준(채널·출력·LBT·점유시간·
> duty)은 모두 충족한다. 실제 배치 시 지정 안테나로 교체할 것.
> KC 등록번호 `R-R-eEt-E22-900T22S1B`, 기기부호 USN1.

### duty cycle 리미터

고정 간격이 아니라 **예산제**다. 60초 창 안에서 누적 공중 점유가 한도를 넘지
않는 한 통과시킨다.

처음에는 "10초에 1프레임" 고정 간격이었는데 두 가지가 틀렸다. 상수가 에어레이트
2.4k 기준이라 62.5k에서는 20배 과보수적이었고, 고정 간격은 버스트를 처리하지
못해 홀 4채널이 동시에 바뀌면 앞의 하나만 통과하고 나머지가 **실제 상태 변화인데도**
버려졌다.

- 한도: 10 dBm은 duty 2% → 60초 중 1200 ms. 그중 80%만 쓴다(추정 오차 여유)
- **화재**는 놓치면 그걸로 끝이라 리미터를 우회한다. 홀은 상태가 다시 보고된다
- 버려진 개수는 `LoRaFrame_GetDroppedCount()`로 확인한다
- 실측 점유는 약 1.3%로 예산 안에 들어온다

### 송신 타이밍

프레임 하나는 UART로 **끊김 없이** 나가야 한다. `HAL_UART_Transmit`이 태스크
전환에 선점당해 1 ms만 멈춰도, E22는 그 공백을 "데이터 끝"으로 보고 그때까지 받은
바이트만 패킷으로 만들어 쏜다. 실측에서 매 패킷이 정확히 13 byte(115200에서 약
1.13 ms = FreeRTOS 틱 하나)에서 잘렸다. 그래서 송신 구간만 `vTaskSuspendAll()`로
감싼다. **근본 해법은 DMA 송신이고 이건 임시방편이다.**

LBT는 채널이 바쁘면 송신을 미루거나 폐기하면서 그 사실을 MCU에 알려주지 않는다.
AUX로 송신 소요 시간을 재서(`LoRa_GetMaxTxBusyMs()`) 그 상황이 드러나게 해뒀다.
공중 점유는 프레임당 약 5 ms이므로 이 값이 크게 길면 채널이 혼잡한 것이다.

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

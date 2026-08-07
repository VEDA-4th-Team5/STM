# STM1 번호판 조명 LED 수신부 설계 (EVDA-194 / EVDA-185)

Pi 쪽 구현(EVDA-194)이 보내는 `ALERT:<id>:LED:ON/OFF:<seq>` 명령을 STM1이 받아 실제
LED를 구동하기 위한 설계 초안. **코드는 아직 없음** — 하드웨어 결정 두 가지가 막혀 있어
설계만 정리하고 팀 확인 후 구현 착수.

관련 문서: `Pi_Server/docs/UART_LORA_PROTOCOL.md`("번호판 조명 LED 명령" 절, 이번에
Pi 쪽에서 확정한 프로토콜의 원본).

## 지금 STM1이 할 수 있는 것 / 못 하는 것

- `USART2`는 이미 `UART_MODE_TX_RX`로 초기화돼 있고 `PA3`가 `USART2_RX`로 배선되어 있다
  (`STM1.ioc:118-125`, `usart.c:46`). **RX 핀 자체는 이미 살아있다.**
- 하지만 이 RX를 실제로 소비하는 코드가 없다. `HAL_UART_Receive_IT`/`_DMA`도,
  `HAL_UART_RxCpltCallback`도, RX용 FreeRTOS 태스크도 존재하지 않는다. 지금까지
  `sensor_protocol.c`는 `HAL_UART_Transmit`으로 **보내기만** 했다
  (`SensorProtocol_SendHallStatus` 등, `sensor_protocol.c:14-33`).
- 즉 필요한 건 "핀을 새로 여는 것"이 아니라 "받는 소프트웨어 파이프라인을 처음부터
  만드는 것"이다.

## 막힌 것: GPIO 핀 배정

`STM1.ioc`(NUCLEO-F401RE)에 실제로 설정된 핀은 19개뿐이고, 이미 다음 용도로 다 찼다:

| 핀 | 용도 |
|---|---|
| PA0, PA1, PA4, PB0 | HALL1~4_D0 (홀센서) |
| PA2, PA3 | USART2 TX/RX |
| PA5 | 보드 온보드 LED(LD2, 상태 표시용 — 번호판 LED 아님) |
| PC0 | FLAME_A0 (불꽃센서 아날로그) |
| PA13, PA14, PB3 | SWD/SWO 디버그 |
| PC13 | 보드 온보드 버튼(B1) |
| PC14/PC15/PH0/PH1 | 오실레이터 |

**번호판 LED용 GPIO는 하나도 배정돼 있지 않다.** 프로토콜은 센서 ID 단위(`HALL01`~`HALL04`)로
독립 ON/OFF를 요구하므로, 슬롯마다 최소 1개씩 출력 핀이 필요하다 — 4채널이면 4핀. 이걸
Nucleo 보드의 남는 핀 중 어디에 배정할지는 하드웨어팀 확인이 필요하다. 홀센서용
CD4051 멀티플렉서를 나중에 다시 붙일지 여부에 따라서도 여유 핀이 달라진다.

추가로 확인 필요: 번호판 근접 LED가 고휘도(수백 mA급)라면 STM32 GPIO가 직접 구동할 수
있는 전류(핀당 통상 20mA대)를 넘을 수 있다. 이 경우 GPIO는 트랜지스터/MOSFET
게이트만 스위칭하고 실제 LED 전류는 별도 전원에서 끌어와야 한다. 이건 순수 회로
설계 영역이라 이 문서에서 결정하지 않는다.

## 소프트웨어 설계안

### 1. UART 수신 파이프라인

Pi의 `SensorLinkManager`가 하는 것과 대칭으로, 개행(`\n`) 단위 라인 수신을 제안한다.

- `HAL_UARTEx_ReceiveToIdle_IT` (IDLE 라인 검출) 또는 단순 1바이트
  `HAL_UART_Receive_IT` + 링 버퍼 중 하나를 선택. STM1은 TX 볼륨이 낮고(초당 4~5줄
  수준) RX도 간헐적(입차 이벤트당 2줄)이라 정교한 DMA보다 단순한 IT 기반으로 충분해
  보인다.
- 콜백에서 바이트를 링 버퍼에 쌓고, `\n`을 만나면 완성된 라인을 큐(기존
  `sensorEventQueueHandle`과 별도 큐 권장 — 그 큐는 STM→Pi 방향 전용이라 방향을 섞지
  않는 게 안전)로 넘긴다.
- 소비 측은 새 태스크(`TaskCommandRX` 등, 기존 `TaskHallSensor`/`TaskFlameSensor`/
  `TaskPacketTX` 네이밍 관례를 따름)로 분리한다.

### 2. 파서

```
ALERT:<sensorId>:LED:ON:<seq>
ALERT:<sensorId>:LED:OFF:<seq>
```

- `sensorId`를 `HALL01`~`HALL04` 중 하나로 매칭해 슬롯 인덱스(0~3)로 변환 —
  `sensor_protocol.c`가 이미 슬롯 인덱스 0~3을 `HALL%02u`로 왕복 변환하는 관례가 있으니
  그대로 재사용.
- `HEARTBEAT`, 알 수 없는 sensorId, 포맷이 어긋난 라인은 조용히 버린다(로그만 남김).
  Pi 쪽과 달리 STM1은 이 프레임을 거부했다고 알려줄 ACK 채널이 없다 — 프로토콜 문서에
  명시된 대로 ACK 없는 설계이므로 자연스럽다.

### 3. LED 상태 + 페일세이프 타이머

- 슬롯별 상태(켜짐/꺼짐)와 FreeRTOS software timer(`osTimerNew`, one-shot)를 각각 하나씩
  둔다.
- `ON` 수신: GPIO를 켜고 타이머를 (재)시작. 이미 켜진 상태에서 또 `ON`이 오면 타이머만
  리셋하고 GPIO는 그대로 둔다 — 프로토콜 문서에 이미 이렇게 명시했다.
- `OFF` 수신: GPIO를 끄고 타이머를 정지. 꺼진 상태에서 `OFF`가 오면 무시.
- 타이머 콜백(페일세이프): GPIO를 강제로 끈다. Pi가 크래시하거나 UART 링크가 끊겨도
  LED가 켜진 채 남지 않는다. 타이머 기본값은 프로토콜 문서에 잠정 5초로 적어뒀는데,
  Pi 쪽 실측(카메라 스냅샷 API가 느리면 노출 호출이 5초를 넘을 수 있음, 자세한 건
  `Pi_Server/docs/UART_LORA_PROTOCOL.md`의 "STM32 페일세이프" 절 참고)에 따라 조정될 수
  있다.

## 다음 단계 (코드 작성 전 확인 목록)

1. **하드웨어팀과 GPIO 핀 배정 확정** — 슬롯 수만큼 출력 핀, 여유 핀 목록에서 선정.
2. **LED 구동 방식 확인** — GPIO 직결 가능한 전류인지, 트랜지스터 스위칭이 필요한지.
3. 위 두 가지가 정해지면 `.ioc`에 출력 핀 추가 → `sensor_protocol.h/.c`에 대칭되는
   수신부 파일(가칭 `alert_command.h/.c`) 추가 → `freertos.c`에 태스크 배선.
4. 실물 없이도 PuTTY로 `ALERT:HALL01:LED:ON:1` 문자열을 STM1에 직접 타이핑해 넣어
   파서/타이머 로직만 먼저 검증 가능 (LED 대신 온보드 LD2로 대체 테스트).

이 문서와 브랜치(`feat/plate-led-EVDA-194`, `origin/develop`에서 분기)는 위 1~2번이
정해질 때까지 설계 상태로 둔다.

# STM1 불꽃센서(DFR0076) 실물 검증 및 알고리즘 재설계 (2026-07-23)

[STM1_pipeline_test_2026-07-21.md](STM1_pipeline_test_2026-07-21.md)에서 확인한 건 "배관"(ADC→FFT→UART 파이프라인 자체가 도는지)이었고, 실제 불꽃센서 하드웨어로 판정 정확도를 검증한 기록은 이 문서.

## 테스트 환경

- 보드: NUCLEO-F401RE (STM1) + DFR0076 아날로그 불꽃센서, PC0(ADC1_IN10)
- 비교용: Arduino Nano + DFR0076 벤더 예제(`analogRead(A0)` 그대로 출력)
- 확인 도구: PuTTY, USART2, 115200 8N1

## 겪은 문제와 원인

1. **ADC 신호가 라이터에 거의 반응 안 함** — `ADC1.SamplingTime`이 `ADC_SAMPLETIME_3CYCLES`(최소값)로 설정돼 있어, 센서 출력 임피던스에서 ADC 내부 sample-hold 커패시터가 충분히 충전되지 못하고 신호가 좁은 범위로 눌림 → `ADC_SAMPLETIME_480CYCLES`로 수정(`adc.c`, `.ioc` 둘 다 반영). 64Hz(15.6ms 간격) 변환이라 여유는 충분함.
2. **그래도 여전히 무반응** — Arduino Nano에 같은 센서를 물려 벤더 예제로 테스트해보니 확실하게 반응(991→6까지 급락). STM32 배선을 재확인한 결과, Nucleo Arduino 헤더의 실크스크린 라벨 **"A0"을 PA0으로 착각하고 꽂았으나 실제 펌웨어는 PC0을 읽고 있었음**. PC0에 제대로 연결 후 정상 반응 확인.

## 알고리즘 재설계

두 문제를 고친 후 재테스트한 결과, FFT flicker 에너지 단독 판정의 한계 발견:

- 라이터 점화 순간(손 떨림 + 불꽃 안정화 전 흔들림)엔 flicker 에너지가 강하게 반응(실측 40~90대, baseline은 1 미만)
- 하지만 불꽃이 안정적으로 지속 연소하면 오히려 flicker가 잦아들어 baseline 수준까지, 심할 땐 (신호 포화로 AC 성분이 완전히 사라져) 정확히 `0.0000`까지 떨어짐 — 실제 불꽃은 완전한 주기 진동이 아니라 카오틱하기 때문

FFT(점화 트리거) + raw ADC 평균의 baseline 대비 delta(지속 확인)를 **OR로 결합**하는 하이브리드 방식으로 재설계(`freertos.c`의 `StartTaskFlameSensor`):

```c
raw_verdict = (energy >= FLAME_ENERGY_THRESHOLD /* 5.0 */)
           || (delta   >= FLAME_DELTA_THRESHOLD  /* 40.0 */);
```

카오틱한 flicker로 인한 순간적 소강에 흔들리지 않도록, 기존의 "N번 연속" 디바운스 대신 K-of-N 다수결(최근 5윈도우 중 3개 이상 hit)로 최종 ALERT/CLEAR를 확정.

## 검증 결과

- **점화 → 지속 연소(FFT 0.0000 구간 포함) → 소화** 전체 사이클에서 ALERT 유지 및 CLEAR 복귀 정상 확인
- **오탐 테스트**: 손 흔들기(빛 가림), 형광등, 모니터 — 전부 CLEAR 유지, 오탐 없음
- **미실시**: 태양광 환경 오탐 테스트

## 다음에 재확인할 것

- 태양광 아래에서 오탐 여부
- `FLAME_ENERGY_THRESHOLD`/`FLAME_DELTA_THRESHOLD`는 현재 실측 기준으로 잘 분리되지만, 여러 다른 개체의 라이터/실제 화재원으로 추가 검증 필요

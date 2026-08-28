# Pi_Server 시퀀스 가드 — 노드 재부팅 시 차단 문제

> 대상: `Pi_Server` (develop) — `include/sensor/SensorSequencePolicy.hpp`
> 관련: EVDA-46 / STM 레포 `STM/Core/Inc/sensor_protocol.h`
> 범위: legacy(v1) 시퀀스 판정 한정. versioned(bootId) 경로는 건드리지 않습니다.

---

## 0. 한 줄 요약

**노드가 켜진 지 약 1시간 이내에 리셋되면, 그 노드의 프레임이 전부 `Stale`로 버려집니다.**
그리고 시퀀스가 리셋 전 값을 다시 넘을 때까지 — 즉 그 노드가 돌아갔던 시간만큼 더 — 계속 막힙니다.

시연 중 노드 하나가 리셋되면 그대로 죽는 시나리오라 우선순위를 높게 봅니다.

---

## 1. 지금 규칙

`include/sensor/SensorSequencePolicy.hpp`

```cpp
inline constexpr std::uint64_t kLegacySequenceRebootDropThreshold = 1000;
```

```cpp
if (*fact.sequence < *state.lastSequence) {
    if (*state.lastSequence - *fact.sequence >
        kLegacySequenceRebootDropThreshold) {
        return {SensorSequenceDecisionCode::Accept, {}};   // 재부팅으로 인정
    }
    return {SensorSequenceDecisionCode::Stale,
            "stale sensor sequence"};                      // 버림
}
```

시퀀스가 역행했을 때 **감소폭이 1000을 넘어야만** 재부팅으로 인정합니다.

STM 은 `bootId` 를 보내지 않으므로(와이어 포맷에 그 필드가 없습니다) 항상 이 legacy 경로를 탑니다.

---

## 2. STM 쪽 사실관계

`STM/Core/Src/sensor_protocol.c`

```c
static uint32_t node_sequence = 0;
```

평범한 RAM 변수입니다. **재부팅하면 0으로 돌아가고 첫 프레임이 `seq=1`** 입니다.
백업 레지스터에 보존하지 않습니다.

이 계약은 `STM/Core/Inc/sensor_protocol.h` 주석에 이미 명시돼 있습니다.

> `!! 재부팅하면 이 값이 1 로 돌아간다. 수신측은 "seq 가 크게 감소 = 노드 재부팅"으로 보고 시퀀스 상태를 초기화해야 한다.`

---

## 3. 그래서 실제로 무슨 일이 생기나

재부팅 후 첫 프레임이 `seq=1` 이므로, 위 규칙이 발동하려면

```
lastSequence - 1 > 1000     →     lastSequence > 1001
```

즉 **리셋 직전 시퀀스가 1001을 넘었을 때만** 복구됩니다.

### 1001까지 얼마나 걸리나

조용할 때(상태 변화 없음) 프레임 발생률 — `SENSOR_HEARTBEAT_MS = 10000`

| 노드 | 매 하트비트 프레임 | 초당 | seq 1001 도달 |
|---|---|---|---|
| STM1 (홀 2 + 화재 1) | 3 | 0.3 | 약 **56분** |
| STM2 (홀 2) | 2 | 0.2 | 약 **83분** |

차량 출입이 잦으면 더 빨라지지만, 조용한 주차장이 기본 조건입니다.

### 실패 시나리오

```
시연 시작
  ↓  20분 경과, seq 가 360 까지 올라감
노드 리셋 (전원 접촉 불량, 리셋 버튼, 워치독 …)
  ↓  seq = 1 부터 다시 시작
Pi:  360 - 1 = 359  ≤ 1000   →   Stale, 버림
  ↓
seq 가 360 을 다시 넘을 때까지 = 약 20분간 그 노드 전체 차단
```

시퀀스는 v1.1부터 **노드 단일 카운터**입니다. 그래서 홀 한 채널이 아니라
**그 노드의 센서 전부**가 같이 막힙니다.

### 왜 지금 규칙으로는 못 잡나

`kLegacySequenceRebootDropThreshold = 1000` 은 "많이 떨어졌으면 재부팅"이라는
감소폭 기준입니다. 그런데 노드가 **오래 돌았을수록 감소폭이 커지므로**,
정작 위험한 경우 — 켜진 지 얼마 안 돼서 리셋되는 경우 — 를 못 잡습니다.
규칙이 실패 케이스와 반대 방향으로 작동합니다.

---

## 4. 요청드리는 수정

역행 판정에 **절대값 조건**을 추가해 주세요.

```cpp
/* STM 은 재부팅 시 시퀀스를 반드시 1 부터 다시 시작한다
 * (STM 레포 Core/Inc/sensor_protocol.h 계약).
 * 따라서 "들어온 seq 가 아주 작다" = 재부팅으로 본다.
 * 기존 감소폭 기준은 노드가 오래 돌았을 때만 발동하므로,
 * 부팅 직후 리셋되는 경우를 못 잡는다. */
inline constexpr std::uint64_t kLegacySequenceRebootRestartCeiling = 16;
```

```cpp
if (*fact.sequence < *state.lastSequence) {
    if (*fact.sequence <= kLegacySequenceRebootRestartCeiling ||
        *state.lastSequence - *fact.sequence >
            kLegacySequenceRebootDropThreshold) {
        return {SensorSequenceDecisionCode::Accept, {}};
    }
    return {SensorSequenceDecisionCode::Stale,
            "stale sensor sequence"};
}
```

### 왜 16인가

- 재부팅 후 첫 프레임은 항상 1입니다. 여러 센서가 초기 몇 프레임을 나눠 가지므로
  각 센서가 처음 보는 값은 대략 1~6 범위입니다. 여유를 둬 16으로 잡았습니다.
- 정상 운용 중에는 시퀀스가 계속 증가하므로 16 이하가 다시 나올 일이 없습니다.
- 재정렬·중복 같은 노이즈는 몇 단위 수준이라 여전히 `Stale` 로 걸립니다.
  중복은 그 위의 `duplicate sensor sequence` 가 먼저 잡습니다.

### 건드리지 않는 것

- `kLegacySequenceRebootDropThreshold` 는 그대로 둡니다. 두 조건은 OR 입니다.
- versioned(bootId) 경로는 손대지 않습니다. STM 이 나중에 bootId 를 실어 보내게
  되면 그쪽이 더 정확한 해법이지만, 와이어 포맷 변경이라 이번 범위가 아닙니다.

---

## 5. 확인 부탁드릴 것

**화재 경로도 같은 가드를 타는지** 봐주세요.

`ParkingSensorSequenceGuard` 는 `parking` 네임스페이스에서 `ParkingSensorEvent` 를
받으므로 홀 경로로 보입니다. `FireSensorMessage` 쪽이 별도 경로라면 화재는 영향이
없고, 같은 가드를 탄다면 재부팅 후 화재 감지도 같이 막히므로 심각도가 달라집니다.

---

## 6. 이 수정 전까지의 임시 대응

가드 상태(`ParkingSensorSequenceGuard::stateBySensor_`)는 인메모리입니다.
**Pi 서버를 재시작하면 초기화됩니다.** `reset(sensorId)` / `clear()` 도 이미 있습니다.

그래서 시연 체크리스트에 이 한 줄을 넣어두려고 합니다.

> STM 노드를 리셋했으면 Pi 서버도 같이 재시작한다.

운영 API 로 `reset()` 을 노출할 수 있으면 그게 더 낫습니다.

---

## 7. 재현 방법

1. 노드를 켜고 5~10분 둔다 (seq 가 100~200 정도까지 올라감)
2. 노드 리셋 버튼을 누른다
3. 서버 로그에서 `stale sensor sequence` 가 반복되는지 확인
4. 대시보드에서 해당 노드의 주차면 상태가 갱신되지 않는지 확인
5. Pi 서버를 재시작하면 즉시 복구되는지 확인 (가드가 원인이라는 확증)

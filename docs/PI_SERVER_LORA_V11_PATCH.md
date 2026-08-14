# Pi_Server LoRa v1.1 수신 대응 — 수정 가이드

> 대상: `Pi_Server` (develop)
> 관련: EVDA-46, EVDA-201 / STM 레포 `docs/LORA_PAYLOAD_V1_1_FOR_PI.md`
> 범위: LoRa 수신 경로 한정. 다른 링크 모드(uart-line)에는 영향 없음.

---

## 0. 지금 무슨 일이 일어나고 있나

**프레임은 정상적으로 도착하는데 서버가 파싱 전에 버리고 있습니다.**

실측 (서버 Pi, `/dev/serial0` 원시 바이트 20초 캡처):

```
aa 55 02 01 00 00 00 3d 00 1c 53 45 4e 53 4f 52 3a 53 54 4d 32 3a 48 41
4c 4c 30 33 3a 56 41 43 41 4e 54 3a 36 31 c2 da

 AA 55 | 02  | 01  | 0000003D | 001C | "SENSOR:STM2:HALL03:VACANT:61" | C2DA
 SOF   | ver | typ | seq=61   | len  | payload                        | CRC
        ^^^^ 여기서 걸린다
```

전파·CRC·프레이밍 전부 정상입니다. 막히는 곳은 두 군데입니다.

| # | 위치 | 증상 |
|---|---|---|
| 1 | `LoRaDriver::consume()` | `kVersion`(0x01) 하드 체크 → **모든 v1.1 프레임이 `rejected_frames_`로** |
| 2 | `SensorProtocolParser` | 노드 ID 필드가 늘어 상태 필드가 한 칸 밀림 |

**1번만 고치면 프레임은 통과하지만 2번에서 다시 버려집니다. 둘 다 필요합니다.**

---

## 1. 페이로드가 어떻게 바뀌었나

노드가 둘(STM1, STM2)이 되면서 **노드 ID 필드**가 추가됐고, 화재에는 **energy**가 붙었습니다.

```
v1.0   SENSOR:HALL01:OCCUPIED:142
v1.1   SENSOR:STM1:HALL01:OCCUPIED:142
              ^^^^^ 노드 ID

v1.0   FIRE:FLAME01:DETECTED:12
v1.1   FIRE:STM1:FLAME01:DETECTED:144:43.70
            ^^^^^                     ^^^^^ energy
```

### 노드와 센서 배치

| 노드 | LoRa 주소 | 홀센서 | 화재 |
|---|---|---|---|
| STM1 | `0x0001` | `HALL01`, `HALL02` | `FLAME01` |
| STM2 | `0x0002` | `HALL03`, `HALL04` | 없음 |

> **센서 ID는 노드를 가로질러 전역 고유합니다.** `HALL03`은 언제나 STM2의 것입니다.
> 그래서 **파싱 단계에서는 노드 ID를 버려도 됩니다** — `parking_slots.json` 매핑을
> 손댈 필요가 없습니다. 노드 ID는 하행 명령 라우팅과 재부팅 감지에 쓰는 값입니다.

`energy`는 불꽃센서 FFT의 1~20Hz 대역 에너지입니다. **판정은 STM32가 하므로
이 값으로 재판정하지 마십시오.** 화재 여부는 `state`만 봅니다. 지금은 버려도
되고, Qt 시각화가 필요해지면 그때 전달 경로를 만들면 됩니다.

---

## 2. `include/device/LoRaDriver.hpp`

### 2-1. 버전 상수

```cpp
class LoRaDriver {
public:
    static constexpr std::uint8_t kSof0 = 0xAA;
    static constexpr std::uint8_t kSof1 = 0x55;
    static constexpr std::uint8_t kVersion = 0x01;
```

아래처럼 수신 허용 범위를 따로 둡니다.

```cpp
class LoRaDriver {
public:
    static constexpr std::uint8_t kSof0 = 0xAA;
    static constexpr std::uint8_t kSof1 = 0x55;

    /** 우리가 프레임을 만들 때 찍는 버전. */
    static constexpr std::uint8_t kVersion = 0x01;

    /*
     * 수신 시 허용하는 버전 범위.
     *
     * 0x01 = v1.0 페이로드 (노드 ID 없음, 스트림별 seq, energy 없음)
     * 0x02 = v1.1 페이로드 (노드 ID 추가, 노드 단일 seq, 화재 energy 동봉)
     *
     * 보낼 때는 좁게, 받을 때는 넓게. 그래야 STM 과 Pi 를 같은 순간에
     * 배포하지 않아도 되고, 한쪽만 롤백해도 링크가 안 깨진다.
     */
    static constexpr std::uint8_t kVersionMin = 0x01;
    static constexpr std::uint8_t kVersionMax = 0x02;
```

### 2-2. `LoRaFrame`에 버전 노출

```cpp
struct LoRaFrame {
    LoRaMessageType type{LoRaMessageType::SensorEvent};
    std::uint8_t version{0x01};      // ← 추가
    std::uint32_t sequence{};
    std::vector<std::uint8_t> payload;
};
```

> `LoRaFrame`이 `LoRaDriver`보다 위에 선언돼 있어 `kVersion`을 쓸 수 없습니다.
> 리터럴 `0x01`로 두거나, 필요하면 파일 스코프 상수로 빼십시오.

버전을 굳이 노출하는 이유: 지금 패치는 필드 위치로 형식을 판별하지만(§4),
나중에 페이로드가 더 갈라지면 **버전으로 분기하는 편이 안전**합니다. 그때를
위해 정보를 버리지 않고 올려둡니다.

---

## 3. `src/device/LoRaDriver.cpp` — `consume()`

### 3-1. 버전 검사

```cpp
        if (buffer_[2] != kVersion) {
            ++rejected_frames_;
            buffer_.erase(buffer_.begin());
            continue;
        }
```

를 아래로 바꿉니다.

```cpp
        if (buffer_[2] < kVersionMin || buffer_[2] > kVersionMax) {
            ++rejected_frames_;
            buffer_.erase(buffer_.begin());
            continue;
        }
```

### 3-2. 버전 전달

프레임을 조립하는 곳에 한 줄 추가합니다.

```cpp
        LoRaFrame frame;
        frame.type = static_cast<LoRaMessageType>(buffer_[3]);
        frame.version = buffer_[2];                    // ← 추가
        frame.sequence = readU32(buffer_.data() + 4);
        frame.payload.assign(buffer_.begin() + 10,
                             buffer_.begin() + 10 + payload_size);
```

### 3-3. `encode()` 는 그대로

`encode()`가 `kVersion`(0x01)을 찍는 것은 **바꾸지 않아도 됩니다.**
STM 쪽 디코더가 이미 0x01/0x02를 모두 받도록 되어 있습니다.

---

## 4. `src/sensor/SensorProtocolParser.cpp`

### 4-1. 왜 지금 깨지나

`split()`이 콜론으로 자른 뒤 위치로 필드를 읽습니다.

```
v1.0  SENSOR : HALL01 : OCCUPIED : 142
       [0]     [1]      [2]        [3]
                        ^^^^ 상태

v1.1  SENSOR : STM2 : HALL03 : VACANT : 61
       [0]     [1]    [2]      [3]      [4]
                      ^^^^^^ 여기가 상태 자리인데 센서 ID 가 들어옴
```

그래서 `sensor state must be OCCUPIED or VACANT`로 거부됩니다.
필드 개수(5개)는 허용 범위 안이라 개수 검사로는 안 걸립니다.

### 4-2. 헬퍼 추가

익명 네임스페이스 안, `applyOptionalTail()` 위쯤에 넣습니다.

```cpp
// v1.1 페이로드는 타입 뒤에 노드 ID 가 하나 더 붙는다.
//   v1.0  SENSOR:HALL01:OCCUPIED:142
//   v1.1  SENSOR:STM1:HALL01:OCCUPIED:142
//
// 버전을 인자로 받지 않고 상태 필드의 위치로 판별한다. fields[2] 가 상태가
// 아닌데 fields[3] 이 상태이면 fields[1] 은 노드 ID 다. 이렇게 하면 v1.0 과
// v1.1 을 같은 코드로 받을 수 있고, 링크 계층 변경 없이 적용된다.
//
// 센서 ID 는 노드를 가로질러 전역 고유하므로(STM1=HALL01/02, STM2=HALL03/04)
// 노드 ID 는 버려도 슬롯 매핑에 지장이 없다.
void stripNodeField(
    std::vector<std::string>* fields,
    const std::string& stateA,
    const std::string& stateB) {
    if (fields == nullptr || fields->size() < 4) {
        return;
    }
    const auto second = upper((*fields)[2]);
    const auto third = upper((*fields)[3]);
    const bool secondIsState = (second == stateA || second == stateB);
    const bool thirdIsState = (third == stateA || third == stateB);
    if (!secondIsState && thirdIsState) {
        fields->erase(fields->begin() + 1);
    }
}
```

### 4-3. `parse()` — 홀센서

```cpp
    const auto fields = split(normalizedLine);
    if (fields.size() < 3 || fields.size() > 5) {
```

를

```cpp
    auto fields = split(normalizedLine);          // const 제거
    stripNodeField(&fields, "OCCUPIED", "VACANT");
    if (fields.size() < 3 || fields.size() > 5) {
```

로 바꿉니다. **아래 로직은 손대지 않습니다** — 노드 필드를 떼어내면 v1.0과
같은 배치가 되기 때문입니다.

### 4-4. `parseFire()` — 화재

```cpp
    const auto fields = split(normalizedLine);
    if (fields.size() < 3 || fields.size() > 5) {
```

를

```cpp
    auto fields = split(normalizedLine);
    stripNodeField(&fields, "DETECTED", "CLEARED");

    // v1.1 화재 프레임은 끝에 energy(1~20Hz 대역 에너지)가 붙는다.
    // 판정은 STM32 가 하고 이 값은 표시용이라 여기서는 버린다.
    // 필요해지면 FireSensorMessage 에 필드를 추가할 것.
    if (fields.size() > 5) {
        fields.resize(5);
    }

    if (fields.size() < 3 || fields.size() > 5) {
```

로 바꿉니다.

---

## 5. 검증

### 5-1. 단위 테스트

`tests/unit/UartLoRaDriverTest.cpp`에 v1.1 프레임을 추가하시면 좋습니다.
아래는 CRC까지 계산된 실제 바이트라 그대로 쓰실 수 있습니다.

```
SENSOR:STM1:HALL01:OCCUPIED:142      (payload 31 B / frame 43 B)
AA 55 02 01 00 00 00 8E 00 1F 53 45 4E 53 4F 52 3A 53 54 4D 31 3A 48 41
4C 4C 30 31 3A 4F 43 43 55 50 49 45 44 3A 31 34 32 EF AC

SENSOR:STM1:HALL03:VACANT:143        (payload 29 B / frame 41 B)
AA 55 02 01 00 00 00 8F 00 1D 53 45 4E 53 4F 52 3A 53 54 4D 31 3A 48 41
4C 4C 30 33 3A 56 41 43 41 4E 54 3A 31 34 33 8D 48

FIRE:STM1:FLAME01:DETECTED:144:43.70 (payload 36 B / frame 48 B)
AA 55 02 01 00 00 00 90 00 24 46 49 52 45 3A 53 54 4D 31 3A 46 4C 41 4D
45 30 31 3A 44 45 54 45 43 54 45 44 3A 31 34 34 3A 34 33 2E 37 30 52 AF
```

CRC 구현 확인용 표준 벡터: `CRC("123456789") = 0x29B1`

파서 단위 테스트에는 문자열만 넣어도 됩니다.

```
SENSOR:STM2:HALL03:VACANT:61     -> sensorId=HALL03, state=Vacant, seq=61
SENSOR:HALL01:OCCUPIED:142       -> v1.0 도 그대로 통과해야 함
FIRE:STM1:FLAME01:DETECTED:144:43.70 -> sensorId=FLAME01, state=Detected, seq=144
FIRE:FLAME01:CLEARED:12          -> v1.0 도 그대로 통과해야 함
```

### 5-2. 실기기

```bash
sudo systemctl restart pi-server
journalctl -u pi-server -f | grep -iE "SENSOR|FIRE|reject|lora"
```

STM 자석을 붙였다 떼면 아래가 떠야 합니다.

```
SENSOR:STM2:HALL03:OCCUPIED:...
```

안 뜨면 `rejectedFrames()` 카운터를 확인하십시오. 올라가면 아직 링크 계층에서
막히는 것이고, 안 올라가는데 상태가 안 바뀌면 파서 쪽입니다.

---

## 6. 이 패치에 **포함되지 않은** 것

### 6-1. 시퀀스 재부팅 처리 ★

v1.1부터 `seq`는 **노드당 단일 카운터**입니다. 홀과 화재가 같은 번호를 나눠 씁니다.

```
SENSOR:STM1:HALL01:VACANT:23
SENSOR:STM1:HALL02:VACANT:24
FIRE:STM1:FLAME01:CLEARED:25:0.52    ← 화재도 같은 줄기
```

**STM32가 재부팅하면 이 값이 1부터 다시 시작합니다.**

현재 `ParkingSensorSequenceGuard`는 `seq <= last`인 프레임을 거부하고
`reset()` / `clear()` 호출 경로가 없습니다. 그대로 두면

> STM32 재부팅 → 새 seq(1, 2, 3…)가 전부 "과거 값"으로 거부
> → **해당 노드가 영구히 막힘.** 장애 지속 시간 ≈ 재부팅 전 가동 시간

필요한 규약:

```
수신한 seq 가 직전 값보다 "크게" 감소하면 노드 재부팅으로 간주하고
해당 노드의 시퀀스 상태를 초기화한 뒤 새 값을 받아들인다.
```

기준은 예컨대 `last - seq > 1000`이면 충분합니다. 무선 구간에서 1000이나
되돌아가는 재정렬은 일어나지 않습니다.

> 이건 v1.1 때문에 생긴 문제가 아니라 **v1.0에도 있는 결함**이며, v1.1로
> 넘어오면 반드시 드러납니다. **시연 중 STM을 리셋하면 바로 겪습니다.**
> 가드 로직이라 LoRa 수신 경로와 성격이 달라 이 패치에서 분리했습니다.

### 6-2. 하행 명령의 목적지 주소

Pi가 STM으로 보낼 때는 프레임 앞에 **목적지 3바이트**가 필요합니다.

```
<ADDH> <ADDL> <CH>  +  프레임

HALL01/02 -> STM1 : 00 01 1E
HALL03/04 -> STM2 : 00 02 1E        (0x1E = 채널 30)
```

`LoRaDriver::send()`가 `encode()` 결과를 그대로 `uart_.writeAll()` 하고 있어
이 3바이트가 없습니다. 없으면 **하행이 통째로 나가지 않습니다** — 모듈이 앞
3바이트를 주소로 해석하므로, 안 붙이면 프레임의 `AA 55 02`가 주소 `0xAA55` /
채널 2로 읽힙니다.

번호판 LED 제어를 서버에서 하실 때 필요합니다.

### 6-3. 미니 UART

```
/dev/serial0 -> ttyS0
```

현재 서버 Pi는 **미니 UART**를 쓰고 있습니다. 보드레이트가 VPU 코어 클럭에
묶여 있어, CPU 부하가 변하면 실제 보드레이트가 흔들려 **115200에서 간헐적으로
바이트가 깨집니다.** 평소엔 멀쩡하다가 서버가 바쁠 때만 CRC 실패가 생기는
형태라 원인을 찾기 어렵습니다.

`config.txt`에 아래를 넣고 재부팅하면 `ttyAMA0`(PL011)로 바뀝니다.

```
dtoverlay=disable-bt
```

블루투스를 써야 한다면 `core_freq_min=500`으로 클럭을 고정하는 우회책도
있지만, `disable-bt`가 정석입니다.

---

## 문의

STM 쪽 규격/구현은 안민철에게 문의 주십시오.
§6-1(시퀀스 재부팅)은 처리 방향을 알려주시면 규격서에 반영하겠습니다.

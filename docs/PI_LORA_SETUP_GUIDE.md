# Raspberry Pi LoRa 수신 환경 구축 가이드

> 다른 Pi에서 STM1의 LoRa 프레임을 받아보려는 팀원용.
> 기준 기기: 안민철 Pi (`amc`, Debian 11 Bullseye, 커널 6.1.21-v8+, aarch64)
> 규격은 `docs/LORA_PAYLOAD_V1_1_FOR_PI.md` 참고.

---

## 0. 먼저 알아둘 것

**LoRa 모듈이 하나 더 필요합니다.** 현재 모듈 3개가 각각 다른 주소로 구워져 있습니다.

| 모듈 | 주소 | 용도 |
|---|---|---|
| STM1 보드에 장착 | `0x0001` | 센서 노드 (HALL01/02 + 화재) |
| STM2 보드에 장착 | `0x0002` | 센서 노드 (HALL03/04) |
| Pi 연결 | `0x0010` | 수신 |

> **STM 쪽 모듈은 어떤 것을 꽂아도 됩니다.** 부팅할 때마다 펌웨어가 자기
> 주소를 다시 쓰기 때문입니다. Pi 모듈만 영구 기록이라 예외입니다.

**같은 주소를 두 모듈이 쓰면 안 됩니다.** 새 Pi로 테스트하시려면 둘 중 하나입니다.

- **A. 기존 Pi 모듈을 빌려간다** — 설정 그대로라 바로 됩니다. 대신 그동안 제 Pi는 못 씁니다
- **B. 새 모듈을 굽는다** — 주소를 새로 정해야 하고, STM 펌웨어의 `LORA_PEER_ADDRESS`도 바꿔야 합니다 (§6)

**A를 권합니다.** 테스트 목적이면 그게 제일 빠릅니다.

---

## 1. 배선

| E22 패드 | Pi 핀 | BCM | 물리핀 |
|---|---|---|---|
| VCC | 3V3 | — | 1 또는 17 |
| GND | GND | — | 6, 9, 14 … |
| TXD | UART RX | GPIO15 | 10 |
| RXD | UART TX | GPIO14 | 8 |
| M0 | GPIO23 | 23 | 16 |
| M1 | GPIO24 | 24 | 18 |
| AUX | (미연결) | — | — |

> **TXD/RXD는 교차입니다.** 모듈 TXD → Pi RX(GPIO15), 모듈 RXD → Pi TX(GPIO14).
> 위 표는 교차가 반영돼 있으니 **표대로 꽂으면 됩니다.**

> **VCC는 반드시 3.3V.** 5V에 물리면 모듈이 상합니다.

AUX는 Pi가 수신 위주라 지금은 필요 없습니다.

---

## 2. OS 설정

### `/boot/config.txt`

> Bullseye 기준입니다. **Bookworm이면 경로가 `/boot/firmware/config.txt`** 입니다.

```bash
sudo cp /boot/config.txt /boot/config.txt.bak
sudo nano /boot/config.txt
```

아래 3줄이 있어야 합니다.

```
enable_uart=1
dtoverlay=disable-bt
gpio=23,24=op,dl
```

| 항목 | 이유 |
|---|---|
| `enable_uart=1` | 직렬 포트 활성화 |
| `dtoverlay=disable-bt` | 블루투스가 PL011을 가져가는 것을 막아 `serial0`을 `ttyAMA0`에 고정. 없으면 성능이 떨어지는 미니 UART로 붙어 115200에서 불안정 |
| `gpio=23,24=op,dl` | 부팅 시 M0/M1을 출력·LOW로 고정. 없으면 입력으로 풀려 모듈이 엉뚱한 모드로 들어갈 수 있음 |

**재부팅해야 적용됩니다.**

### 콘솔 getty 끄기

`serial0`을 로그인 콘솔이 잡고 있으면 데이터가 섞입니다.

```bash
sudo systemctl disable --now serial-getty@ttyAMA0
```

확인:

```bash
systemctl is-enabled serial-getty@ttyAMA0   # disabled
systemctl is-active  serial-getty@ttyAMA0   # inactive
```

`/boot/cmdline.txt`에 `console=serial0,115200`이 있으면 그것도 지워야 합니다.

### 권한

`sudo` 없이 쓰려면 두 그룹이 필요합니다.

```bash
sudo usermod -aG dialout,gpio $USER
```

- `dialout` → `/dev/ttyAMA0` (root:dialout 660)
- `gpio` → `/dev/gpiomem` (root:gpio 660). 이게 있으면 `raspi-gpio`에 sudo가 필요 없습니다

**로그아웃했다 다시 로그인해야 적용됩니다.**

---

## 3. 확인

```bash
ls -l /dev/serial0                 # -> ttyAMA0 심볼릭 링크
stty -F /dev/serial0 115200 raw -echo
raspi-gpio get 23,24               # 둘 다 func=OUTPUT level=0
```

`level=0`이 곧 `M0=0, M1=0` = **일반(운용) 모드**입니다.

---

## 4. LoRa 모듈 설정 확인

모듈이 올바르게 구워져 있는지 봅니다. 설정 모드로 들어갔다 나오는 절차입니다.

```bash
# 설정모드 진입 (M1=1, M0=0)
raspi-gpio set 23 op dl; raspi-gpio set 24 op dh; sleep 2
stty -F /dev/serial0 9600 raw -echo          # 설정모드는 9600 고정
exec 3<>/dev/serial0; printf "\xC1\x00\x09" >&3
timeout 2 dd bs=1 count=12 <&3 2>/dev/null | hexdump -C
exec 3>&-

# 일반모드 복귀
raspi-gpio set 24 op dl; sleep 2
stty -F /dev/serial0 115200 raw -echo
```

**기대 응답:**

```
c1 00 09 | 00 10 | 00 | e7 | 03 | 1e | 53 | 00 00
           ADDH/L  NETID REG0 REG1 REG2 REG3  CRYPT
```

| 바이트 | 값 | 의미 |
|---|---|---|
| ADDH/ADDL | `00 10` | 노드 주소 0x0010 |
| NETID | `00` | 네트워크 ID |
| REG0 | `E7` | UART 115200 · 8N1 · 에어레이트 62.5k |
| REG1 | `03` | 송신출력 10 dBm |
| REG2 | `1E` | 채널 30 (922.9 MHz) |
| REG3 | `53` | bit6 고정점 · bit4 LBT on |
| CRYPT | `00 00` | 암호키 (write-only, 항상 0으로 읽힘) |

> **설정 모드 UART는 저장된 baud와 무관하게 항상 9600입니다.** 115200으로 물어보면 영영 무응답입니다. 브링업 때 이걸로 반나절 날린 적 있습니다.

> **E22는 E220이 아닙니다.** M0/M1 인코딩이 반대라 E220 기준 코드는 아예 동작하지 않습니다. E22에서 `M1=1, M0=1`은 딥슬립입니다.

---

## 5. 점검 도구

제 Pi의 `~/lora_console.py`를 복사해 쓰시면 됩니다.

```bash
scp amc71@<제Pi주소>:~/lora_console.py ~/
python3 ~/lora_console.py
```

의존성 없습니다. 표준 라이브러리만 씁니다 (`pyserial` 불필요).

**기능:**
- 수신 프레임 디코딩 + CRC 검증 + 색 구분 표시 (v1.0/v1.1 동시 지원)
- `1`~`4` 로 해당 슬롯 LED 토글 (고정점 목적지 헤더 자동 부착)
- `s` 중간 통계 / `q` 또는 Ctrl-C 종료 시 리포트
- 노드별 유실률·CRC 실패·**재부팅 감지**

**정상 출력 예:**

```
◀ STM1 v2  HALL01 VACANT    #23
◀ STM1 v2  FLAME01 DETECTED #26  energy=42.09
▶ ALERT:HALL01:LED:ON:13  (frame 34B, +3B 목적지)
```

`~/lora_monitor.py`도 있는데 그건 5초마다 자동으로 PING을 보내는 구버전입니다. 대화형이 필요 없을 때만 쓰세요.

---

## 6. 다른 주소로 새 모듈을 굽는 경우

§0의 B안을 택하셨다면 추가 작업이 있습니다.

1. **모듈 굽기** — MCU가 없으면 스스로 설정 못 합니다. STM 보드에 잠깐 물려 `LORA_BURN_MODULE 1` 로 빌드해 구워야 합니다 (`STM/Core/Inc/lora_e22.h`)
2. **STM 펌웨어의 목적지 변경** — `LORA_PEER_ADDRESS`가 `LORA_ADDR_RPI(0x0010)`로 박혀 있습니다. 새 주소로 바꿔야 STM이 그쪽으로 보냅니다
3. **`lora_console.py`의 `STM1_ADDR`** — 하행 목적지. STM 주소라 그대로 두면 됩니다

**두 Pi가 같은 주소를 쓰면 둘 다 같은 프레임을 받습니다.** 고정점 모드라 주소가 일치하는 모든 모듈이 수신합니다. 테스트 중 혼선이 생기니 주소를 겹치지 마세요.

---

## 7. 전파법 — 설정을 바꾸기 전에

**채널·출력·에어레이트·LBT를 임의로 바꾸면 전파법 위반이 될 수 있습니다.**

```
채널 30 / 10 dBm / 62.5k / LBT on
```

- **공장 기본값(채널 2, 22 dBm)은 위법입니다.** 채널 2는 RFID 리더 전용인데 출력이 일반 데이터 한도의 16배입니다
- **에어레이트를 낮추면 duty가 비례해서 늘어납니다.** 2.4k로 내리면 같은 프레임의 공중 점유가 25배가 됩니다
- **UART baud는 duty와 무관합니다.** 공중 점유는 에어레이트로만 정해집니다

상세는 **`docs/LORA_RF_COMPLIANCE_KR.md`** 를 읽어주세요. AI에게 작업을 시키실 거면 그 문서를 먼저 먹이시는 것을 권합니다 — 설정 변경 요청 시 경고하도록 지침이 들어 있습니다.

---

## 8. 안 될 때

| 증상 | 확인할 것 |
|---|---|
| 설정모드 응답 없음 | baud가 9600인지 / M1=1,M0=0 인지 / TXD·RXD 교차 / VCC 3.3V |
| 일반모드에서 아무것도 안 옴 | STM1이 켜져 있는지 / 채널·에어레이트·NETID가 같은지 / getty가 포트를 잡고 있는지 |
| 프레임이 중간에 잘림 | `hexdump ... \| head` 로 보면 버퍼링 때문에 출력이 날아갑니다. `lora_console.py`를 쓰세요 |
| CRC 실패가 많음 | 안테나 간 거리가 너무 가까우면(10cm 이하) 수신단이 포화됩니다. 2~3 m 떼어보세요 |
| 재부팅 후 안 됨 | `/boot/config.txt`의 `gpio=23,24=op,dl` 확인 |

---

## 부록: `~/n_lora/` 는 무엇인가

제 Pi에 커널 line discipline 모듈 실험이 남아 있습니다.

**아직 실험 단계고 한 번도 로드된 적 없습니다.** 프레임 조립을 커널에서 하는 것으로, `read()` 한 번에 프레임 하나가 나오게 하는 물건입니다. 요건상 "드라이버" 항목에 쓸 수 있을지 보려고 만들었습니다.

지금 단계에서는 **쓰지 마세요.** 유저스페이스(`lora_console.py`)로 유실 0이 나오고 있어 실익이 없고, 커널 모듈은 버그 하나에 시스템이 멈춰 원격 복구가 안 됩니다.

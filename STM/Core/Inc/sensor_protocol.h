#ifndef __SENSOR_PROTOCOL_H
#define __SENSOR_PROTOCOL_H

#include <stdint.h>
#include "board_config.h"

/* 센서 페이로드 규격 v1.1 (프레임 version = 0x02)
 *
 *   SENSOR:<node>:<sensor>:<state>:<seq>\r\n
 *   FIRE:<node>:<sensor>:<state>:<seq>:<energy>\r\n
 *
 *   SENSOR:STM1:HALL01:OCCUPIED:142
 *   FIRE:STM1:FLAME01:DETECTED:144:43.70
 *
 * v1.0 대비 바뀐 것:
 *
 *  1) <node> 추가 -- 어느 보드에서 온 이벤트인지 링크 계층에서 알 수 없었다.
 *     LoRa 고정점 헤더에 주소가 있지만 모듈이 떼어내므로 수신측에는 안 온다.
 *     STM2 가 붙으면 센서 이름이 겹치지 않는다는 약속에만 기대게 되는데,
 *     그건 규약이 아니다.
 *
 *  2) <seq> 를 노드 단일 카운터로 통일 -- 예전엔 홀/화재가 각각 셌다.
 *     Pi 의 시퀀스 가드는 단일 스트림으로 보고 있어서, 홀 141 다음 화재 1 이
 *     오면 역행으로 판정했다. 이제 프레임 transport seq 와 항상 같은 값이다.
 *
 *     !! 재부팅하면 이 값이 1 로 돌아간다. 수신측은 "seq 가 크게 감소 =
 *        노드 재부팅"으로 보고 시퀀스 상태를 초기화해야 한다. 그렇게 하지
 *        않으면 재부팅 후 그 노드가 영구히 막힌다(Pi 쪽 기존 결함).
 *
 *  3) <energy> 추가 -- 1~20Hz 대역 에너지. 화재 프레임에만 붙는다.
 *     Qt 대시보드 시각화용. 판정 자체는 STM32 가 계속 책임진다.
 *
 * 센서 ID 는 Pi 의 config/parking_slots.json(HALL01~04) 과
 * .env.fire.local 의 FIRE_SENSOR_SLOT_MAP(FLAME01) 에 맞춘 값이다.
 *
 * 전송 주기 (전파법 duty 예산 안에서 설계됨. 최악 1.36%, 한도 2%):
 *   - HALL  : 상태 변화 즉시 + 30초 하트비트.
 *             단 채널별 SENSOR_HALL_COOLDOWN_MS 쿨다운이 걸린다(아래 참고).
 *   - FIRE  : 상태 변화 즉시 + 30초 하트비트.
 *             DETECTED 인 동안에는 SENSOR_FLAME_TELEMETRY_MS 마다 energy 갱신.
 *
 * 어느 쪽이든 전선에 나가는 값은 *디바운스가 끝난* 상태다. Pi 는 재디바운스
 * 하지 않는다(팀 계약: 디바운스는 STM32 책임).
 */

/* 전송 경로 선택 (EVDA-46).
 *   SENSOR_TX_UART : USART2 로 문자열을 그대로 보낸다. 지금까지 검증된 경로.
 *   SENSOR_TX_LORA : 같은 문자열을 Pi 규격 LoRa binary frame 에 실어 보낸다.
 *                    규격은 Pi_Server docs/UART_LORA_PROTOCOL.md, 구현은
 *                    lora_frame.c 참고.
 * 둘 다 1로 두면 같은 이벤트가 유선·무선 양쪽으로 나가므로 A/B 비교가 된다.
 * LoRa 를 켜려면 main.c 에서 LoRa_Init() 이 먼저 호출되어야 한다.
 *
 * !! LoRa 를 켜기 전에 읽을 것: 홀센서는 1초마다 4채널을 전부 보낸다.
 * 그대로 무선으로 흘리면 전파법 duty cycle 한도를 크게 넘는다. lora_frame.c
 * 의 리미터가 초과분을 버리므로 위법 송신은 막히지만, 그만큼 홀 상태가
 * 유실된다. 무선 전환 시 전송 주기 정책을 다시 정해야 한다. */
#define SENSOR_TX_UART 1
#define SENSOR_TX_LORA 1

/* 1 = human-readable ANSI dashboard on UART for watching in PuTTY (fire/
 * occupied highlighted, screen redraws in place). 0 = raw SENSOR:/FIRE:
 * lines the Raspberry Pi parser expects. These are mutually exclusive on
 * the same wire -- only flip this to 1 for local PuTTY-only testing before
 * the Pi is actually wired up; must be 0 for real Pi integration. */
#define SENSOR_DEBUG_UI 0

/* 1 = also print the per-window flame energy/delta as a comment line, so the
 * thresholds can be retuned without a debugger. Not Pi-safe: keep at 0 for
 * integration. */
#define FLAME_DEBUG_ENERGY 0

/* ==========================================================================
 * 보드 구성
 * ==========================================================================
 *
 * 여기서 정하지 않는다. board_config.h 의 BOARD_SELECT 한 줄이 정본이고,
 * 아래는 그것을 이 파일이 쓰는 이름으로 옮겨 놓은 것뿐이다.
 *
 * 보드를 바꾸려면 board_config.h 만 열면 된다. 노드 ID 는 링크 진단·재부팅
 * 감지·하행 라우팅에 쓰인다.
 */
#define SENSOR_NODE_ID     BOARD_NODE_ID
#define SENSOR_HALL_COUNT  BOARD_HALL_COUNT
#define SENSOR_HALL_BASE   BOARD_HALL_BASE
#define SENSOR_HAS_FLAME   BOARD_HAS_FLAME

/* 슬롯 인덱스(0-based) -> 전선에 나가는 센서 번호 */
#define SENSOR_HALL_ID(idx)  ((unsigned)(SENSOR_HALL_BASE + (idx)))

/*
 * 홀 채널별 최소 재전송 간격(ms).
 *
 * 첫 상태 변화는 항상 즉시 나간다. 이 쿨다운은 *그 다음* 변화부터 적용되며,
 * 쿨다운 중에 생긴 변화는 버리지 않고 최신 상태만 기억했다가 쿨다운이
 * 끝날 때 한 번 보낸다(합치기). 그래서 채터링이 나도 프레임 수는 늘지 않고,
 * 수신측 상태는 최대 이 시간 안에 반드시 맞아진다.
 *
 * 값의 근거 -- 4채널이 계속 떨어도 duty 예산을 넘지 않는 선:
 *   4채널 x (60초 / 5초) x 13.5ms = 648ms
 *   + 화재 연소 중 텔레메트리 169ms = 817ms  (내부 예산 960ms, 법정 1200ms)
 *
 * 이게 없으면 채터링 시 duty 리미터가 프레임을 *버리고*, 버려진 게 상태
 * 변화라면 수신측은 다음 하트비트(30초)까지 틀린 상태를 들고 있게 된다.
 * 전파법을 지키려다 데이터 정합성을 잃는 셈이라 그쪽이 더 나쁘다.
 */
#define SENSOR_HALL_COOLDOWN_MS      5000u

/*
 * DETECTED 인 동안 energy 를 다시 보내는 간격(ms).
 *
 * 연속 스트리밍은 불가능하다 -- 1초 간격이면 분당 780ms 로 duty 예산의
 * 81% 를 화재 하나가 먹는다. 대신 평상시에는 조용하고(30초 하트비트),
 * 불이 났을 때만 촘촘해지도록 했다. Qt 는 그때 5초 간격 곡선을 얻는다.
 */
#define SENSOR_FLAME_TELEMETRY_MS    5000u

/* slot_index is 0-3; sent to the Pi as HALL01..HALL04 (1-based). */
void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied);
/* verdict: 1 = DETECTED, 0 = CLEARED. energy 는 1~20Hz 대역 에너지로,
 * 이제 전선에 실려 나간다(Qt 시각화용). 판정은 여전히 STM32 가 한다. */
void SensorProtocol_SendFlameStatus(uint8_t verdict, float energy);
void SensorProtocol_SendFlameEnergyDebug(float raw_avg, float baseline, float energy,
                                         float delta, uint8_t raw_verdict,
                                         uint8_t votes, uint8_t verdict);

/* Dashboard mode only (SENSOR_DEBUG_UI=1) -- no-ops when disabled. */
void SensorDashboard_Init(void);
void SensorDashboard_UpdateHall(uint8_t slot_index, uint8_t occupied);
void SensorDashboard_UpdateFlame(uint8_t verdict, float energy);

#endif /* __SENSOR_PROTOCOL_H */

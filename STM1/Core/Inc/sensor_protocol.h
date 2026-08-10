#ifndef __SENSOR_PROTOCOL_H
#define __SENSOR_PROTOCOL_H

#include <stdint.h>

/* Wire format consumed by the Pi's SensorProtocolParser:
 *   SENSOR:HALL01:OCCUPIED|VACANT:<seq>\r\n     (parser: SENSOR:<id>:<state>[:seq][:ts])
 *   FIRE:FLAME01:DETECTED|CLEARED:<seq>\r\n     (parser: FIRE:<id>:<state>[:seq[:unix_ms]])
 * Sensor IDs must match the Pi's config/parking_slots.json (HALL01..HALL04)
 * and .env.fire.local FIRE_SENSOR_SLOT_MAP (FLAME01). Both streams share one
 * UART link, and each carries its own sequence counter so the Pi can spot
 * drops/reorders.
 *
 * Cadence differs per stream:
 *   - HALL  : all four slots reported every 1s, whether or not they changed.
 *             The Pi agreed on 1~2s polling and ignores duplicates, so this
 *             doubles as a liveness heartbeat.
 *   - FIRE  : edge-triggered, emitted only when the verdict actually flips.
 * Either way the value on the wire is the *debounced* state -- the Pi does
 * no debouncing of its own (contract: debounce is STM32's job).
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

/* slot_index is 0-3; sent to the Pi as HALL01..HALL04 (1-based). */
void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied);
/* verdict: 1 = DETECTED, 0 = CLEARED. Energy is deliberately not on the wire
 * (the Pi's parser has no field for it); use FLAME_DEBUG_ENERGY to see it. */
void SensorProtocol_SendFlameStatus(uint8_t verdict);
void SensorProtocol_SendFlameEnergyDebug(float raw_avg, float baseline, float energy,
                                         float delta, uint8_t raw_verdict,
                                         uint8_t votes, uint8_t verdict);

/* Dashboard mode only (SENSOR_DEBUG_UI=1) -- no-ops when disabled. */
void SensorDashboard_Init(void);
void SensorDashboard_UpdateHall(uint8_t slot_index, uint8_t occupied);
void SensorDashboard_UpdateFlame(uint8_t verdict, float energy);

#endif /* __SENSOR_PROTOCOL_H */

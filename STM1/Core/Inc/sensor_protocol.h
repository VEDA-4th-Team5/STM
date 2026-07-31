#ifndef __SENSOR_PROTOCOL_H
#define __SENSOR_PROTOCOL_H

#include <stdint.h>

/* Wire format consumed by the Pi's SensorProtocolParser:
 *   SENSOR:HALL01:OCCUPIED|VACANT:<seq>\r\n     (parser: SENSOR:<id>:<state>[:seq][:ts])
 *   FIRE:FLAME01:DETECTED|CLEARED:<seq>\r\n     (parser: FIRE:<id>:<state>[:seq[:unix_ms]])
 * Sensor IDs must match the Pi's config/parking_slots.json (HALL01..HALL04)
 * and .env.fire.local FIRE_SENSOR_SLOT_MAP (FLAME01). Both streams share one
 * UART link, and each carries its own sequence counter so the Pi can spot
 * drops/reorders. The value on the wire is always the *debounced* state --
 * the Pi does no debouncing of its own (contract: debounce is STM32's job).
 */

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

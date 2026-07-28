#ifndef __SENSOR_PROTOCOL_H
#define __SENSOR_PROTOCOL_H

#include <stdint.h>

/* 1 = human-readable ANSI dashboard on UART for watching in PuTTY (ALERT/
 * OCCUPIED highlighted, screen redraws in place). 0 = raw SENSOR:/FLAME:
 * lines the Raspberry Pi parser expects. These are mutually exclusive on
 * the same wire -- only flip this to 1 for local PuTTY-only testing before
 * the Pi is actually wired up; must be 0 for real Pi integration. */
#define SENSOR_DEBUG_UI 0

/* slot_index is 0-3; sent to the Pi as sensor_01..sensor_04 (1-based). */
void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied);
void SensorProtocol_SendFlameStatus(uint8_t verdict, float energy);

/* Dashboard mode only (SENSOR_DEBUG_UI=1) -- no-ops when disabled. */
void SensorDashboard_Init(void);
void SensorDashboard_UpdateHall(uint8_t slot_index, uint8_t occupied);
void SensorDashboard_UpdateFlame(uint8_t verdict, float energy);

#endif /* __SENSOR_PROTOCOL_H */

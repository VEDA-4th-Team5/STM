#ifndef __SENSOR_PROTOCOL_H
#define __SENSOR_PROTOCOL_H

#include <stdint.h>

void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied);
void SensorProtocol_SendFlameStatus(uint8_t verdict, float energy);

#endif /* __SENSOR_PROTOCOL_H */

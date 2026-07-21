/* Formats sensor events as the plain-text lines the Raspberry Pi's parser
 * already expects (see docs/STM1_pipeline_test_2026-07-21.md). No CRC/binary
 * framing here on purpose - nothing on the receiving end consumes it. */
#include "sensor_protocol.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* Each stream has its own counter so the Pi can detect dropped/out-of-order
 * packets; hall_sequence is shared across all 4 slots, not per-slot. */
static uint32_t hall_sequence = 0;
static uint32_t flame_sequence = 0;

void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied)
{
  char line[48];
  hall_sequence++;
  int len = snprintf(line, sizeof(line), "SENSOR:sensor_%02u:%s:%lu\r\n",
                      (unsigned int)(slot_index + 1),
                      occupied ? "OCCUPIED" : "VACANT",
                      (unsigned long)hall_sequence);
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
}

void SensorProtocol_SendFlameStatus(uint8_t verdict, float energy)
{
  char line[48];
  flame_sequence++;
  int len = snprintf(line, sizeof(line), "FLAME:flame_01:%s:%lu:%.4f\r\n",
                      verdict ? "ALERT" : "CLEAR",
                      (unsigned long)flame_sequence,
                      (double)energy);
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
}

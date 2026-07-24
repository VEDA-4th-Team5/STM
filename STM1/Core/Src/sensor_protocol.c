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

#if SENSOR_DEBUG_UI
/* PuTTY-only status board: redraws the whole screen in place with ANSI
 * escapes every time any event comes in, so watching a live feed doesn't
 * mean scrolling through SENSOR:/FLAME: lines by eye. Slots show "?" until
 * their first real reading -- TaskHallSensor intentionally doesn't emit an
 * event for the baseline capture at boot, only for actual state changes. */
static uint8_t hall_state[4] = {0, 0, 0, 0};
static uint8_t hall_known[4] = {0, 0, 0, 0};
static uint8_t flame_state = 0;
static float flame_energy_val = 0.0f;
static uint8_t flame_known = 0;

static void uart_puts(const char *s)
{
  HAL_UART_Transmit(&huart2, (uint8_t *)s, (uint16_t)strlen(s), HAL_MAX_DELAY);
}

static void render_dashboard(void)
{
  char line[64];

  uart_puts("\x1b[2J\x1b[H"); /* clear screen, cursor home */
  uart_puts("=== STM1 Sensor Dashboard ===\r\n\r\n");
  uart_puts("Hall Sensors:\r\n");
  for (uint8_t i = 0; i < 4; i++)
  {
    const char *color = !hall_known[i] ? "\x1b[90m" : (hall_state[i] ? "\x1b[31m" : "\x1b[32m");
    const char *label = !hall_known[i] ? "?" : (hall_state[i] ? "OCCUPIED" : "VACANT");
    snprintf(line, sizeof(line), "  slot %u: %s%-8s\x1b[0m\r\n", (unsigned)(i + 1), color, label);
    uart_puts(line);
  }

  uart_puts("\r\nFlame Sensor:\r\n");
  if (flame_known)
  {
    const char *color = flame_state ? "\x1b[31m" : "\x1b[32m";
    const char *label = flame_state ? "ALERT" : "CLEAR";
    snprintf(line, sizeof(line), "  flame_01: %s%-5s\x1b[0m  (energy=%.2f)\r\n",
             color, label, (double)flame_energy_val);
  }
  else
  {
    snprintf(line, sizeof(line), "  flame_01: \x1b[90m?\x1b[0m\r\n");
  }
  uart_puts(line);

  uart_puts("\r\n(debug UI mode -- raw SENSOR:/FLAME: lines suppressed, not Pi-safe)\r\n");
}

void SensorDashboard_Init(void)
{
  render_dashboard();
}

void SensorDashboard_UpdateHall(uint8_t slot_index, uint8_t occupied)
{
  if (slot_index < 4)
  {
    hall_state[slot_index] = occupied;
    hall_known[slot_index] = 1;
  }
  render_dashboard();
}

void SensorDashboard_UpdateFlame(uint8_t verdict, float energy)
{
  flame_state = verdict;
  flame_energy_val = energy;
  flame_known = 1;
  render_dashboard();
}
#else
void SensorDashboard_Init(void) {}
void SensorDashboard_UpdateHall(uint8_t slot_index, uint8_t occupied) { (void)slot_index; (void)occupied; }
void SensorDashboard_UpdateFlame(uint8_t verdict, float energy) { (void)verdict; (void)energy; }
#endif

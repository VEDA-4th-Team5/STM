/* Formats sensor events as the plain-text lines the Raspberry Pi's parser
 * already expects (see docs/STM1_pipeline_test_2026-07-21.md).
 *
 * 전송 경로는 sensor_protocol.h 의 SENSOR_TX_UART / SENSOR_TX_LORA 로 고른다.
 * 문자열 생성은 여기 한 곳에만 있고 실제 송신은 emit_line() 이 분기하므로,
 * 무선으로 바꿔도 태스크 로직(freertos.c)은 건드릴 필요가 없다.
 * README 에 적어둔 "LoRa 전송으로 전환 시 이 함수 내부만 바꾸면 되도록
 * 분리해둠" 이 실제로 여기서 쓰인다. */
#include "sensor_protocol.h"
#include "usart.h"
#include <stdio.h>
#include <string.h>

/* SENSOR_TX_LORA 가 0이어도 헤더는 넣는다 -- 메시지 타입 상수와 bool 이
 * 필요하고, 실제 코드는 --gc-sections 가 링크에서 걷어낸다. */
#include "lora_frame.h"

/* Each stream has its own counter so the Pi can detect dropped/out-of-order
 * packets; hall_sequence is shared across all 4 slots, not per-slot. */
static uint32_t hall_sequence = 0;
static uint32_t flame_sequence = 0;

/**
 * 완성된 한 줄을 선택된 경로로 내보낸다.
 * @param seq       Pi 문서 권장대로 payload 안의 seq 와 같은 값을 프레임
 *                  transport sequence 로도 쓴다.
 * @param critical  LoRa duty 리미터를 우회할지 여부. 화재만 true.
 */
static void emit_line(const char *line, int len, uint8_t msg_type,
                      uint32_t seq, bool critical)
{
  if ((line == NULL) || (len <= 0))
  {
    return;
  }

#if SENSOR_TX_UART
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
#endif

#if SENSOR_TX_LORA
  (void)LoRaFrame_SendLine(msg_type, seq, line, critical);
#else
  (void)msg_type; (void)seq; (void)critical;
#endif
}

void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied)
{
  char line[48];
  hall_sequence++;
  int len = snprintf(line, sizeof(line), "SENSOR:HALL%02u:%s:%lu\r\n",
                      (unsigned int)(slot_index + 1),
                      occupied ? "OCCUPIED" : "VACANT",
                      (unsigned long)hall_sequence);
  /* 홀은 1초마다 4개씩 나가므로 duty 리미터 대상 */
  emit_line(line, len, LORA_MSG_SENSOR_EVENT, hall_sequence, false);
}

void SensorProtocol_SendFlameStatus(uint8_t verdict)
{
  char line[48];
  flame_sequence++;
  int len = snprintf(line, sizeof(line), "FIRE:FLAME01:%s:%lu\r\n",
                      verdict ? "DETECTED" : "CLEARED",
                      (unsigned long)flame_sequence);
  /* 화재는 엣지 트리거라 드물고 놓치면 안 되므로 리미터를 우회한다 */
  emit_line(line, len, LORA_MSG_SENSOR_EVENT, flame_sequence, true);
}

void SensorProtocol_SendFlameEnergyDebug(float raw_avg, float baseline, float energy,
                                         float delta, uint8_t raw_verdict,
                                         uint8_t votes, uint8_t verdict)
{
#if FLAME_DEBUG_ENERGY
  char line[112];
  int len = snprintf(line, sizeof(line),
                      "# raw=%.1f base=%.1f e=%.4f d=%.1f hit=%u votes=%u -> %s\r\n",
                      (double)raw_avg, (double)baseline, (double)energy, (double)delta,
                      (unsigned)raw_verdict, (unsigned)votes,
                      verdict ? "DETECTED" : "CLEARED");
  HAL_UART_Transmit(&huart2, (uint8_t *)line, (uint16_t)len, HAL_MAX_DELAY);
#else
  (void)raw_avg; (void)baseline; (void)energy;
  (void)delta; (void)raw_verdict; (void)votes; (void)verdict;
#endif
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

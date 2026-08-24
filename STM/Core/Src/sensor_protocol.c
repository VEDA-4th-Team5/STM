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

/* v1.1: 노드 단일 카운터. 예전엔 홀/화재가 각각 셌는데, 수신측 시퀀스 가드가
 * 단일 스트림으로 보고 있어서 홀 141 다음 화재 1 이 오면 역행으로 판정했다.
 * 이제 프레임 transport seq 와 항상 같은 값이다.
 * 재부팅하면 0 으로 돌아간다 -- 수신측은 이를 노드 재시작으로 해석해야 한다. */
static uint32_t node_sequence = 0;

/* 마지막으로 내보낸 화재 라인. 재전송에서 **같은 seq 로 똑같이** 다시
 * 보내기 위해 통째로 들고 있는다. 다시 만들면 node_sequence 가 올라가
 * 수신측이 별개 이벤트로 받아들인다(EVDA-124). */
static char     flame_last_line[64];
static int      flame_last_len = 0;
static uint32_t flame_last_seq = 0;

/**
 * 완성된 한 줄을 선택된 경로로 내보낸다.
 * @param seq       Pi 문서 권장대로 payload 안의 seq 와 같은 값을 프레임
 *                  transport sequence 로도 쓴다.
 * @param critical  LoRa duty 리미터를 우회할지 여부. 화재만 true.
 */
/**
 * snprintf 반환값을 그대로 쓰면 안 되기 때문에 한 곳에서 막는다.
 *
 * snprintf 는 버퍼가 모자라면 *잘린 길이* 가 아니라 *쓰였어야 할 길이* 를
 * 돌려준다. 그 값을 emit_line() 에 넘기면 line[] 범위를 넘어 읽어서 송신하게
 * 된다. 지금 페이로드가 40자 남짓이라 실제로 넘치진 않지만, 노드 이름이나
 * 센서 이름이 길어지는 순간 조용히 터진다.
 *
 * 잘렸으면 0 을 돌려 아예 보내지 않는다. 반쪽 프레임은 CRC 는 맞는데 내용이
 * 틀린 상태라, 수신측이 잘못된 값을 정상으로 받아들이게 된다. 안 보내는
 * 편이 낫다 -- 홀은 다음 하트비트에, 화재는 다음 상태 변화에 다시 나간다.
 *
 * @return 실제로 버퍼에 들어간 길이. 잘렸거나 인코딩 오류면 0.
 */
static int safe_len(int printed, size_t bufsize)
{
  if (printed < 0) return 0;
  if ((size_t)printed >= bufsize) return 0;
  return printed;
}

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
  char line[64];
  node_sequence++;
  int len = snprintf(line, sizeof(line), "SENSOR:%s:HALL%02u:%s:%lu\r\n",
                      SENSOR_NODE_ID,
                      SENSOR_HALL_ID(slot_index),
                      occupied ? "OCCUPIED" : "VACANT",
                      (unsigned long)node_sequence);
  len = safe_len(len, sizeof(line));
  /* 홀은 채터링이 나면 양이 늘 수 있으므로 duty 리미터 대상으로 둔다.
   * 다만 TaskHallSensor 의 채널별 쿨다운이 앞단에서 이미 양을 묶어주므로
   * 정상 동작에서는 리미터에 걸릴 일이 없다. */
  emit_line(line, len, LORA_MSG_SENSOR_EVENT, node_sequence, false);
}

void SensorProtocol_SendFlameStatus(uint8_t verdict, float energy)
{
  char line[64];
  node_sequence++;
  /* energy 는 1~20Hz 대역 에너지. 실측 범위가 0~103 정도라 소수 둘째 자리면
   * 충분하다. (-u _printf_float 가 링크 옵션에 있어 %f 가 동작한다) */
  int len = snprintf(line, sizeof(line), "FIRE:%s:FLAME01:%s:%lu:%.2f\r\n",
                      SENSOR_NODE_ID,
                      verdict ? "DETECTED" : "CLEARED",
                      (unsigned long)node_sequence,
                      (double)energy);
  len = safe_len(len, sizeof(line));
  len = safe_len(len, sizeof(line));

  /* 재전송용으로 보관한다(EVDA-124). 다시 만들어 보내면 node_sequence 가
   * 올라가서 수신측이 별개 이벤트로 받아들이므로, 라인을 통째로 들고 있다가
   * 같은 seq 로 똑같이 내보내야 한다. */
  if ((len > 0) && (len <= (int)sizeof(flame_last_line)))
  {
    memcpy(flame_last_line, line, (size_t)len);
    flame_last_len = len;
    flame_last_seq = node_sequence;
  }
  else
  {
    flame_last_len = 0;
  }

  /* 화재는 놓치면 그걸로 끝이라 리미터를 우회한다. 홀은 상태가 다시
   * 보고되지만 화재의 발생 순간은 다시 오지 않는다. */
  emit_line(line, len, LORA_MSG_SENSOR_EVENT, node_sequence, true);
}

void SensorProtocol_ResendLastFlame(void)
{
  /* 바이트 단위로 동일하게, 같은 seq 로 다시 보낸다. 수신측 시퀀스 가드가
   * 중복으로 보고 버리므로 이벤트가 두 번 기록되지 않는다. 아직 화재
   * 라인을 만든 적이 없으면 flame_last_len 이 0 이라 emit_line 이 무시한다. */
  emit_line(flame_last_line, flame_last_len, LORA_MSG_SENSOR_EVENT,
            flame_last_seq, true);
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

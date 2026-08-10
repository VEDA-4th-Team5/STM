/* Parses the Pi's AlertCommand lines and drives the plate-illumination
 * LEDs. See alert_command.h for the wire format and the ISR/task split
 * rationale. */
#include "alert_command.h"

#include "main.h"
#include "usart.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdint.h>
#include <stdio.h>   /* ALERT_DEBUG_LOG */

#define ALERT_LINE_MAX 48

/* Pi's OFF is not guaranteed to arrive (link drop, Pi restart mid-capture),
 * so each LED forces itself off this long after its own last ON if nothing
 * else says otherwise. Must stay comfortably above PLATE_LED_SETTLE_MS +
 * worst-case capture time on the Pi side -- see the "STM32 페일세이프"
 * section of Pi_Server/docs/UART_LORA_PROTOCOL.md, which currently assumes
 * this exact value. Keep the two in sync if this changes. */
#define ALERT_LED_FAILSAFE_MS 5000U

/* 1 = 수신한 명령의 수락/거부를 USART2 콘솔에 남긴다.
 * Pi 링크가 LoRa 로 옮겨가면서 USART2 는 콘솔 전용이라 프로토콜을 오염시키지
 * 않는다. 하행이 안 먹을 때 원인을 가르는 유일한 단서라 기본으로 켜둔다. */
#define ALERT_DEBUG_LOG 1

/* 1 = 번호판 LED 4개 중 하나라도 켜지면 보드 내장 LED(LD2, PA5)도 같이 켠다.
 *
 * 번호판 LED 는 PA0/PA1/PA4/PB0 에 외부로 다는 부품이라, 아직 안 달았거나
 * 배선이 잘못돼 있으면 명령이 정상 처리돼도 눈에 보이는 변화가 없다.
 * 그때 "명령이 안 먹은 것"과 "LED 가 없는 것"을 구분하려면 이 미러가 필요하다.
 * 실배치 때는 0 으로 두면 된다. */
#define ALERT_MIRROR_LD2 1

/* Slot order matches sensor_protocol.c's HALL01..HALL04 numbering (n-1). */
static GPIO_TypeDef *const led_port_[4] = {
  PLATE_LED1_GPIO_Port, PLATE_LED2_GPIO_Port, PLATE_LED3_GPIO_Port,
  PLATE_LED4_GPIO_Port
};
static const uint16_t led_pin_[4] = {
  PLATE_LED1_Pin, PLATE_LED2_Pin, PLATE_LED3_Pin, PLATE_LED4_Pin
};

/* Touched from both AlertCommand_Task (TaskCommandRX) and
 * AlertCommand_FailsafeCallback (FreeRTOS Timer Service Task). Each element
 * is a single uint8_t set/checked as a plain flag, never read-modify-write
 * across the two sites, so a race just picks one of two harmless outcomes
 * (LED ends up on with a fresh timer, or off) -- not worth a mutex for a
 * flag this simple, same call the rest of this file makes elsewhere. */
static volatile uint8_t led_on_[4] = {0};

static osTimerId_t failsafe_timer_[4];
static const osTimerAttr_t failsafe_timer_attr_[4] = {
  {.name = "plateLed1FailsafeTimer"},
  {.name = "plateLed2FailsafeTimer"},
  {.name = "plateLed3FailsafeTimer"},
  {.name = "plateLed4FailsafeTimer"},
};

static osSemaphoreId_t line_ready_sem_;
static const osSemaphoreAttr_t line_ready_sem_attr_ = {
  .name = "alertLineReadySem"
};

/* ISR-only state: only AlertCommand_OnByteReceived touches these, so no
 * locking is needed even though it runs at IRQ priority. */
static uint8_t rx_byte_;
static char line_buf_[ALERT_LINE_MAX];
static size_t line_len_;
static uint8_t line_overflow_;

/* Written by the ISR right before releasing line_ready_sem_, read by
 * AlertCommand_Task right after acquiring it. If lines somehow arrived
 * faster than the task drains them, the task just sees the newer line and
 * the older one is silently dropped -- acceptable for a low-volume,
 * ACK-less control channel like this one. */
static char pending_line_[ALERT_LINE_MAX];

static void AlertCommand_FailsafeCallback(void *argument)
{
  uint32_t slot = (uint32_t)(uintptr_t)argument;
  if (slot >= 4U) return;
  if (led_on_[slot])
  {
    HAL_GPIO_WritePin(led_port_[slot], led_pin_[slot], GPIO_PIN_RESET);
    led_on_[slot] = 0U;

#if ALERT_MIRROR_LD2
    {
      uint8_t any = 0U;
      for (int i = 0; i < 4; i++) any |= led_on_[i];
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                        any ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
#endif
  }
}

void AlertCommand_Init(void)
{
  line_ready_sem_ = osSemaphoreNew(1, 0, &line_ready_sem_attr_);
  for (uint32_t slot = 0U; slot < 4U; slot++)
  {
    failsafe_timer_[slot] = osTimerNew(AlertCommand_FailsafeCallback,
                                       osTimerOnce, (void *)(uintptr_t)slot,
                                       &failsafe_timer_attr_[slot]);
  }
}

void AlertCommand_StartReceive(void)
{
  HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);
}

void AlertCommand_SubmitLine(const char *line, uint16_t len)
{
  if (line == NULL) return;
  if (len >= ALERT_LINE_MAX) len = ALERT_LINE_MAX - 1U;

  /* pending_line_ 은 한 칸짜리라, 두 입력원(USART2 ISR / LoRa 태스크)이
   * 동시에 쓰면 반쯤 섞인 줄이 나올 수 있다. 복사 구간만 짧게 막는다.
   * 30바이트 남짓이라 인터럽트 지연은 마이크로초 단위다.
   *
   * !! 태스크가 소비하기 전에 다음 줄이 들어오면 앞의 것을 덮어쓴다.
   *    명령이 드물어 실용상 문제는 없지만, 하행이 잦아지면 큐로 바꿔야 한다. */
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memcpy(pending_line_, line, len);
  pending_line_[len] = '\0';
  __set_PRIMASK(primask);

  osSemaphoreRelease(line_ready_sem_);
}

void AlertCommand_OnByteReceived(void)
{
  const char received = (char)rx_byte_;
  /* Re-arm first so the next byte can't be missed while this one is still
   * being buffered below. */
  HAL_UART_Receive_IT(&huart2, &rx_byte_, 1);

  if (received == '\n')
  {
    if (!line_overflow_)
    {
      size_t len = line_len_;
      if (len > 0U && line_buf_[len - 1U] == '\r') len--;
      AlertCommand_SubmitLine(line_buf_, (uint16_t)len);
    }
    line_len_ = 0U;
    line_overflow_ = 0U;
    return;
  }

  if (line_len_ < ALERT_LINE_MAX - 1U)
  {
    line_buf_[line_len_++] = received;
  }
  else
  {
    /* Drop bytes past the limit and wait for the terminator to resync,
     * instead of parsing whatever garbage happens to precede it. */
    line_overflow_ = 1U;
  }
}

/* id/id_len is the substring between the first two ':' -- not
 * NUL-terminated, so length is checked explicitly rather than assumed. */
static int MatchHallSlot(const char *id, size_t id_len)
{
  if (id_len != 6U) return -1;
  if (strncmp(id, "HALL", 4) != 0) return -1;
  if (id[4] < '0' || id[4] > '9' || id[5] < '0' || id[5] > '9') return -1;
  int n = (id[4] - '0') * 10 + (id[5] - '0');
  if (n < 1 || n > 4) return -1;
  return n - 1;
}

/* Parses "ALERT:<sensorId>:LED:<ON|OFF>[:<anything>]". Anything after the
 * ON/OFF token (the Pi's sequence number) is accepted but not read -- see
 * alert_command.h. Returns the slot index (0-3) and sets *is_on, or returns
 * -1 if the line doesn't match (unknown id, malformed, HEARTBEAT, etc.). */
static int ParseAlertLine(const char *line, uint8_t *is_on)
{
  const char *p = line;
  if (strncmp(p, "ALERT:", 6) != 0) return -1;
  p += 6;

  const char *id_start = p;
  const char *sep = strchr(p, ':');
  if (sep == NULL) return -1;
  const int slot = MatchHallSlot(id_start, (size_t)(sep - id_start));
  if (slot < 0) return -1;
  p = sep + 1;

  if (strncmp(p, "LED:", 4) != 0) return -1;
  p += 4;

  if (strncmp(p, "ON", 2) == 0 && (p[2] == '\0' || p[2] == ':'))
  {
    *is_on = 1U;
    return slot;
  }
  if (strncmp(p, "OFF", 3) == 0 && (p[3] == '\0' || p[3] == ':'))
  {
    *is_on = 0U;
    return slot;
  }
  return -1;
}

void AlertCommand_Task(void)
{
  for (;;)
  {
    osSemaphoreAcquire(line_ready_sem_, osWaitForever);

    char line[ALERT_LINE_MAX];
    memcpy(line, pending_line_, sizeof(line));

    uint8_t is_on = 0U;
    const int slot = ParseAlertLine(line, &is_on);

#if ALERT_DEBUG_LOG
    /* 명령이 어디서 끊기는지 로그 없이는 알 수 없다. LED 가 안 켜질 때
     * "전파가 안 왔다 / 파싱이 실패했다 / GPIO 는 움직였는데 LED 가 안 달려
     * 있다" 를 가르는 게 이 한 줄이다. */
    if (slot < 0)
    {
      printf("# ALERT 거부: \"%s\"\r\n", line);
    }
    else
    {
      printf("# ALERT 수락: slot=%d(%s) %s -> %s\r\n",
             slot + 1,
             (slot == 0) ? "PA0" : (slot == 1) ? "PA1" :
             (slot == 2) ? "PA4" : "PB0",
             is_on ? "ON" : "OFF",
             is_on ? "핀 HIGH" : "핀 LOW");
    }
#endif

    if (slot < 0) continue;

    if (is_on)
    {
      if (!led_on_[slot])
      {
        HAL_GPIO_WritePin(led_port_[slot], led_pin_[slot], GPIO_PIN_SET);
        led_on_[slot] = 1U;
      }
      /* Already-on ON just refreshes the failsafe window -- osTimerStart on
       * a running one-shot timer restarts it (CMSIS-RTOS2), matching the
       * "같은 센서에 ON이 연속으로 오면 타이머만 갱신" rule in
       * UART_LORA_PROTOCOL.md. */
      osTimerStart(failsafe_timer_[slot], ALERT_LED_FAILSAFE_MS);
    }
    else if (led_on_[slot])
    {
      HAL_GPIO_WritePin(led_port_[slot], led_pin_[slot], GPIO_PIN_RESET);
      led_on_[slot] = 0U;
      osTimerStop(failsafe_timer_[slot]);
    }
    /* else: OFF while already off -- ignored per protocol. */

#if ALERT_MIRROR_LD2
    {
      uint8_t any = 0U;
      for (int i = 0; i < 4; i++) any |= led_on_[i];
      HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin,
                        any ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
#endif
  }
}

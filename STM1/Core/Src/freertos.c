/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include "sensor_queue.h"
#include "sensor_protocol.h"
#include "alert_command.h"
#include "lora_e22.h"
#include "lora_frame.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern volatile uint32_t tim2_tick_count;
extern uint16_t adc_buf[64];

/* Shared mailbox: TaskHallSensor/TaskFlameSensor push SensorEvent_t here,
 * TaskPacketTX pops and sends them over UART. One queue instead of a
 * mutex-protected struct so producers/consumer stay naturally serialized. */
osMessageQueueId_t sensorEventQueueHandle;
const osMessageQueueAttr_t sensorEventQueue_attributes = {
  .name = "sensorEventQueue"
};

/* Released by HAL_ADC_ConvCpltCallback (main.c) each time the circular DMA
 * buffer finishes one full 1-second window, so TaskFlameSensor knows when
 * adc_buf is safe to read without racing the next DMA write. */
osSemaphoreId_t adcBufReadySemHandle;
const osSemaphoreAttr_t adcBufReadySem_attributes = {
  .name = "adcBufReadySem"
};

/* Released by HAL_GPIO_EXTI_Callback (main.c) on either edge of any of the
 * 4 hall D0 pins, so TaskHallSensor can block instead of polling every
 * 50ms -- same shape as adcBufReadySemHandle above. */
osSemaphoreId_t hallEdgeSemHandle;
const osSemaphoreAttr_t hallEdgeSem_attributes = {
  .name = "hallEdgeSem"
};

/* Hall debounce state, kept at file scope (was local to
 * StartTaskHallSensor) so the confirmed value and the last-reported value
 * can be compared across wakes. */
static uint8_t hall_debounced_state[4] = {0};
static uint8_t hall_candidate_state[4] = {0};
static uint32_t hall_stable_count[4] = {0};
/* Last state actually put on the wire per slot -- lets TaskHallSensor emit
 * only on a confirmed change instead of a fixed cadence, same as
 * TaskFlameSensor's last_verdict. */
static uint8_t hall_last_reported_state[4] = {0};

/* Fires every 30s regardless of GPIO activity: resends whatever
 * hall_debounced_state currently holds, purely as a liveness signal. Edge
 * changes are still reported instantly by TaskHallSensor above -- this
 * timer exists only so the Pi side can tell "nothing changed in a while"
 * apart from "the STM32/UART link died", which pure on-change reporting
 * can't distinguish. The Pi's duplicate/stale-sequence guard already
 * treats repeats as safe, so re-sending unchanged state here is fine. */
osTimerId_t hallHeartbeatTimerHandle;
const osTimerAttr_t hallHeartbeatTimer_attributes = {
  .name = "hallHeartbeatTimer"
};

/* Last verdict actually put on the wire by TaskFlameSensor, and whether the
 * boot-time baseline/first verdict has been recorded yet. Moved out of
 * StartTaskFlameSensor's locals (was last_verdict/verdict_seeded) so
 * FlameHeartbeatTimerCallback below can read the current value too. */
static uint8_t flame_last_verdict = 0;
static uint8_t flame_verdict_seeded = 0;

/* Same idea as hallHeartbeatTimerHandle: every 30s, resend the last flame
 * verdict regardless of change, purely as a liveness signal. Actual
 * ALERT/CLEAR transitions are still reported instantly by
 * TaskFlameSensor. Guarded on flame_verdict_seeded so it can't fire before
 * the boot baseline is established (see FLAME_SETTLE_WINDOWS below). */
osTimerId_t flameHeartbeatTimerHandle;
const osTimerAttr_t flameHeartbeatTimer_attributes = {
  .name = "flameHeartbeatTimer"
};
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskFlameSensor */
osThreadId_t TaskFlameSensorHandle;
const osThreadAttr_t TaskFlameSensor_attributes = {
  .name = "TaskFlameSensor",
  .stack_size = 512 * 4, /* FFT buffers alone use ~700B of stack (see .su); 256*4 left almost no margin */
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskHallSensor */
osThreadId_t TaskHallSensorHandle;
const osThreadAttr_t TaskHallSensor_attributes = {
  .name = "TaskHallSensor",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskPacketTX */
osThreadId_t TaskPacketTXHandle;
const osThreadAttr_t TaskPacketTX_attributes = {
  .name = "TaskPacketTX",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for TaskCommandRX (EVDA-194: receives the Pi's plate-LED
 * AlertCommand lines, see alert_command.c). Stack is small on purpose --
 * AlertCommand_Task() only does short string parsing and a couple of HAL/
 * RTOS calls per wake, nothing like TaskFlameSensor's FFT buffers. */
osThreadId_t TaskCommandRXHandle;
const osThreadAttr_t TaskCommandRX_attributes = {
  .name = "TaskCommandRX",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void HallHeartbeatTimerCallback(void *argument);
static void FlameHeartbeatTimerCallback(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskFlameSensor(void *argument);
void StartTaskHallSensor(void *argument);
void StartTaskPacketTX(void *argument);
void StartTaskCommandRX(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */

  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  adcBufReadySemHandle = osSemaphoreNew(1, 0, &adcBufReadySem_attributes);
  hallEdgeSemHandle = osSemaphoreNew(1, 0, &hallEdgeSem_attributes);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  hallHeartbeatTimerHandle = osTimerNew(HallHeartbeatTimerCallback, osTimerPeriodic, NULL, &hallHeartbeatTimer_attributes);
  osTimerStart(hallHeartbeatTimerHandle, 30000);

  flameHeartbeatTimerHandle = osTimerNew(FlameHeartbeatTimerCallback, osTimerPeriodic, NULL, &flameHeartbeatTimer_attributes);
  osTimerStart(flameHeartbeatTimerHandle, 30000);

  /* Creates the line-ready semaphore and the 4 per-LED failsafe one-shot
   * timers. Grouped here (not a separate RTOS_SEMAPHORES entry) because
   * AlertCommand owns both and creates them together; see alert_command.c.
   * Must run before AlertCommand_StartReceive() (main.c,
   * RTOS_PERIPH_START) so a line can never arrive before these objects
   * exist. */
  AlertCommand_Init();
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  sensorEventQueueHandle = osMessageQueueNew(8, sizeof(SensorEvent_t), &sensorEventQueue_attributes);
  /* USER CODE END RTOS_QUEUES */

  /* NOTE: CubeMX 2.2.0 has a recurring bug where regenerating FreeRTOS code
   * wipes the 4 osThreadNew(...) calls below and replaces them with an empty
   * header comment block. If you regenerate from the .ioc, check this
   * function and restore these lines if they're missing. */
  /* Create the thread(s) */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);
  TaskFlameSensorHandle = osThreadNew(StartTaskFlameSensor, NULL, &TaskFlameSensor_attributes);
  TaskHallSensorHandle = osThreadNew(StartTaskHallSensor, NULL, &TaskHallSensor_attributes);
  TaskPacketTXHandle = osThreadNew(StartTaskPacketTX, NULL, &TaskPacketTX_attributes);
  TaskCommandRXHandle = osThreadNew(StartTaskCommandRX, NULL, &TaskCommandRX_attributes);

  /* USER CODE BEGIN RTOS_THREADS */

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */

  /* USER CODE END RTOS_EVENTS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
#if LORA_DEBUG_RX
  /* LoRa 하행(Pi -> STM1) 확인용. 들어온 바이트를 그대로 USART2 에 찍는다.
   * Pi 링크가 LoRa 로 옮겨가면서 USART2 는 콘솔 전용이 됐으므로 프로토콜을
   * 오염시키지 않는다.
   *
   * !! 이건 진단용이지 하행 수신 구현이 아니다. LoRa_Recv() 는 블로킹 폴링이라
   *    폴링 사이에 도착한 바이트가 버려지고, yield 도 하지 않아 CPU 를 계속
   *    태운다. 실제로 ALERT 명령을 LoRa 로 받게 되면 인터럽트/DMA 수신으로
   *    바꾸고 이 블록은 지울 것. (지금 하행 명령은 alert_command.c 가 USART2
   *    인터럽트로 받는다 -- EVDA-194) */
  static uint8_t lora_rx[64];
  for (;;)
  {
    uint16_t n = LoRa_Recv(lora_rx, sizeof(lora_rx), 500);
    if (n > 0)
    {
      printf("# [%8lu] LORA RX %u bytes:", (unsigned long)HAL_GetTick(), (unsigned)n);
      for (uint16_t i = 0; i < n; i++) printf(" %02X", lora_rx[i]);
      printf("  |");
      for (uint16_t i = 0; i < n; i++)
      {
        char c = (char)lora_rx[i];
        printf("%c", (c >= 0x20 && c < 0x7F) ? c : '.');
      }
      printf("|\r\n");
    }
  }
#else
  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
#endif
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartTaskFlameSensor */
/**
* @brief Function implementing the TaskFlameSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskFlameSensor */
void StartTaskFlameSensor(void *argument)
{
  /* USER CODE BEGIN StartTaskFlameSensor */
  /* 64 samples @ 64Hz = 1 second window -> 1Hz per FFT bin, matching the
   * 1~20Hz flame-flicker band we care about. */
  #define FFT_SIZE 64
  /* STM32 doesn't own the final fire-alarm decision -- the Pi does -- but
   * this local verdict still needs to be a meaningful ALERT/CLEAR over
   * UART, not just raw telemetry. Both thresholds below are tuned against
   * real DFR0076 hardware: ignition/hand-movement/fluorescent-light/monitor
   * tests on 2026-07-23 (see docs/), not arbitrary placeholders. */
  /* Ambient energy measured 0.15~0.61; flame transition spikes hit 10~103.
   * 5.0 is ~8x over ambient. */
  #define FLAME_ENERGY_THRESHOLD 5.0f
  /* FFT flicker energy is the primary signal and usually holds up during a
   * burn, but once the sensor saturates the window goes flat and the AC
   * content vanishes -- energy reads exactly 0.0000, observed for up to 9
   * consecutive windows. Raw DC level is the opposite: slower, but it tracks
   * continuous IR intensity right through saturation. OR them so either one
   * alone can carry a window.
   *
   * Measured 2026-07-30 at the current sensor gain/distance:
   *   ambient  raw 46~69     -> delta 0~12      (drift only, no flame)
   *   flame    raw 1480~4095 -> delta 1427~4042 (pins at ADC full scale)
   * 300 sits ~13x above the worst ambient drift and ~4.7x below the weakest
   * flame window. (Was 40, which left only ~3x headroom over ambient.) */
  #define FLAME_DELTA_THRESHOLD 300.0f
  /* Real flame flicker is chaotic, not a clean steady oscillation -- a small
   * lighter flame can have individual 1-second windows that happen to land
   * calm and read near baseline even while genuinely burning. A strict
   * "N consecutive windows" debounce keeps getting reset by those calm
   * windows and flaps ALERT/CLEAR. A sliding K-of-N majority vote tolerates
   * a few low windows mixed in without losing the detection. */
  #define FLAME_VOTE_WINDOW 5    /* look at the last 5 one-second windows */
  #define FLAME_VOTE_THRESHOLD 3 /* ...and require at least 3 of them over threshold */

  arm_rfft_fast_instance_f32 fft_inst;
  float32_t input_buf[FFT_SIZE];
  float32_t output_buf[FFT_SIZE];
  float32_t mag_buf[FFT_SIZE / 2];

  uint8_t vote_history[FLAME_VOTE_WINDOW] = {0}; /* raw_verdict of the last N windows */
  uint8_t vote_index = 0;
  uint8_t vote_count = 0; /* running count of 1s currently in vote_history */

  /* Boot settling. Taking the very first window as the ambient baseline was
   * unreliable: right after power-up the sensor output (and the ADC/DMA
   * pipeline) hasn't settled, so the baseline could latch a bogus low value.
   * Everything after that then sat above threshold, pinning the verdict to
   * DETECTED forever -- and since it never changed again, no further edge
   * events were ever emitted, which looked like the sensor going dead.
   * Discard the first few windows, then average a few more into the baseline. */
  #define FLAME_SETTLE_WINDOWS   3  /* discarded entirely */
  #define FLAME_BASELINE_WINDOWS 3  /* averaged into the initial baseline */

  uint32_t window_count = 0;
  float32_t baseline_accum = 0.0f;
  uint8_t baseline_ready = 0;
  float32_t baseline_avg = 0.0f;

  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);

  for (;;)
  {
    /* Blocks here until main.c's ADC callback signals one full window is ready. */
    osSemaphoreAcquire(adcBufReadySemHandle, osWaitForever);

    /* adc_buf is 12-bit unsigned (0~4095); recenter around the 2048 midpoint
     * and scale to roughly -1..1 for the FFT. Also accumulate the raw sum
     * for the DC-level check below (same samples, no extra ADC work). */
    uint32_t adc_sum = 0;
    for (int i = 0; i < FFT_SIZE; i++)
    {
      adc_sum += adc_buf[i];
      input_buf[i] = ((float32_t)adc_buf[i] - 2048.0f) / 2048.0f;
    }
    float32_t raw_avg = (float32_t)adc_sum / FFT_SIZE;

    window_count++;
    if (window_count <= FLAME_SETTLE_WINDOWS)
    {
      continue; /* still settling -- this reading means nothing yet */
    }

    if (!baseline_ready)
    {
      baseline_accum += raw_avg;
      if (window_count >= FLAME_SETTLE_WINDOWS + FLAME_BASELINE_WINDOWS)
      {
        baseline_avg = baseline_accum / (float32_t)FLAME_BASELINE_WINDOWS;
        baseline_ready = 1;
      }
      continue; /* no verdict until we know what "ambient" looks like */
    }

    float32_t delta = raw_avg - baseline_avg;
    if (delta < 0.0f)
    {
      delta = -delta;
    }

    arm_rfft_fast_f32(&fft_inst, input_buf, output_buf, 0);
    arm_cmplx_mag_f32(output_buf, mag_buf, FFT_SIZE / 2);

    /* Sum magnitude across bins 1~20 (i.e. 1~20Hz) into one "energy" number
     * instead of sending all 20 bins over the wire. */
    float32_t energy = 0.0f;
    for (int bin = 1; bin <= 20; bin++)
    {
      energy += mag_buf[bin];
    }

    /* Hybrid trigger: flicker spike (ignition/movement) OR sustained DC
     * level shift (steady burn) either one counts as a hit this window. */
    uint8_t raw_verdict = (energy >= FLAME_ENERGY_THRESHOLD || delta >= FLAME_DELTA_THRESHOLD) ? 1 : 0;

    /* Slide the vote window: drop the oldest window's vote, add this one's. */
    vote_count -= vote_history[vote_index];
    vote_history[vote_index] = raw_verdict;
    vote_count += raw_verdict;
    vote_index = (vote_index + 1) % FLAME_VOTE_WINDOW;

    uint8_t debounced_verdict = (vote_count >= FLAME_VOTE_THRESHOLD) ? 1 : 0;

    /* energy no longer goes on the wire (the Pi's parser has no field for
     * it), so expose it here instead for threshold retuning. Compiled out
     * unless FLAME_DEBUG_ENERGY is on. */
    SensorProtocol_SendFlameEnergyDebug(raw_avg, baseline_avg, energy, delta,
                                        raw_verdict, vote_count, debounced_verdict);

    /* Emit only on a confirmed state change: the Pi expects fire as an edge
     * event, not a repeating status line. FlameHeartbeatTimerCallback below
     * covers the separate "is the link even alive" concern on its own 30s
     * cadence. The first window after boot seeds flame_last_verdict
     * without emitting, so a board coming up in a quiet room doesn't
     * announce a redundant CLEARED. */
    if (!flame_verdict_seeded)
    {
      flame_last_verdict = debounced_verdict;
      flame_verdict_seeded = 1;
    }
    else if (debounced_verdict != flame_last_verdict)
    {
      flame_last_verdict = debounced_verdict;

      SensorEvent_t evt = {
        .type = EVT_FLAME,
        .slot = 0,
        .state = debounced_verdict,
        .energy = energy
      };
      osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
    }
  }
  /* USER CODE END StartTaskFlameSensor */
}

/* USER CODE BEGIN Header_FlameHeartbeatTimerCallback */
/**
* @brief 30s liveness heartbeat, independent of the edge-triggered reports
* in StartTaskFlameSensor above: resends flame_last_verdict so the Pi can
* tell "quiet sensor" apart from "dead link". No-op until
* flame_verdict_seeded is set (i.e. until the boot baseline is ready) so it
* can't leak a bogus verdict before TaskFlameSensor has ever recorded one.
* Runs in the Timer Service Task context (not ISR), so calling
* osMessageQueuePut directly here is safe.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_FlameHeartbeatTimerCallback */
static void FlameHeartbeatTimerCallback(void *argument)
{
  /* USER CODE BEGIN FlameHeartbeatTimerCallback */
  (void)argument;

  if (!flame_verdict_seeded)
  {
    return;
  }

  SensorEvent_t evt = {
    .type = EVT_FLAME,
    .slot = 0,
    .state = flame_last_verdict,
    .energy = 0.0f
  };
  osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
  /* USER CODE END FlameHeartbeatTimerCallback */
}

/* USER CODE BEGIN Header_StartTaskHallSensor */
/**
* @brief Function implementing the TaskHallSensor thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskHallSensor */
void StartTaskHallSensor(void *argument)
{
  /* USER CODE BEGIN StartTaskHallSensor */
  const uint32_t SAMPLE_INTERVAL_MS = 50;  /* same cadence as the old poll loop */
  const uint32_t DEBOUNCE_COUNT = 5;       /* 5 * 50ms = 250ms stable required, unchanged */
  const uint32_t CONFIRM_ITERATIONS = 6;   /* 300ms settle window per wake: enough for one full debounce plus margin */

  /* One GPIO per sensor. The CD4051 mux was dropped after 4-channel bring-up:
   * going through it pinned every channel to OCCUPIED even with no magnet
   * present, while reading D0 directly worked. Index here is slot_index 0~3,
   * which the protocol layer maps to sensor 1~4. HALL1/2/3/4 are on
   * PA8/PB10/PB4/PB5 -- picked so no two share an EXTI line number, see
   * gpio.c. */
  GPIO_TypeDef *const hall_port[4] = {
    HALL1_D0_GPIO_Port, HALL2_D0_GPIO_Port, HALL3_D0_GPIO_Port, HALL4_D0_GPIO_Port
  };
  const uint16_t hall_pin[4] = {
    HALL1_D0_Pin, HALL2_D0_Pin, HALL3_D0_Pin, HALL4_D0_Pin
  };

  /* This used to be a 50ms poll loop that scanned all 4 channels forever
   * and reported all 4 on a fixed 1s tick regardless of change. Now the 4
   * D0 pins are EXTI (both edges, see gpio.c) and HAL_GPIO_EXTI_Callback
   * (main.c) just releases hallEdgeSemHandle -- same "ISR signals, task
   * does the work" shape as TaskFlameSensor's adcBufReadySemHandle.
   * Reporting is also edge-triggered now, same as TaskFlameSensor: only a
   * confirmed change goes out over UART, not a repeating status line. The
   * debounce logic itself (N consecutive matching reads) is unchanged --
   * only the outer drive (always-on tick vs. wake-on-edge) and the report
   * condition (fixed cadence vs. on-change) changed. */

  /* One full scan before ever blocking, seeding both the debounced and the
   * last-reported state without emitting -- same idea as TaskFlameSensor's
   * verdict_seeded boot window, just instantaneous here since there's no
   * ambient signal to average. Without this, a sensor already occupied at
   * boot would never be reported until its next physical transition. */
  for (uint8_t ch = 0; ch < 4; ch++)
  {
    uint8_t raw = (HAL_GPIO_ReadPin(hall_port[ch], hall_pin[ch]) == GPIO_PIN_RESET) ? 1 : 0;
    hall_candidate_state[ch] = raw;
    hall_stable_count[ch] = DEBOUNCE_COUNT;
    hall_debounced_state[ch] = raw;
    hall_last_reported_state[ch] = raw;
  }

  for (;;)
  {
    /* Blocks here until HAL_GPIO_EXTI_Callback signals an edge on any of
     * the 4 hall pins. */
    osSemaphoreAcquire(hallEdgeSemHandle, osWaitForever);

    for (uint32_t i = 0; i < CONFIRM_ITERATIONS; i++)
    {
      for (uint8_t ch = 0; ch < 4; ch++)
      {
        /* D0 is open-collector: idle (no magnet) floats HIGH via the pull-up,
         * and a magnet trips the comparator which sinks the line LOW. So LOW
         * is OCCUPIED, not HIGH. */
        uint8_t raw = (HAL_GPIO_ReadPin(hall_port[ch], hall_pin[ch]) == GPIO_PIN_RESET) ? 1 : 0;

        /* Debounce: only trust a new value once it's been read the same way
         * DEBOUNCE_COUNT times in a row; any different reading resets the count. */
        if (raw == hall_candidate_state[ch])
        {
          if (hall_stable_count[ch] < DEBOUNCE_COUNT)
          {
            hall_stable_count[ch]++;
          }
        }
        else
        {
          hall_candidate_state[ch] = raw;
          hall_stable_count[ch] = 1;
        }

        if (hall_stable_count[ch] >= DEBOUNCE_COUNT)
        {
          hall_debounced_state[ch] = hall_candidate_state[ch];

          /* Emit only on a confirmed change from what was last put on the
           * wire -- repeated confirmations of the same value (e.g. more
           * iterations in this window, or a later wake with no real
           * change) must not re-send. */
          if (hall_debounced_state[ch] != hall_last_reported_state[ch])
          {
            hall_last_reported_state[ch] = hall_debounced_state[ch];

            SensorEvent_t evt = {
              .type = EVT_HALL,
              .slot = ch,
              .state = hall_debounced_state[ch],
              .energy = 0.0f
            };
            osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
          }
        }
      }

      osDelay(SAMPLE_INTERVAL_MS);
    }
  }
  /* USER CODE END StartTaskHallSensor */
}

/* USER CODE BEGIN Header_HallHeartbeatTimerCallback */
/**
* @brief 30s liveness heartbeat, independent of the edge-triggered reports
* in StartTaskHallSensor above: resends whatever hall_debounced_state
* currently holds so the Pi can tell "quiet sensor" apart from "dead
* link". Runs in the Timer Service Task context (not ISR), so calling
* osMessageQueuePut directly here is safe -- same as every other non-ISR
* call site in this file.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_HallHeartbeatTimerCallback */
static void HallHeartbeatTimerCallback(void *argument)
{
  /* USER CODE BEGIN HallHeartbeatTimerCallback */
  (void)argument;

  for (uint8_t ch = 0; ch < 4; ch++)
  {
    SensorEvent_t evt = {
      .type = EVT_HALL,
      .slot = ch,
      .state = hall_debounced_state[ch],
      .energy = 0.0f
    };
    osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
  }
  /* USER CODE END HallHeartbeatTimerCallback */
}

/* USER CODE BEGIN Header_StartTaskPacketTX */
/**
* @brief Function implementing the TaskPacketTX thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskPacketTX */
void StartTaskPacketTX(void *argument)
{
  /* USER CODE BEGIN StartTaskPacketTX */
  /* Sole consumer of sensorEventQueue: turns whatever TaskHallSensor/
   * TaskFlameSensor produced into a UART line and sends it. */
  SensorEvent_t evt;

#if SENSOR_DEBUG_UI
  SensorDashboard_Init();
#endif

  for (;;)
  {
    if (osMessageQueueGet(sensorEventQueueHandle, &evt, NULL, osWaitForever) == osOK)
    {
      switch (evt.type)
      {
        case EVT_HALL:
#if SENSOR_DEBUG_UI
          SensorDashboard_UpdateHall(evt.slot, evt.state);
#else
          SensorProtocol_SendHallStatus(evt.slot, evt.state);
#endif
          break;
        case EVT_FLAME:
#if SENSOR_DEBUG_UI
          SensorDashboard_UpdateFlame(evt.state, evt.energy);
#else
          SensorProtocol_SendFlameStatus(evt.state);
#endif
          break;
      }
    }
  }
  /* USER CODE END StartTaskPacketTX */
}

/* USER CODE BEGIN Header_StartTaskCommandRX */
/**
* @brief Function implementing the TaskCommandRX thread. Blocks on the
* line-ready semaphore AlertCommand_OnByteReceived (main.c, ISR context)
* releases, then parses and acts on one AlertCommand line per wake. See
* alert_command.c/.h.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTaskCommandRX */
void StartTaskCommandRX(void *argument)
{
  /* USER CODE BEGIN StartTaskCommandRX */
  (void)argument;
  AlertCommand_Task();
  /* USER CODE END StartTaskCommandRX */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  Error_Handler();
}
/* USER CODE END Application */


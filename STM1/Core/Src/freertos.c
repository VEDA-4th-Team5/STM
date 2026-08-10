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
#include <stdbool.h>
#include "arm_math.h"
#include "sensor_queue.h"
#include "sensor_protocol.h"
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

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void StartTaskFlameSensor(void *argument);
void StartTaskHallSensor(void *argument);
void StartTaskPacketTX(void *argument);

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
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */

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
  /* RF 링크 진단용. LoRa 로 뭔가 들어오면 그대로 찍는다.
   * 우리 송신이 상대에게 닿는지는 여기서 알 수 없지만, 반대 방향(상대 -> 우리)이
   * 되는지는 알 수 있다. 한 방향이라도 통하면 안테나/RF 자체는 살아 있다는 뜻이라
   * "전파가 아예 안 나간다"와 "송신부만 문제"를 가를 수 있다.
   * 진단이 끝나면 LORA_DEBUG_RX 를 0 으로 되돌릴 것. */
  static uint8_t lora_rx[64];
  for (;;)
  {
    uint16_t n = LoRa_Recv(lora_rx, sizeof(lora_rx), 500);
    if (n > 0)
    {
      printf("# [%8lu] LORA RX %u bytes:", (unsigned long)HAL_GetTick(), (unsigned)n);
      for (uint16_t i = 0; i < n; i++)
      {
        printf(" %02X", lora_rx[i]);
      }
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

  uint8_t last_verdict = 0;    /* last verdict actually put on the wire */
  uint8_t verdict_seeded = 0;  /* 1 once the boot-time verdict has been recorded */

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

    /* Emit only on a confirmed state change, unlike the hall task's 1s
     * heartbeat: the Pi expects fire as an edge event, not a repeating
     * status line. The first window after boot seeds last_verdict without
     * emitting, so a board coming up in a quiet room doesn't announce a
     * redundant CLEARED. */
    if (!verdict_seeded)
    {
      last_verdict = debounced_verdict;
      verdict_seeded = 1;
    }
    else if (debounced_verdict != last_verdict)
    {
      last_verdict = debounced_verdict;

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
  const uint32_t POLL_PERIOD_MS = 50;
  const uint32_t DEBOUNCE_COUNT = 5; /* 5 * 50ms = 250ms stable required */

  /* One GPIO per sensor. The CD4051 mux was dropped after 4-channel bring-up:
   * going through it pinned every channel to OCCUPIED even with no magnet
   * present, while reading D0 directly worked. Index here is slot_index 0~3,
   * which the protocol layer maps to sensor 1~4. */
  GPIO_TypeDef *const hall_port[4] = {
    HALL1_D0_GPIO_Port, HALL2_D0_GPIO_Port, HALL3_D0_GPIO_Port, HALL4_D0_GPIO_Port
  };
  const uint16_t hall_pin[4] = {
    HALL1_D0_Pin, HALL2_D0_Pin, HALL3_D0_Pin, HALL4_D0_Pin
  };

  /* 보고 정책: 변화 즉시 + 10초 주기 전체 갱신.
   *
   * 원래는 1초마다 4채널을 전부 보냈다. 유선 UART 시절에는 문제가 없었지만
   * LoRa 로 넘어오면서 duty cycle 한도에 정면으로 걸린다.
   *
   *   초당 4프레임 x 60초 = 240프레임, 프레임당 공중 점유 약 13ms
   *   -> 60초에 3120ms. 10dBm 등급 예산은 1200ms(2%). 약 2.6배 초과.
   *
   * 그래서 발생 빈도 자체를 줄인다. 상태가 바뀐 슬롯만 즉시 보내고,
   * 변화가 없어도 10초마다 4채널을 갱신해 liveness 를 유지한다.
   *
   *   정상 상태: 4프레임 / 10초 = 60초에 24프레임 x 13ms = 312ms  (예산 내)
   *
   * Pi 규격은 그대로다. 같은 SENSOR: 라인, 같은 메시지 타입이고 Pi 는 중복을
   * idempotent 하게 처리하므로(DuplicateOccupiedIgnored) 주기만 느려진 것이다.
   * 오히려 변화 감지는 최대 1초 지연에서 디바운스 250ms 로 빨라진다. */
  const uint32_t REFRESH_EVERY_N_POLLS = 10000 / POLL_PERIOD_MS; /* 200 * 50ms = 10s */
  uint32_t poll_tick = 0;

  uint8_t debounced_state[4] = {0};   /* last confirmed state, sent to Pi */
  uint8_t candidate_state[4] = {0};   /* raw reading being debounced */
  uint32_t stable_count[4] = {0};     /* consecutive reads matching candidate_state */

  /* 마지막으로 실제 보낸 값. 첫 주기 갱신 전까지는 미보고 상태로 둔다. */
  uint8_t reported_state[4] = {0};
  uint8_t reported_known[4] = {0};

  for (;;)
  {
    for (uint8_t ch = 0; ch < 4; ch++)
    {
      /* D0 is open-collector: idle (no magnet) floats HIGH via the pull-up,
       * and a magnet trips the comparator which sinks the line LOW. So LOW
       * is OCCUPIED, not HIGH. */
      uint8_t raw = (HAL_GPIO_ReadPin(hall_port[ch], hall_pin[ch]) == GPIO_PIN_RESET) ? 1 : 0;

      /* Debounce: only trust a new value once it's been read the same way
       * DEBOUNCE_COUNT times in a row; any different reading resets the count. */
      if (raw == candidate_state[ch])
      {
        if (stable_count[ch] < DEBOUNCE_COUNT)
        {
          stable_count[ch]++;
        }
      }
      else
      {
        candidate_state[ch] = raw;
        stable_count[ch] = 1;
      }

      if (stable_count[ch] >= DEBOUNCE_COUNT)
      {
        debounced_state[ch] = candidate_state[ch];
      }
    }

    /* 주기 갱신인가? 그렇다면 4채널 전부, 아니면 바뀐 것만. */
    bool refresh = (++poll_tick >= REFRESH_EVERY_N_POLLS);
    if (refresh)
    {
      poll_tick = 0;
    }

    for (uint8_t ch = 0; ch < 4; ch++)
    {
      bool changed = (!reported_known[ch]) ||
                     (debounced_state[ch] != reported_state[ch]);

      if (!refresh && !changed)
      {
        continue;
      }

      SensorEvent_t evt = {
        .type = EVT_HALL,
        .slot = ch,
        .state = debounced_state[ch],
        .energy = 0.0f
      };
      osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);

      reported_state[ch] = debounced_state[ch];
      reported_known[ch] = 1;
    }

    osDelay(POLL_PERIOD_MS);
  }
  /* USER CODE END StartTaskHallSensor */
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

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
  (void)xTask;
  (void)pcTaskName;
  Error_Handler();
}
/* USER CODE END Application */


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
  /* Infinite loop */
  for(;;)
  {
//      printf("TIM2 tick count in 1 sec: %u\r\n", (unsigned int)tim2_tick_count);
//      tim2_tick_count = 0;
      osDelay(1000);

//      printf("ADC buf[0..7]: ");
//      for (int i = 0; i < 8; i++) // putty ?��?��?�� 출력 코드
//      {
//          printf("%u ", adc_buf[i]);
//      }
//      printf("\r\n");
  }
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
  /* Placeholder only: STM32 does not decide the real fire threshold, the Pi does.
     This local verdict just lets us see something meaningful over UART before that. */
  #define FLAME_ENERGY_THRESHOLD 5.0f
  /* Raw single-window energy is noisy (hand jitter/distance changes land in
   * the same 1~20Hz band as real flicker), so the reported verdict only
   * flips after this many consecutive 1-second windows agree -- same idea
   * as TaskHallSensor's debounce, applied to ALERT/CLEAR instead of a GPIO
   * read. The raw energy value is still sent every window regardless. */
  #define FLAME_DEBOUNCE_COUNT 3

  arm_rfft_fast_instance_f32 fft_inst;
  float32_t input_buf[FFT_SIZE];
  float32_t output_buf[FFT_SIZE];
  float32_t mag_buf[FFT_SIZE / 2];

  uint8_t debounced_verdict = 0; /* last confirmed verdict, sent to Pi */
  uint8_t candidate_verdict = 0; /* raw per-window verdict being debounced */
  uint32_t stable_count = 0;     /* consecutive windows matching candidate_verdict */

  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);

  for (;;)
  {
    /* Blocks here until main.c's ADC callback signals one full window is ready. */
    osSemaphoreAcquire(adcBufReadySemHandle, osWaitForever);

    /* adc_buf is 12-bit unsigned (0~4095); recenter around the 2048 midpoint
     * and scale to roughly -1..1 for the FFT. */
    for (int i = 0; i < FFT_SIZE; i++)
    {
      input_buf[i] = ((float32_t)adc_buf[i] - 2048.0f) / 2048.0f;
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

    uint8_t raw_verdict = (energy >= FLAME_ENERGY_THRESHOLD) ? 1 : 0;

    /* Debounce: only trust a new verdict once it's shown up FLAME_DEBOUNCE_COUNT
     * windows in a row; any window that disagrees resets the count. */
    if (raw_verdict == candidate_verdict)
    {
      if (stable_count < FLAME_DEBOUNCE_COUNT)
      {
        stable_count++;
      }
    }
    else
    {
      candidate_verdict = raw_verdict;
      stable_count = 1;
    }

    if (stable_count >= FLAME_DEBOUNCE_COUNT)
    {
      debounced_verdict = candidate_verdict;
    }

    /* Energy is reported every window (for live calibration/telemetry);
     * only the state field is debounced. */
    SensorEvent_t evt = {
      .type = EVT_FLAME,
      .slot = 0,
      .state = debounced_verdict,
      .energy = energy
    };
    osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
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

  /* Per-channel (0~3) tracking, since one physical wire (MUX_COM) carries
   * all 4 sensors one at a time depending on what MUX_A/MUX_B select. */
  uint8_t debounced_state[4] = {0};   /* last confirmed state, sent to Pi */
  uint8_t candidate_state[4] = {0};   /* raw reading being debounced */
  uint32_t stable_count[4] = {0};     /* consecutive reads matching candidate_state */
  uint8_t baseline_set[4] = {0};      /* becomes 1 once each channel has an initial value */

  for (;;)
  {
    /* Round-robin through the 4 CD4051 channels: pick a channel via
     * MUX_A/MUX_B, wait for the analog switch to settle, then read it back
     * on the shared MUX_COM line. */
    for (uint8_t ch = 0; ch < 4; ch++)
    {
      HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      osDelay(1); /* mux settle time */

      uint8_t raw = (HAL_GPIO_ReadPin(MUX_COM_GPIO_Port, MUX_COM_Pin) == GPIO_PIN_SET) ? 1 : 0;

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
        if (!baseline_set[ch])
        {
          /* first stable read after boot: record baseline, do not emit an event */
          debounced_state[ch] = candidate_state[ch];
          baseline_set[ch] = 1;
        }
        else if (candidate_state[ch] != debounced_state[ch])
        {
          /* Real state change confirmed -> only now do we notify TaskPacketTX. */
          debounced_state[ch] = candidate_state[ch];

          SensorEvent_t evt = {
            .type = EVT_HALL,
            .slot = ch,
            .state = debounced_state[ch],
            .energy = 0.0f
          };
          osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
        }
      }
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

  for (;;)
  {
    if (osMessageQueueGet(sensorEventQueueHandle, &evt, NULL, osWaitForever) == osOK)
    {
      switch (evt.type)
      {
        case EVT_HALL:
          SensorProtocol_SendHallStatus(evt.slot, evt.state);
          break;
        case EVT_FLAME:
          SensorProtocol_SendFlameStatus(evt.state, evt.energy);
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


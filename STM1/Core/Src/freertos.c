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

osMessageQueueId_t sensorEventQueueHandle;
const osMessageQueueAttr_t sensorEventQueue_attributes = {
  .name = "sensorEventQueue"
};

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
  .stack_size = 256 * 4,
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
  #define FFT_SIZE 64
  /* Placeholder only: STM32 does not decide the real fire threshold, the Pi does.
     This local verdict just lets us see something meaningful over UART before that. */
  #define FLAME_ENERGY_THRESHOLD 5.0f

  arm_rfft_fast_instance_f32 fft_inst;
  float32_t input_buf[FFT_SIZE];
  float32_t output_buf[FFT_SIZE];
  float32_t mag_buf[FFT_SIZE / 2];

  arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE);

  for (;;)
  {
    osSemaphoreAcquire(adcBufReadySemHandle, osWaitForever);

    for (int i = 0; i < FFT_SIZE; i++)
    {
      input_buf[i] = ((float32_t)adc_buf[i] - 2048.0f) / 2048.0f;
    }

    arm_rfft_fast_f32(&fft_inst, input_buf, output_buf, 0);
    arm_cmplx_mag_f32(output_buf, mag_buf, FFT_SIZE / 2);

    float32_t energy = 0.0f;
    for (int bin = 1; bin <= 20; bin++)
    {
      energy += mag_buf[bin];
    }

    uint8_t verdict = (energy >= FLAME_ENERGY_THRESHOLD) ? 1 : 0;

    SensorEvent_t evt = {
      .type = EVT_FLAME,
      .slot = 0,
      .state = verdict,
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

  uint8_t debounced_state[4] = {0};
  uint8_t candidate_state[4] = {0};
  uint32_t stable_count[4] = {0};
  uint8_t baseline_set[4] = {0};

  for (;;)
  {
    for (uint8_t ch = 0; ch < 4; ch++)
    {
      HAL_GPIO_WritePin(MUX_A_GPIO_Port, MUX_A_Pin, (ch & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      HAL_GPIO_WritePin(MUX_B_GPIO_Port, MUX_B_Pin, (ch & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
      osDelay(1); /* mux settle time */

      uint8_t raw = (HAL_GPIO_ReadPin(MUX_COM_GPIO_Port, MUX_COM_Pin) == GPIO_PIN_SET) ? 1 : 0;

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

/* USER CODE END Application */


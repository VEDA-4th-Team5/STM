/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "main.h"
#include "cmsis_os.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "arm_math.h"
#include <stdlib.h>
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

/* USER CODE BEGIN PV */
volatile uint32_t tim2_tick_count = 0;

#define ADC_BUF_SIZE 64
uint16_t adc_buf[ADC_BUF_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
    return ch;
}

/* Fires (from ISR context) every time the circular DMA buffer wraps, i.e.
 * once per full 1-second/64-sample window. Just signals TaskFlameSensor -
 * no heavy work here since we're in an interrupt handler. */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
  {
    osSemaphoreRelease(adcBufReadySemHandle);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
#if LORA_BURN_MODULE
  /* Pi 쪽 모듈처럼 MCU 없이 동작할 모듈을 준비하는 전용 모드.
   * 굽고 확인한 뒤 여기서 멈춘다 -- 센서 노드로 동작하지 않으므로 이 모드를
   * 켜둔 채 잊어버려도 바로 알아차릴 수 있다. */
  LoRa_Init();
  printf("\r\n");
  printf("##################################################\r\n");
  printf("#  LORA BURN MODE -- 센서 동작 안 함              \r\n");
  printf("#  주소 0x%04X / NETID 0x%02X                     \r\n",
         (unsigned)LORA_BURN_ADDRESS, (unsigned)LORA_BURN_NETID);
  printf("#  ch=%u / 10dBm / LBT on / 62.5k / 115200        \r\n",
         (unsigned)LORA_CH_DATA_DEFAULT);
  printf("#  C0(영구 저장) -- 전원 끊어도 유지됨            \r\n");
  printf("##################################################\r\n");

  {
    HAL_StatusTypeDef burn_st = LoRa_BurnModule(LORA_BURN_ADDRESS, LORA_BURN_NETID);

    if (burn_st == HAL_OK)
    {
      printf("\r\n[burn] 완료. 되읽기 대조까지 통과했습니다.\r\n");
      LoRa_SelfTest();   /* 최종 상태를 디코드해서 한 번 더 남긴다 */
      printf("\r\n이 모듈을 대상 보드로 옮기고, LORA_BURN_MODULE 을 0 으로\r\n");
      printf("되돌린 뒤 재빌드하세요.\r\n");
    }
    else
    {
      printf("\r\n[burn] 실패. 모듈을 옮기지 마세요.\r\n");
      printf("배선(TXD->PA10 / RXD->PA9 / M0->PB4 / M1->PB5 / 3V3 / GND) 확인 후 재시도.\r\n");
    }

    /* 센서 동작으로 넘어가지 않는다. LED 로 결과를 표시하며 정지:
     * 느린 깜빡임 = 성공, 빠른 깜빡임 = 실패. */
    for (;;)
    {
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_Delay((burn_st == HAL_OK) ? 1000 : 150);
    }
  }
#endif

#if SENSOR_TX_LORA
  /* USART1(PA9/PA10)과 M0/M1(PB4/PB5) 초기화를 드라이버가 직접 하므로
   * .ioc 를 건드리지 않는다 -- CubeMX 재생성에도 안전하다.
   * 채널/출력/LBT 는 C0 명령으로 모듈에 영구 저장되어 있지만, 다른 모듈로
   * 교체했을 때를 대비해 부팅할 때마다 한 번 더 맞춰준다.
   * 주의: LoRa_SelfTest()/LoRa_Diag() 는 printf 로 USART2 에 찍는데 그건
   * Pi 데이터 링크라 프로토콜을 오염시킨다. 브링업 때만 임시로 켤 것. */
  LoRa_Init();
  /* persist=false(C2) 로 매 부팅마다 맞춘다. C0 는 모듈 플래시에 쓰는 거라
   * 부팅마다 때리면 수명을 깎는다. STM32 가 붙어 있는 쪽은 어차피 매번
   * 설정하므로 임시 저장으로 충분하다.
   * !! Pi 쪽 모듈은 MCU 가 없어서 스스로 설정하지 못한다. 그 모듈은 이
   *    보드에 잠깐 물려서 persist=true 로 한 번 구워서 넘겨야 한다. */
  {
    HAL_StatusTypeDef lora_st =
        /* LBT 는 KC 기술기준 항목이라 켜둔다. 브링업 중 링크가 안 붙을 때
         * 잠깐 false 로 내려 원인에서 배제하는 데 썼는데, 진단이 끝났으므로
         * 되돌렸다. LBT 가 송신을 미뤄도 STM32 에 알려주지는 않으므로,
         * "LORA OK" 는 UART 로 넘겼다는 뜻이지 전파가 나갔다는 보장은 아니다. */
        LoRa_ConfigureFull(LORA_CH_DATA_DEFAULT, LORA_PWR_10DBM, LORA_LBT_ENABLE,
                           LORA_AIR_PROJECT, LORA_UART_PROJECT, false);
#if LORA_DEBUG_HEX
    /* 브링업 진단: 설정이 안 먹으면 인터록이 송신을 전부 막으므로,
     * 여기서 성공 여부를 못 보면 원인을 찾을 수 없다. */
    printf("\r\n# LORA CFG %s (ch=%u pwr=10dBm lbt=on air=62.5k uart=115200)\r\n",
           (lora_st == HAL_OK) ? "OK" : "FAIL -- 송신 차단됨",
           (unsigned)LORA_CH_DATA_DEFAULT);
    if (lora_st == HAL_OK) { LoRa_DumpRegisters(); LoRa_CheckNormalMode(); }
    if (lora_st != HAL_OK)
    {
      /* 설정이 안 먹었으면 어디서 막혔는지 자동으로 훑는다. 라인 프로브 ->
       * 단락 검사 -> baud 스윕 -> USART 우회 비트뱅잉 순서. 지그에서 E22 를
       * E220 으로 착각해 딥슬립에 빠뜨렸던 그 상황을 잡으려고 만든 루틴이다. */
      LoRa_SelfTest();
      LoRa_CheckNormalMode();
      LoRa_DetectVariant();
      LoRa_Diag();
    }
#else
    (void)lora_st;
#endif
  }
#endif
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  /* USER CODE BEGIN RTOS_PERIPH_START */
  /* adcBufReadySemHandle must exist before conversions can start firing the callback */
  HAL_TIM_Base_Start_IT(&htim2);
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buf, ADC_BUF_SIZE);
  /* USER CODE END RTOS_PERIPH_START */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  /* BYPASS not Crystal: on this Nucleo board HSE comes from the ST-Link's
   * 8MHz MCO into PH0, there's no resonator on OSC_IN/OSC_OUT.
   * PLLM=8 -> 8MHz/8=1MHz VCO input, same as before; PLLN/PLLP unchanged so
   * SYSCLK is still 84MHz and TIM2's 64Hz math doesn't need to change. */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */
  if (htim->Instance == TIM2)
  {
      tim2_tick_count++;
  }
  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

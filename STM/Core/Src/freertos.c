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
#include "arm_const_structs.h"
#include "sensor_queue.h"
#include "sensor_protocol.h"
#include "alert_command.h"
#include "lora_e22.h"
#include "lora_frame.h"
#include "usart.h"    /* huart2 -- vApplicationStackOverflowHook */
#include <string.h>
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
static uint8_t hall_debounced_state[SENSOR_HALL_COUNT] = {0};
static uint8_t hall_candidate_state[SENSOR_HALL_COUNT] = {0};
static uint32_t hall_stable_count[SENSOR_HALL_COUNT] = {0};
/* Last state actually put on the wire per slot -- lets TaskHallSensor emit
 * only on a confirmed change instead of a fixed cadence, same as
 * TaskFlameSensor's last_verdict. */
static uint8_t hall_last_reported_state[SENSOR_HALL_COUNT] = {0};

/* 채널별 쿨다운 상태 (SENSOR_HALL_COOLDOWN_MS).
 *
 * 첫 변화는 즉시 나가고, 그 뒤 쿨다운 동안 생긴 변화는 버리지 않고 pending
 * 으로 표시만 해둔다. 쿨다운이 끝나면 그때의 *최신* 상태를 한 번 보낸다.
 * 그래서 채터링이 나도 프레임 수가 늘지 않고, 수신측 상태는 늦어도 쿨다운
 * 시간 안에 반드시 맞아진다.
 *
 * 이게 없으면 채터링 시 duty 리미터가 프레임을 버리는데, 버려진 게 상태
 * 변화면 수신측은 다음 하트비트(SENSOR_HEARTBEAT_MS)까지 틀린 상태를 든다. */
static uint32_t hall_last_sent_tick[SENSOR_HALL_COUNT] = {0};
static uint8_t  hall_sent_once[SENSOR_HALL_COUNT] = {0};
static uint8_t  hall_pending[SENSOR_HALL_COUNT] = {0};

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
/* 마지막으로 계산한 1~20Hz 대역 에너지. v1.1 부터 전선에 실려 나가므로
 * 하트비트/텔레메트리에서도 이 값을 쓴다. */
static float   flame_last_energy = 0.0f;

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
  /* LoRa 하행 펌프 + 진단 printf 가 여기서 돈다. 128*4(512B)로는 printf
   * 하나에 스택이 넘친다 -- newlib 의 포맷 파싱만으로 200~300B 를 쓴다.
   * 넘치면 vApplicationStackOverflowHook -> Error_Handler() 무한루프라
   * 로그가 뚝 끊기고 원인이 안 보인다. 실제로 그 증상을 겪었다. */
  .stack_size = 512 * 4,
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
  /* 원래는 짧은 문자열 파싱 + HAL/RTOS 호출 몇 개뿐이라 128*4 로 충분했는데,
   * 명령 수락/거부 진단 로그(printf)가 붙으면서 부족해졌다. defaultTask 와
   * 같은 이유다 -- 위 주석 참고. */
  .stack_size = 384 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void HallHeartbeatTimerCallback(void *argument);
static void FlameHeartbeatTimerCallback(void *argument);
static void HallEmitIfAllowed(uint8_t ch);
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
  /* 시작은 StartDefaultTask 에서 한다. 노드별로 반 주기 어긋내야 하는데
   * 여기는 스케줄러가 아직 안 돌아서 osDelay 를 쓸 수 없다. */

#if SENSOR_HAS_FLAME
  flameHeartbeatTimerHandle = osTimerNew(FlameHeartbeatTimerCallback, osTimerPeriodic, NULL, &flameHeartbeatTimer_attributes);
  /* 5초 주기로 돌지만 CLEARED 상태에서는 SENSOR_HEARTBEAT_MS 마다만 보낸다.
   * DETECTED 인 동안에만 5초 간격 텔레메트리가 된다 -- 콜백 주석 참고. */
  osTimerStart(flameHeartbeatTimerHandle, SENSOR_FLAME_TELEMETRY_MS);
#endif

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
#if SENSOR_HAS_FLAME
  /* 화재센서가 없는 보드(STM2)에서는 태스크를 아예 만들지 않는다. FFT 버퍼
   * 때문에 스택이 2KB라, 안 쓰면서 잡아두면 힙만 축낸다. */
  TaskFlameSensorHandle = osThreadNew(StartTaskFlameSensor, NULL, &TaskFlameSensor_attributes);
#endif
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
  /* 홀 하트비트 타이머는 여기서 건다.
   *
   * 두 보드를 같이 켜면 하트비트가 정렬돼서 매 주기 같은 시점에 충돌한다.
   * LoRa 는 half-duplex 라 그러면 한쪽이 계속 지는 형태가 될 수 있고,
   * 그건 확률적 충돌보다 나쁘다 -- 특정 노드만 만성적으로 유실된다.
   * 노드별로 반 주기 어긋내면 그 상황이 구조적으로 안 생긴다.
   *
   * MX_FREERTOS_Init 에서 못 하는 이유는 그 시점에 스케줄러가 아직
   * 시작되지 않아 osDelay 가 동작하지 않기 때문이다. */
#if SENSOR_HEARTBEAT_STAGGER_MS > 0
  osDelay(SENSOR_HEARTBEAT_STAGGER_MS);
#endif
  osTimerStart(hallHeartbeatTimerHandle, SENSOR_HEARTBEAT_MS);

#if SENSOR_TX_LORA
  /* LoRa 하행 펌프.
   *
   * USART1 인터럽트가 바이트를 링버퍼에 넣고(lora_e22.c), 여기서 꺼내
   * 프레임으로 조립한다. CRC 까지 통과한 payload 만 명령 파서로 넘긴다.
   *
   * 예전에는 이 자리에서 LoRa_Recv() 블로킹 폴링을 돌렸는데, 그건 부르는
   * 동안에만 듣는 구조라 호출 사이에 온 바이트가 사라졌고(Pi PING 5개 중
   * 2개 유실) yield 도 하지 않아 CPU 를 계속 태웠다. 이제는 ISR 이 항상
   * 듣고 있으므로 여기서는 10ms 마다 훑기만 하면 된다 -- 명령 지연으로는
   * 무시할 수준이고, 그 사이 도착분은 링버퍼가 받아둔다.
   *
   * 명령 파서는 USART2 경로와 공유한다(alert_command.c). 즉 Pi 는 LoRa 로,
   * 사람은 PuTTY 로 같은 명령을 넣을 수 있다. */
  static char lora_line[LORA_FRAME_MAX_PAYLOAD + 1];
  for (;;)
  {
    uint8_t  type = 0;
    uint32_t seq = 0;
    uint16_t n = LoRaFrame_Poll(lora_line, sizeof(lora_line), &type, &seq);

    if (n == 0)
    {
      /* 수신이 꺼져 있으면 되살린다. 정상일 때는 아무 일도 안 한다. */
      if (LoRa_EnsureReceiving())
      {
        printf("# [%8lu] LORA RX 재무장 (누적 %lu회, 링버퍼 유실 %lu B)\r\n",
               (unsigned long)HAL_GetTick(),
               (unsigned long)LoRa_GetRxRearmCount(),
               (unsigned long)LoRa_GetRxOverrunCount());
      }
      osDelay(10);
      continue;
    }

#if LORA_DEBUG_RX
    printf("# [%8lu] LORA RX type=0x%02X seq=%lu: %s\r\n",
           (unsigned long)HAL_GetTick(), (unsigned)type,
           (unsigned long)seq, lora_line);
#endif

    /* 알 수 없는 줄은 파서가 조용히 무시하므로 타입으로 미리 거르지 않는다.
     * 그래야 Pi 쪽에서 형식을 바꿔도 로그로는 보인다. */
    AlertCommand_SubmitLine(lora_line, n);
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
  /* baseline 추종 계수 (EVDA-218). 1초 창 기준 시정수 약 100초.
   *
   * 크게 잡으면 baseline 이 불을 따라가 검출을 놓치고, 작게 잡으면 조명
   * 변화를 못 따라가 예전 결함이 그대로 남는다. 추종 자체가 "깨끗한 창"
   * 에서만 돌기 때문에 이 값이 안전을 혼자 책임지지는 않는다. */
  #define FLAME_BASELINE_TRACK_ALPHA 0.01f

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
   * Discard the first few windows, then average a few more into the baseline.
   * 그 뒤로는 깨끗한 창에서만 천천히 따라간다(EVDA-218). */
  #define FLAME_SETTLE_WINDOWS   3  /* discarded entirely */
  #define FLAME_BASELINE_WINDOWS 3  /* averaged into the initial baseline */

  uint32_t window_count = 0;
  float32_t baseline_accum = 0.0f;
  uint8_t baseline_ready = 0;
  float32_t baseline_avg = 0.0f;

  /* arm_rfft_fast_init_f32(&fft_inst, FFT_SIZE) 를 쓰지 않고 인스턴스를
   * 직접 채운다. 동작은 완전히 같고, 플래시를 78.6KB 아낀다.
   *
   * 그 함수는 Sint->fftLen(= FFT_SIZE/2) 으로 switch 를 돌면서 지원하는
   * 모든 길이의 테이블을 참조한다. 분기 하나만 실행되지만 참조는 전부
   * 남으므로 -ffunction-sections/--gc-sections 로도 못 버린다. 64포인트만
   * 쓰는데 4096포인트까지의 twiddle 이 통째로 링크됐다:
   *
   *     FFT 테이블 총합   78,936 B   (텍스트 132,520 B 의 59.6%)
   *     실제 필요분          608 B
   *
   * 아래는 그 switch 의 case 32U 가 채우는 값과 필드 단위로 동일하다
   * (Sint->fftLen = FFT_SIZE/2 = 32 이므로 그 분기가 선택된다):
   *
   *     Sint.fftLen        32
   *     Sint.pTwiddle      twiddleCoef_32
   *     Sint.pBitRevTable  armBitRevIndexTable32
   *     Sint.bitRevLength  ARMBITREVINDEXTABLE_32_TABLE_LENGTH
   *     fftLenRFFT         64
   *     pTwiddleRFFT       twiddleCoef_rfft_64
   *
   * arm_cfft_sR_f32_len32 (arm_const_structs.c) 가 앞의 네 필드를 그대로
   * 들고 있어서 대입 한 번이면 된다.
   *
   * !! FFT_SIZE 를 바꾸면 아래 두 상수도 같이 바꿔야 한다 --
   *    arm_cfft_sR_f32_len<FFT_SIZE/2> 와 twiddleCoef_rfft_<FFT_SIZE>. */
  fft_inst.Sint = arm_cfft_sR_f32_len32;
  fft_inst.Sint.fftLen = FFT_SIZE / 2;
  fft_inst.fftLenRFFT = FFT_SIZE;
  fft_inst.pTwiddleRFFT = (float32_t *)twiddleCoef_rfft_64;

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

    /* delta 는 **위쪽 방향만** 본다.
     *
     * 예전에는 절댓값이었는데, DFR0076 은 불이 나면 값이 *올라가는* 센서라
     * 내려가는 방향까지 화재로 치는 것은 근거가 없었다. 오히려 위험했다 --
     * 센서가 빠지거나 baseline 이 밝을 때 잡히면, 어두워지는 것만으로
     * DETECTED 가 되고 그 상태로 고정됐다.
     *
     * 아래로 벗어난 것은 baseline 이 틀렸다는 뜻이므로, 화재로 보는 대신
     * 아래 추종 로직이 흡수하게 둔다. */
    float32_t delta = raw_avg - baseline_avg;
    if (delta < 0.0f)
    {
      delta = 0.0f;
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

    /* ---- baseline 추종 (EVDA-218) --------------------------------------
     *
     * baseline 을 부팅 때 한 번 잡고 끝내면, 주변 밝기가 그 뒤로 달라졌을 때
     * delta 가 영구히 임계 위에 남는다. 그러면 불이 꺼져도 CLEARED 로 못
     * 돌아오고, 상태가 안 바뀌니 엣지 이벤트도 안 나가 센서가 죽은 것처럼
     * 보인다. 예전에 부팅 직후 값으로 baseline 을 잡았다가 같은 증상을 겪고
     * settle 구간을 넣어 고쳤는데, 그건 부팅 순간만 다룬 것이라 부팅 이후
     * 어긋나는 경우가 그대로 남아 있었다.
     *
     * ★ 이 창이 깨끗할 때(raw_verdict == 0)만 따라간다.
     *
     * 이 조건이 안전의 핵심이다. raw_verdict 가 0 이라는 것은 flicker 도
     * DC 상승도 임계 아래라는 뜻이므로, 불이 조금이라도 걸리는 순간 추종이
     * 즉시 멈춘다. 판정과 무관하게 계속 따라가게 두면 연소 중에 baseline 이
     * 불을 쫓아 올라가 스스로 CLEARED 로 빠진다.
     *
     * 시정수는 약 100초(alpha = 0.01, 1초 창 기준)다. 조명 변화나 센서
     * 드리프트는 흡수하면서, 불꽃이 이 속도보다 느리게 올라오는 일은 없다 --
     * 실측 불꽃은 1480~4095 로 튀고 그 훨씬 전에 raw_verdict 가 1 이 되어
     * 추종이 멈춘다. */
    if (raw_verdict == 0)
    {
      baseline_avg += FLAME_BASELINE_TRACK_ALPHA * (raw_avg - baseline_avg);
    }

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
    /* 하트비트/텔레메트리가 최신 세기를 실어 보낼 수 있도록 매 윈도우
     * 갱신한다. 판정과 무관하게 값 자체는 계속 살아 있어야 한다. */
    flame_last_energy = energy;

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

  /* 타이머는 SENSOR_FLAME_TELEMETRY_MS(5초) 주기로 돌지만 매번 보내지는
   * 않는다. 연속 스트리밍은 duty 예산을 감당할 수 없다 -- 1초 간격이면
   * 분당 780ms 로 예산 960ms 의 81% 를 화재 하나가 먹는다.
   *
   *   CLEARED (평상시) : SENSOR_HEARTBEAT_MS 마다 한 번. 순수 liveness 신호.
   *   DETECTED (연소중) : 5초마다. 필요한 순간에만 촘촘해진다.
   *
   * 화재는 드문 사건이라 이 비대칭이 duty 에 주는 부담은 실질적으로 없고,
   * Qt 는 정작 봐야 할 때 5초 간격 곡선을 얻는다. */
  static uint32_t tick = 0;
  const uint32_t TICKS_PER_HEARTBEAT =
      SENSOR_HEARTBEAT_MS / SENSOR_FLAME_TELEMETRY_MS;  /* 10초 = 2틱 */

  tick++;
  if ((flame_last_verdict == 0) && (tick < TICKS_PER_HEARTBEAT))
  {
    return;
  }
  tick = 0;

  SensorEvent_t evt = {
    .type = EVT_FLAME,
    .slot = 0,
    .state = flame_last_verdict,
    /* 하트비트/텔레메트리도 최신 energy 를 실어 보낸다. 0 으로 보내면
     * Qt 그래프가 주기적으로 바닥을 찍는 것처럼 보인다. */
    .energy = flame_last_energy
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
  /* 보드에 실제로 달린 개수만 쓴다(SENSOR_HALL_COUNT). 배열에는 핀 4개를
   * 그대로 두되 앞에서부터 잘라 쓰는 방식이라, 홀을 2개로 줄여도 배선은
   * HALL1/HALL2 자리를 그대로 쓰면 된다. */
  GPIO_TypeDef *const hall_port[] = {
    HALL1_D0_GPIO_Port, HALL2_D0_GPIO_Port, HALL3_D0_GPIO_Port, HALL4_D0_GPIO_Port
  };
  const uint16_t hall_pin[] = {
    HALL1_D0_Pin, HALL2_D0_Pin, HALL3_D0_Pin, HALL4_D0_Pin
  };
  _Static_assert(SENSOR_HALL_COUNT <= 4, "핀 정의는 4개까지만 있다");

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
  for (uint8_t ch = 0; ch < SENSOR_HALL_COUNT; ch++)
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
     * the 4 hall pins.
     *
     * 다만 쿨다운에 걸려 대기 중인(pending) 채널이 있으면 무한정 기다리면
     * 안 된다. 채터링이 멎어 더 이상 엣지가 안 오면 그 상태 변화가 다음
     * 하트비트까지 묻히기 때문이다. 남은 쿨다운만큼만 기다렸다가
     * 깨어나 밀어낸다. */
    uint32_t wait = osWaitForever;
    for (uint8_t ch = 0; ch < SENSOR_HALL_COUNT; ch++)
    {
      if (!hall_pending[ch]) continue;
      uint32_t elapsed = osKernelGetTickCount() - hall_last_sent_tick[ch];
      uint32_t remain = (elapsed >= SENSOR_HALL_COOLDOWN_MS)
                        ? 1u : (SENSOR_HALL_COOLDOWN_MS - elapsed);
      if ((wait == osWaitForever) || (remain < wait)) wait = remain;
    }
    osSemaphoreAcquire(hallEdgeSemHandle, wait);

    /* 엣지로 깨웠든 쿨다운 만료로 깨웠든, 밀린 것부터 처리한다. */
    for (uint8_t ch = 0; ch < SENSOR_HALL_COUNT; ch++)
    {
      if (hall_pending[ch]) HallEmitIfAllowed(ch);
    }

    for (uint32_t i = 0; i < CONFIRM_ITERATIONS; i++)
    {
      for (uint8_t ch = 0; ch < SENSOR_HALL_COUNT; ch++)
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
           * change) must not re-send.
           * 쿨다운 판단은 HallEmitIfAllowed() 안에 있다. */
          HallEmitIfAllowed(ch);
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
/* 채널 하나의 확정 상태를 내보낼 수 있으면 내보내고, 쿨다운 중이면
 * pending 으로만 남긴다. 상태 자체는 hall_debounced_state 에 이미 최신값이
 * 들어 있으므로, 나중에 보낼 때 자동으로 최신 상태가 나간다(합치기). */
static void HallEmitIfAllowed(uint8_t ch)
{
  if (hall_debounced_state[ch] == hall_last_reported_state[ch])
  {
    hall_pending[ch] = 0;      /* 떨다가 원래 값으로 돌아왔다 -- 보낼 것 없음 */
    return;
  }

  uint32_t now = osKernelGetTickCount();
  if (hall_sent_once[ch] &&
      ((now - hall_last_sent_tick[ch]) < SENSOR_HALL_COOLDOWN_MS))
  {
    hall_pending[ch] = 1;      /* 쿨다운 중. 끝나면 최신 상태로 나간다 */
    return;
  }

  hall_last_reported_state[ch] = hall_debounced_state[ch];
  hall_last_sent_tick[ch] = now;
  hall_sent_once[ch] = 1;
  hall_pending[ch] = 0;

  SensorEvent_t evt = {
    .type = EVT_HALL,
    .slot = ch,
    .state = hall_debounced_state[ch],
    .energy = 0.0f
  };
  osMessageQueuePut(sensorEventQueueHandle, &evt, 0, 0);
}

static void HallHeartbeatTimerCallback(void *argument)
{
  /* USER CODE BEGIN HallHeartbeatTimerCallback */
  (void)argument;

  uint32_t now = osKernelGetTickCount();

  for (uint8_t ch = 0; ch < SENSOR_HALL_COUNT; ch++)
  {
    /* 최근 SENSOR_HEARTBEAT_MS 안에 이미 보낸 채널은 건너뛴다.
     *
     * 하트비트의 목적은 "이 노드가 살아 있다"를 알리는 것뿐인데, 방금
     * 상태를 보낸 채널은 그게 이미 증명돼 있다. 무조건 다 보내면 채터링
     * 중인 채널이 쿨다운 전송과 하트비트를 둘 다 내보낸다.
     *
     * duty 최악 케이스 (홀 2채널 + 화재, 프레임당 약 14ms, 예산 960ms/60초):
     *   홀 채터링      2ch x (60/5초) x 14ms = 336 ms
     *   화재 텔레메트리 (60/5초) x 14ms      = 168 ms
     *   하트비트 (건너뛰기 없을 때)          = 252 ms
     *
     *   건너뛰면  504 ms (예산의 53%)
     *   안 건너뛰면 756 ms (예산의 79%)
     *
     * 수신측이 받는 정보는 어느 쪽이든 같다. 그래서 건너뛴다.
     *
     * ★ 이 창은 하트비트 주기와 반드시 같아야 한다. 더 길면 타이머는
     *   도는데 매번 전부 건너뛰어서 하트비트가 사실상 멈춘다. */
    if (hall_sent_once[ch] && ((now - hall_last_sent_tick[ch]) < SENSOR_HEARTBEAT_MS))
    {
      continue;
    }

    /* Heartbeat should not affect cooldown timestamps (HallEmitIfAllowed). */

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
          SensorProtocol_SendFlameStatus(evt.state, evt.energy);

          /* DETECTED 만 몇 번 더 보낸다(EVDA-124). 충돌로 한 번 씹혀도
           * 서버가 화재를 놓치지 않게 하기 위한 것이다. CLEARED 는
           * 하트비트가 다시 알려주므로 재전송하지 않는다.
           *
           * 여기서 osDelay 로 이 태스크가 잠깐 멈춘다. 그 사이 홀 이벤트는
           * 큐에 쌓이는데, 홀 쿨다운이 5초라 400ms 지연은 흡수된다.
           * 화재가 홀보다 우선이라 이 교환은 성립한다. */
          if (evt.state != 0)
          {
            for (uint8_t r = 0; r < SENSOR_FLAME_REPEAT_COUNT; r++)
            {
              osDelay(SENSOR_FLAME_REPEAT_GAP_MS);
              SensorProtocol_ResendLastFlame();
            }
          }
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

  /* 예전에는 그냥 Error_Handler() 였는데, 그러면 무한 루프에 빠지면서
   * 아무 흔적도 안 남는다. 로그가 뚝 끊긴 것만 보이고 원인은 안 보여서
   * "LoRa 가 안 된다"로 오진하게 된다 -- 실제로 그렇게 반나절을 썼다.
   *
   * printf 는 쓰지 않는다. 지금은 스택이 이미 망가진 상태라 그 자체가
   * 위험하다. HAL 로 고정 문자열만 직접 밀어낸다. */
  const char *msg = "\r\n!!! STACK OVERFLOW: ";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100);
  if (pcTaskName)
  {
    HAL_UART_Transmit(&huart2, (uint8_t *)pcTaskName,
                      (uint16_t)strlen(pcTaskName), 100);
  }
  msg = " -- 해당 태스크의 stack_size 를 키울 것\r\n";
  HAL_UART_Transmit(&huart2, (uint8_t *)msg, (uint16_t)strlen(msg), 100);

  /* LD2 빠른 깜빡임으로도 알린다. 콘솔을 안 보고 있어도 눈에 띈다. */
  for (;;)
  {
    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    HAL_Delay(80);
  }
}
/* USER CODE END Application */


/**
  ******************************************************************************
  * @file    lora_e22.c
  * @brief   EBYTE E22-900T22S(1B) driver (UART transparent transmission).
  ******************************************************************************
  */

#include "lora_e22.h"
#include "lora_frame.h"   /* LORA_DEBUG_HEX */
#include "FreeRTOS.h"     /* 프레임 송신 중 선점 차단 -- LoRa_SendTo() 주석 참고 */
#include "task.h"
#include <stdio.h>
#include <string.h>

UART_HandleTypeDef hlora;

/* 전파법 인터록. LoRa_ConfigureFull() 이 되읽기까지 성공해야 true 가 된다.
 * false 인 동안은 lora_frame.c 가 송신을 거부한다 -- 미설정 모듈은 공장
 * 기본값(채널 2 = RFID 전용, 22dBm)이라 그대로 쏘면 위법이다. */
static bool lora_configured = false;

/* 다음 프레임을 모듈에 넣어도 되는 시각(HAL tick). LoRa_SendTo() 참고. */
static uint32_t lora_tx_free_tick = 0;

/* 송신 후 모듈이 idle 로 돌아오기까지 걸린 시간(ms). 공중 점유 추정치보다
 * 크게 길면 LBT 가 채널을 기다린 것이다 -- LBT 는 그 사실을 안 알려주므로
 * 이 값이 유일한 단서다. */
static uint32_t lora_last_tx_busy_ms = 0;
static uint32_t lora_max_tx_busy_ms  = 0;

uint32_t LoRa_GetLastTxBusyMs(void) { return lora_last_tx_busy_ms; }
uint32_t LoRa_GetMaxTxBusyMs(void)  { return lora_max_tx_busy_ms; }

/* -------------------------------------------------------------------------- */
/* Low level init                                                             */
/* -------------------------------------------------------------------------- */

static void LoRa_GPIO_Init(void)
{
  GPIO_InitTypeDef gpio = {0};

  /* M0/M1 은 서로 다른 포트에 있다(PC7 / PB6). 한 번에 못 초기화하므로
   * 각각 따로 부른다 -- 핀을 옮길 때 여기를 같이 고쳐야 한다. */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  gpio.Mode  = GPIO_MODE_OUTPUT_PP;
  gpio.Pull  = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = LORA_M0_Pin;
  HAL_GPIO_Init(LORA_M0_GPIO_Port, &gpio);

  gpio.Pin = LORA_M1_Pin;
  HAL_GPIO_Init(LORA_M1_GPIO_Port, &gpio);

#if LORA_AUX_ENABLE
  /* AUX 는 모듈이 미는 출력이므로 입력으로 받는다. 풀업을 거는 이유는
   * 미배선일 때 HIGH(=idle)로 읽혀 예전 동작으로 안전하게 떨어지기 위함이다. */
  gpio.Pin   = LORA_AUX_Pin;
  gpio.Mode  = GPIO_MODE_INPUT;
  gpio.Pull  = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_AUX_GPIO_Port, &gpio);
#endif

  /* Park in normal mode until told otherwise. */
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_RESET);
}

static void LoRa_UART_Init(uint32_t baud)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_USART1_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* PA9 = USART1_TX, PA10 = USART1_RX */
  gpio.Pin       = GPIO_PIN_9 | GPIO_PIN_10;
  gpio.Mode      = GPIO_MODE_AF_PP;
  gpio.Pull      = GPIO_PULLUP;          /* idle-high keeps RX quiet when unplugged */
  gpio.Speed     = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &gpio);

  hlora.Instance          = USART1;
  hlora.Init.BaudRate     = baud;
  hlora.Init.WordLength   = UART_WORDLENGTH_8B;
  hlora.Init.StopBits     = UART_STOPBITS_1;
  hlora.Init.Parity       = UART_PARITY_NONE;
  hlora.Init.Mode         = UART_MODE_TX_RX;
  hlora.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
  hlora.Init.OverSampling = UART_OVERSAMPLING_16;

  if (HAL_UART_Init(&hlora) != HAL_OK)
  {
    Error_Handler();
  }
}

void LoRa_Init(void)
{
  LoRa_GPIO_Init();
  LoRa_UART_Init(LORA_CFG_BAUD);   /* factory default is 9600 8N1 in both modes */
  HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);
}

/* -------------------------------------------------------------------------- */
/* Mode control                                                               */
/* -------------------------------------------------------------------------- */

/* AUX 가 실제로 움직이는 것을 한 번이라도 본 적이 있는가(=배선 확인). */
static bool lora_aux_seen_low = false;

bool LoRa_AuxDetected(void)
{
  return lora_aux_seen_low;
}

bool LoRa_WaitAuxIdle(uint32_t timeout_ms)
{
#if LORA_AUX_ENABLE
  uint32_t start = HAL_GetTick();
  uint32_t high_since = 0;

  for (;;)
  {
    if (HAL_GPIO_ReadPin(LORA_AUX_GPIO_Port, LORA_AUX_Pin) == GPIO_PIN_SET)
    {
      /* 데이터시트상 AUX 가 올라간 뒤에도 잠깐 여유가 필요하다. HIGH 가
       * 유지되는지 확인해서 글리치를 idle 로 오인하지 않는다. */
      if (high_since == 0) high_since = HAL_GetTick();
      if ((HAL_GetTick() - high_since) >= 2u) return true;
    }
    else
    {
      lora_aux_seen_low = true;   /* 배선 살아 있음 */
      high_since = 0;
    }

    if ((HAL_GetTick() - start) >= timeout_ms) return false;
  }
#else
  (void)timeout_ms;
  return true;
#endif
}

void LoRa_SetMode(LoRa_Mode_t mode)
{
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin,
                    (mode & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin,
                    (mode & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);

  /* 모드 전환 자체에 필요한 최소 시간은 어차피 기다려야 한다. 그 뒤에
   * AUX 로 실제 idle 을 한 번 더 확인한다(미배선이면 즉시 통과). */
  HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);
  LoRa_WaitAuxIdle(LORA_AUX_TIMEOUT_MS);
  LoRa_FlushRx();
}

void LoRa_FlushRx(void)
{
  uint8_t dump;
  while (HAL_UART_Receive(&hlora, &dump, 1, 2) == HAL_OK) { /* drain */ }
  __HAL_UART_CLEAR_OREFLAG(&hlora);
}

/* -------------------------------------------------------------------------- */
/* Config mode register access                                                */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef LoRa_ReadRegisters(uint8_t start, uint8_t len, uint8_t *out)
{
  uint8_t cmd[3] = { 0xC1, start, len };
  uint8_t hdr[3];

  LoRa_FlushRx();

  if (HAL_UART_Transmit(&hlora, cmd, sizeof(cmd), 200) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_UART_Receive(&hlora, hdr, sizeof(hdr), LORA_CFG_REPLY_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_TIMEOUT;   /* nothing came back: wiring / power / TXD-RXD swap */
  }
  if (hdr[0] == 0xFF)
  {
    return HAL_ERROR;     /* module answered FF FF FF = command rejected */
  }
  if (hdr[0] != 0xC1 || hdr[1] != start || hdr[2] != len)
  {
    return HAL_ERROR;
  }

  return HAL_UART_Receive(&hlora, out, len, LORA_CFG_REPLY_TIMEOUT_MS);
}

/**
 * Same as LoRa_WriteRegisters but with the volatile 0xC2 opcode, which applies
 * the settings without burning them into non-volatile storage.
 */
static HAL_StatusTypeDef LoRa_WriteRegistersOpcode(uint8_t opcode, uint8_t start,
                                                   uint8_t len, const uint8_t *data)
{
  uint8_t cmd[3 + LORA_REG_COUNT];
  uint8_t echo[3 + LORA_REG_COUNT];

  if (len > LORA_REG_COUNT) return HAL_ERROR;

  cmd[0] = opcode;
  cmd[1] = start;
  cmd[2] = len;
  memcpy(&cmd[3], data, len);

  LoRa_FlushRx();

  if (HAL_UART_Transmit(&hlora, cmd, 3 + len, 200) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_UART_Receive(&hlora, echo, 3 + len, LORA_CFG_REPLY_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  return (echo[0] == 0xC1) ? HAL_OK : HAL_ERROR;
}

HAL_StatusTypeDef LoRa_Configure(uint8_t channel, LoRa_Power_t power,
                                 bool lbt, bool persist)
{
  uint8_t regs[3];              /* REG1 (0x04), REG2 (0x05), REG3 (0x06) */
  HAL_StatusTypeDef st;
  uint32_t saved_baud = hlora.Init.BaudRate;

  /* 설정모드 UART 는 저장된 baud 와 무관하게 항상 9600 이다. 이 프로젝트는
   * 평상시 115200 으로 말하므로 내려주지 않으면 영영 무응답이다. */
  LoRa_SetBaud(LORA_CFG_BAUD);
  LoRa_SetMode(LORA_MODE_CONFIG);

  st = LoRa_ReadRegisters(LORA_REG_REG1, 3, regs);
  if (st != HAL_OK)
  {
    LoRa_SetMode(LORA_MODE_NORMAL);
    LoRa_SetBaud(saved_baud);
    return st;
  }

  regs[0] = (uint8_t)((regs[0] & ~0x03u) | ((uint8_t)power & 0x03u));  /* tx power */
  regs[1] = channel;                                                    /* channel  */
  regs[2] = (uint8_t)(lbt ? (regs[2] | 0x10u) : (regs[2] & ~0x10u));    /* LBT bit  */

  st = LoRa_WriteRegistersOpcode(persist ? 0xC0 : 0xC2, LORA_REG_REG1, 3, regs);

  LoRa_SetMode(LORA_MODE_NORMAL);
  LoRa_SetBaud(saved_baud);
  return st;
}

void LoRa_DetectVariant(void)
{
  uint8_t regs[LORA_REG_COUNT];
  bool e22_ok, e220_ok;
  uint32_t saved_baud = hlora.Init.BaudRate;

  /* 설정모드는 9600 고정. 이걸 빠뜨리면 두 조합 다 silent 로 나와서
   * "전원/배선을 볼 것" 이라고 오진한다 -- 멀쩡한 하드웨어를 뜯게 된다. */
  LoRa_SetBaud(LORA_CFG_BAUD);

  printf("\r\n--- variant probe (E22 vs E220 mode encoding) ---\r\n");
  printf("  같은 M1/M0 조합이 두 계열에서 다른 모드를 뜻한다.\r\n");
  printf("  E22 : M1=1 M0=0 = Config,  M1=1 M0=1 = DeepSleep\r\n");
  printf("  E220: M1=1 M0=0 = WOR RX,  M1=1 M0=1 = Config\r\n\r\n");

  /* E22 기준 설정모드 */
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_RESET);
  HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);
  LoRa_FlushRx();
  e22_ok = (LoRa_ReadRegisters(0, LORA_REG_COUNT, regs) == HAL_OK);
  printf("  M1=1 M0=0 : %s\r\n", e22_ok ? "REPLY" : "silent");
  if (e22_ok)
  {
    printf("    raw:");
    for (int i = 0; i < LORA_REG_COUNT; i++) printf(" %02X", regs[i]);
    printf("\r\n");
  }

  /* E220 기준 설정모드 */
  HAL_GPIO_WritePin(LORA_M1_GPIO_Port, LORA_M1_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LORA_M0_GPIO_Port, LORA_M0_Pin, GPIO_PIN_SET);
  HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);
  LoRa_FlushRx();
  e220_ok = (LoRa_ReadRegisters(0, LORA_REG_COUNT, regs) == HAL_OK);
  printf("  M1=1 M0=1 : %s\r\n", e220_ok ? "REPLY" : "silent");
  if (e220_ok)
  {
    printf("    raw:");
    for (int i = 0; i < LORA_REG_COUNT; i++) printf(" %02X", regs[i]);
    printf("\r\n");
  }

  printf("  => ");
  if (e22_ok && !e220_ok)
  {
    printf("E22. 현재 드라이버 인코딩이 맞다.\r\n");
  }
  else if (!e22_ok && e220_ok)
  {
    printf("E220! 모드 인코딩이 반대다 -- 드라이버를 고쳐야 한다.\r\n");
  }
  else if (e22_ok && e220_ok)
  {
    printf("둘 다 응답. M0 가 모듈에 안 닿아 무시되는 중일 수 있다.\r\n");
  }
  else
  {
    printf("둘 다 무응답. 모드 문제가 아니다 -- 전원/배선을 볼 것.\r\n");
  }

  LoRa_SetMode(LORA_MODE_NORMAL);
  LoRa_SetBaud(saved_baud);
}

void LoRa_DumpRegisters(void)
{
  static const char *names[9] = { "ADDH", "ADDL", "NETID", "REG0", "REG1",
                                  "REG2", "REG3", "CRYPT_H", "CRYPT_L" };
  uint8_t regs[9];
  uint32_t saved_baud = hlora.Init.BaudRate;
  HAL_StatusTypeDef st;

  LoRa_SetBaud(LORA_CFG_BAUD);
  LoRa_SetMode(LORA_MODE_CONFIG);
  st = LoRa_ReadRegisters(0x00, 9, regs);
  LoRa_SetMode(LORA_MODE_NORMAL);
  LoRa_SetBaud(saved_baud);

  if (st != HAL_OK)
  {
    printf("# LORA REGS: 읽기 실패 (%d)\r\n", (int)st);
    return;
  }

  printf("# LORA REGS:");
  for (int i = 0; i < 9; i++) printf(" %02X", regs[i]);
  printf("\r\n#   ");
  for (int i = 0; i < 9; i++) printf("%s=%02X ", names[i], regs[i]);
  printf("\r\n#   기대값(이 보드): %02X %02X 00 E7 xx 1E 5x 00 00  (REG3 bit6=1 고정점)\r\n",
         (unsigned)((LORA_MY_ADDRESS >> 8) & 0xFFu),
         (unsigned)( LORA_MY_ADDRESS       & 0xFFu));
  printf("#   Pi 모듈 기대값: 00 10 00 E7 03 1E 53 00 00\r\n");
}

void LoRa_CheckNormalMode(void)
{
  uint8_t cmd[3] = { 0xC1, 0x00, 0x09 };
  uint8_t reply[12];
  HAL_StatusTypeDef st;

  printf("\r\n--- normal-mode check (M1 배선 확인) ---\r\n");
  printf("  일반 모드로 내린 뒤 설정 명령을 보낸다.\r\n");
  printf("  정상이면 이 바이트는 전파로 나가고 UART 응답이 없어야 한다.\r\n");

  LoRa_SetMode(LORA_MODE_NORMAL);          /* M0=0, M1=0 을 의도 */
  HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);

  /* 설정모드 UART 는 저장값과 무관하게 항상 9600. 모듈이 설정 모드에 갇혀
   * 있다면 이 속도로 응답한다. */
  LoRa_SetBaud(LORA_CFG_BAUD);
  LoRa_FlushRx();

  (void)HAL_UART_Transmit(&hlora, cmd, sizeof(cmd), 200);
  st = HAL_UART_Receive(&hlora, reply, sizeof(reply), LORA_CFG_REPLY_TIMEOUT_MS);

  if ((st == HAL_OK) && (reply[0] == 0xC1))
  {
    printf("  응답 옴:");
    for (int i = 0; i < (int)sizeof(reply); i++) printf(" %02X", reply[i]);
    printf("\r\n");
    printf("  => M1 이 모듈에 닿지 않는다. 일반 모드로 못 내려가고 설정 모드에\r\n");
    printf("     갇혀 있다. 그래서 LoRa_Send() 가 보낸 바이트는 전파가 아니라\r\n");
    printf("     설정 명령으로 해석돼 버려진다 -- 송신도 수신도 안 되는 이유.\r\n");
    printf("     확인: Nucleo D10(PB6) <-> 모듈 6번(M1) 배선/납땜\r\n");
  }
  else
  {
    printf("  무응답 -> 정상. 모듈은 일반 모드에 있다. 원인은 다른 곳.\r\n");
  }

  LoRa_SetBaud(LoRa_BaudCodeToRate(LORA_UART_PROJECT));
}

HAL_StatusTypeDef LoRa_BurnModule(uint16_t address, uint8_t netid)
{
  /* ADDH(0x00) ADDL(0x01) NETID(0x02) REG0(0x03) REG1(0x04) REG2(0x05) REG3(0x06) */
  uint8_t regs[9];
  uint8_t back[7];   /* CRYPT 는 write-only 라 되읽기 대조에서 제외 */
  HAL_StatusTypeDef st;

  /* 설정모드 UART 는 저장값과 무관하게 항상 9600 */
  LoRa_SetBaud(LORA_CFG_BAUD);
  LoRa_SetMode(LORA_MODE_CONFIG);

  /* REG1/REG3 은 우리가 관리하지 않는 비트(서브패킷 크기, RSSI, WOR 주기 등)가
   * 섞여 있으므로 현재값을 읽어 필요한 비트만 바꾼다. */
  st = LoRa_ReadRegisters(LORA_REG_ADDH, 7, regs);
  if (st != HAL_OK)
  {
    printf("[burn] 설정모드 진입 실패 -- 모듈이 응답하지 않음\r\n");
    LoRa_SetMode(LORA_MODE_NORMAL);
    return st;
  }

  regs[0] = (uint8_t)((address >> 8) & 0xFFu);                       /* ADDH  */
  regs[1] = (uint8_t)( address       & 0xFFu);                       /* ADDL  */
  regs[2] = netid;                                                   /* NETID */
  regs[3] = (uint8_t)(((uint8_t)LORA_UART_PROJECT & 0x07u) << 5)     /* REG0  */
          | (uint8_t)(0x00u)                                         /* 8N1   */
          | (uint8_t)((uint8_t)LORA_AIR_PROJECT & 0x07u);
  regs[4] = (uint8_t)((regs[4] & ~0x03u) | ((uint8_t)LORA_PWR_10DBM & 0x03u));
  regs[5] = LORA_CH_DATA_DEFAULT;                                    /* REG2  */
  regs[6] = (uint8_t)(regs[6] | 0x10u);                              /* LBT on */

  /* CRYPT_H/CRYPT_L (0x07/0x08) -- 무선 데이터 암호화 키.
   * 키가 다르면 전파는 나가는데 상대가 복호화를 못 해 아무것도 안 잡힌다.
   * !! 이 레지스터는 write-only 라 읽으면 항상 0x00 이 돌아온다. 즉 "읽어서
   * 같은지 확인"이 원리적으로 불가능하다. 그래서 모든 모듈에 같은 값을
   * 명시적으로 써 넣는 것 말고는 맞출 방법이 없다. */
  regs[7] = (uint8_t)((LORA_CRYPT_KEY >> 8) & 0xFFu);
  regs[8] = (uint8_t)( LORA_CRYPT_KEY       & 0xFFu);

  /* C0 = 영구 저장. 평상시 경로의 C2(휘발성)와 다른 점이 이것 하나다. */
  st = LoRa_WriteRegistersOpcode(0xC0, LORA_REG_ADDH, 9, regs);
  if (st != HAL_OK)
  {
    printf("[burn] 쓰기 실패\r\n");
    LoRa_SetMode(LORA_MODE_NORMAL);
    return st;
  }

  /* 쓰기 직후는 모듈이 처리 중일 수 있다. AUX 를 안 쓰므로 고정 딜레이. */
  HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);
  LoRa_FlushRx();

  st = LoRa_ReadRegisters(LORA_REG_ADDH, 7, back);
  if (st != HAL_OK)
  {
    printf("[burn] 되읽기 실패\r\n");
    LoRa_SetMode(LORA_MODE_NORMAL);
    return st;
  }

  for (int i = 0; i < 7; i++)
  {
    if (back[i] != regs[i])
    {
      printf("[burn] 대조 불일치: reg 0x%02X 쓴값 %02X 읽은값 %02X\r\n",
             i, regs[i], back[i]);
      LoRa_SetMode(LORA_MODE_NORMAL);
      return HAL_ERROR;
    }
  }

  lora_configured = true;
  LoRa_SetMode(LORA_MODE_NORMAL);
  LoRa_SetBaud(LoRa_BaudCodeToRate(LORA_UART_PROJECT));

  return HAL_OK;
}

uint32_t LoRa_BaudCodeToRate(LoRa_UartBaud_t code)
{
  static const uint32_t table[8] = {
    1200u, 2400u, 4800u, 9600u, 19200u, 38400u, 57600u, 115200u
  };
  return table[(uint8_t)code & 0x07u];
}

HAL_StatusTypeDef LoRa_ConfigureFull(uint8_t channel, LoRa_Power_t power, bool lbt,
                                     LoRa_AirRate_t air, LoRa_UartBaud_t uart,
                                     bool persist)
{
  /* ADDH(0x00) ADDL(0x01) NETID(0x02) REG0(0x03) REG1(0x04) REG2(0x05)
   * REG3(0x06) CRYPT_H(0x07) CRYPT_L(0x08) -- 9개를 한 번에 쓴다.
   * 주소까지 매 부팅 써 넣으므로 모듈을 바꿔 껴도 이 보드는 늘 같은 주소다. */
  uint8_t regs[9];
  HAL_StatusTypeDef st;

  /* 설정모드는 저장된 baud 와 무관하게 항상 9600 이다. 직전에 115200 으로
   * 올려놨더라도 여기서는 9600 으로 내려야 응답을 받는다. */
  LoRa_SetBaud(LORA_CFG_BAUD);
  LoRa_SetMode(LORA_MODE_CONFIG);

  /* 읽어서 우리가 안 건드리는 비트(서브패킷 크기, WOR 주기 등)는 보존한다. */
  st = LoRa_ReadRegisters(LORA_REG_ADDH, 7, regs);
  if (st != HAL_OK)
  {
    LoRa_SetMode(LORA_MODE_NORMAL);
    return st;
  }

  regs[0] = (uint8_t)((LORA_MY_ADDRESS >> 8) & 0xFFu);  /* ADDH */
  regs[1] = (uint8_t)( LORA_MY_ADDRESS       & 0xFFu);  /* ADDL */
  /* regs[2] (NETID) 는 읽은 값 그대로 둔다 -- 굽기 때 정해진다. */

  /* REG0: [7:5] UART baud | [4:3] parity(00=8N1) | [2:0] air data rate */
  regs[3] = (uint8_t)(((uint8_t)uart & 0x07u) << 5)
          | (uint8_t)(0x00u)
          | (uint8_t)((uint8_t)air & 0x07u);

  regs[4] = (uint8_t)((regs[4] & ~0x03u) | ((uint8_t)power & 0x03u)); /* tx power */
  regs[5] = channel;                                                  /* channel  */
  regs[6] = (uint8_t)(lbt ? (regs[6] | 0x10u) : (regs[6] & ~0x10u));  /* LBT bit  */
  /* REG3 bit6 = 1 -> 고정점 전송. 송신 데이터 앞에 <ADDH ADDL CH> 를 붙이면
   * 모듈이 그 3 byte 를 떼어내고 지정한 주소로 보낸다. 투명 모드와 달리
   * 양쪽 주소가 달라도 되고, 그래야 STM1/STM2 를 구분할 수 있다. */
  regs[6] = (uint8_t)(regs[6] | 0x40u);

  /* CRYPT_H/CRYPT_L (0x07/0x08) -- 무선 데이터 암호화 키.
   * 키가 다르면 전파는 정상적으로 나가는데 상대가 복호화를 못 해서
   * 수신단에 아무것도 안 잡힌다. 링크 죽음과 구분이 안 되는 증상이다.
   *
   * !! write-only 라 되읽으면 항상 0x00 이 돌아온다. 즉 "맞는지 확인"이
   *    불가능하므로 양쪽 다 매번 명시적으로 써 넣는 수밖에 없다.
   *    그래서 아래 되읽기 대조에서는 이 두 byte 를 제외한다. */
  regs[7] = (uint8_t)((LORA_CRYPT_KEY >> 8) & 0xFFu);
  regs[8] = (uint8_t)( LORA_CRYPT_KEY       & 0xFFu);

  st = LoRa_WriteRegistersOpcode(persist ? 0xC0 : 0xC2, LORA_REG_ADDH, 9, regs);

  /* 전파법 인터록: 에코만 믿지 않고 되읽어 대조한다. 여기서 어긋난 채로
   * 송신하면 모듈이 공장 기본값(채널 2 = RFID 전용, 22dBm)으로 쏘게 되고
   * 그건 명백한 위법이다. 확인되지 않으면 송신 자체를 막는다. */
  if (st == HAL_OK)
  {
    /* CRYPT(0x07/0x08)는 write-only 라 항상 00 으로 읽힌다 -- 대조에서 제외. */
    uint8_t back[7];
    /* 쓰기 직후 바로 읽으면 모듈이 아직 처리 중일 수 있다. AUX 를 안 쓰는
     * 구성이라 상태를 물어볼 수 없으므로 고정 딜레이로 넘긴다. */
    HAL_Delay(LORA_MODE_SWITCH_DELAY_MS);
    LoRa_FlushRx();
    if (LoRa_ReadRegisters(LORA_REG_ADDH, 7, back) != HAL_OK)
    {
      st = HAL_ERROR;
    }
    else if (memcmp(back, regs, 7) != 0)
    {
      st = HAL_ERROR;
    }
  }

  lora_configured = (st == HAL_OK);

  LoRa_SetMode(LORA_MODE_NORMAL);

  /* 일반 모드부터는 방금 저장한 baud 로 말한다. 쓰기가 실패했으면 모듈은
   * 옛 baud 그대로이므로 로컬도 9600 에 남겨둔다. */
  if (st == HAL_OK)
  {
    LoRa_SetBaud(LoRa_BaudCodeToRate(uart));
  }

  return st;
}

bool LoRa_IsConfigured(void)
{
  return lora_configured;
}

HAL_StatusTypeDef LoRa_WriteRegisters(uint8_t start, uint8_t len, const uint8_t *data)
{
  uint8_t cmd[3 + LORA_REG_COUNT];
  uint8_t echo[3 + LORA_REG_COUNT];

  if (len > LORA_REG_COUNT) return HAL_ERROR;

  cmd[0] = 0xC0;
  cmd[1] = start;
  cmd[2] = len;
  memcpy(&cmd[3], data, len);

  LoRa_FlushRx();

  if (HAL_UART_Transmit(&hlora, cmd, 3 + len, 200) != HAL_OK)
  {
    return HAL_ERROR;
  }
  if (HAL_UART_Receive(&hlora, echo, 3 + len, LORA_CFG_REPLY_TIMEOUT_MS) != HAL_OK)
  {
    return HAL_TIMEOUT;
  }

  return (echo[0] == 0xC1) ? HAL_OK : HAL_ERROR;
}

/* -------------------------------------------------------------------------- */
/* Normal mode data path                                                      */
/* -------------------------------------------------------------------------- */

HAL_StatusTypeDef LoRa_SendTo(uint16_t addr, uint8_t channel,
                              const uint8_t *buf, uint16_t len)
{
  /* 헤더와 payload 를 반드시 한 번의 전송으로 내보내야 한다.
   *
   * 처음엔 HAL_UART_Transmit 을 두 번(헤더 3 byte, 그 다음 payload) 불렀는데
   * 그러면 두 호출 사이의 틈에서 모듈이 앞 3 byte 를 완결된 패킷으로 끊어
   * 버릴 수 있다. 그러면 이어지는 payload 의 첫 3 byte(AA 55 01)가 다음
   * 패킷의 <주소, 채널> 로 해석돼 0xAA55 / 채널 1 로 나간다 -- 아무도 못
   * 받고, 송신 측에는 아무 오류도 안 남는다. 실제로 그 증상을 겪었다. */
  static uint8_t txbuf[240];
  HAL_StatusTypeDef st;

  /* E22 default sub-packet is 240 bytes; anything longer is split by the module
     and needs extra inter-packet spacing we cannot observe without AUX.
     고정점 헤더 3 byte 도 그 240 안에 포함된다.                            */
  if (len == 0 || len > (sizeof(txbuf) - 3u)) return HAL_ERROR;

  /* 고정점 전송 헤더. 모듈이 이 3 byte 를 떼어내고 목적지로 보내므로
   * 상대 UART 에는 payload 만 나온다. */
  txbuf[0] = (uint8_t)((addr >> 8) & 0xFFu);
  txbuf[1] = (uint8_t)( addr       & 0xFFu);
  txbuf[2] = channel;
  memcpy(&txbuf[3], buf, len);

#if LORA_DEBUG_HEX
  /* 모듈에 실제로 밀어 넣는 바이트를 그대로 찍는다. 고정점 헤더가 붙어
   * 나가는지, 붙는다면 목적지가 맞는지를 눈으로 확인하는 유일한 방법이다. */
  /* 시각이 있어야 4개가 한꺼번에 몰려 나가는지 고르게 나가는지 알 수 있다.
   * 이걸 빼놓고 로그만 보면 두 경우가 똑같이 보여서 원인을 못 가른다. */
  printf("# [%8lu] LORA TX->%02X%02X ch%u:", (unsigned long)HAL_GetTick(),
         txbuf[0], txbuf[1], (unsigned)txbuf[2]);
  for (uint16_t i = 0; i < len + 3u && i < 12u; i++) printf(" %02X", txbuf[i]);
  printf(" ...(%u B) aux=%s busy=%lu/%lums\r\n", (unsigned)(len + 3u),
         LoRa_AuxDetected() ? "ok" : "미배선?",
         (unsigned long)lora_last_tx_busy_ms, (unsigned long)lora_max_tx_busy_ms);
#endif

  /* 연속 송신 사이 최소 간격.
   *
   * 홀 4채널은 한 묶음으로 연달아 나간다. 그런데 모듈이 앞 프레임을 아직
   * 공중으로 내보내는 중에 다음 프레임을 UART 로 밀어 넣으면, 버스트의
   * 앞쪽 프레임이 통째로 사라지거나 페이로드 중간에서 잘려 나간다.
   * 실측에서 4개 중 1·2번째는 유실, 4번째는 CRC 실패로 나왔다.
   * HAL_UART_Transmit 은 UART 로 넘겼다는 뜻일 뿐이라 매번 성공을 돌려주고,
   * AUX 를 안 보고 있으니 STM32 는 이 상황을 알 방법이 없다.
   *
   * 정공법은 AUX 핀(이미 납땜돼 있다)을 읽어 모듈이 idle 로 돌아올 때까지
   * 기다리는 것이다. 그전까지는 공중 점유 시간 추정치에 여유를 얹어 재운다. */
  /* 1) 직전 프레임이 완전히 나갈 때까지 기다린다.
   *    AUX 가 배선돼 있으면 실제 idle 을 보고, 아니면 추정 딜레이로 떨어진다. */
  if (!LoRa_WaitAuxIdle(LORA_AUX_TIMEOUT_MS))
  {
    /* idle 로 안 돌아왔다. 그래도 보낸다 -- 여기서 포기하면 센서 이벤트가
     * 통째로 사라지고, 그건 늦게 도착하는 것보다 나쁘다. */
  }
  {
    uint32_t now = HAL_GetTick();
    if ((int32_t)(lora_tx_free_tick - now) > 0)
    {
      HAL_Delay(lora_tx_free_tick - now);
    }
  }

  /* 프레임 하나는 UART 로 끊김 없이 나가야 한다.
   *
   * HAL_UART_Transmit 은 블로킹 폴링이라 전송 도중 태스크 전환에 선점당한다.
   * 1ms 라도 스트림이 멈추면 E22 는 그 공백을 "데이터 끝" 으로 보고 그때까지
   * 받은 바이트만 패킷으로 만들어 쏜다. 실측에서 매 패킷이 정확히 13 byte
   * (= 115200 에서 약 1.13ms, FreeRTOS 틱 하나) 에서 잘렸다.
   *
   * 그래서 이 구간만 태스크 전환을 막는다. 38 byte 면 3.3ms 라 스케줄러에
   * 주는 부담이 크지 않다. 인터럽트는 계속 살아 있으므로 ISR 지연도 없다.
   * 근본적으로는 DMA 송신으로 가는 게 맞지만, 그건 별도 작업이다. */
  {
    bool sched_running = (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING);
    if (sched_running) vTaskSuspendAll();
    st = HAL_UART_Transmit(&hlora, txbuf, (uint16_t)(len + 3u), 1000);
    if (sched_running) (void)xTaskResumeAll();
  }

  /* 2) 모듈이 실제로 다 쏠 때까지 기다린 뒤 반환한다. 이게 없으면 호출자가
   *    곧바로 다음 프레임을 넣어 버려서, 버스트의 앞쪽이 잘리거나 사라진다.
   *
   *    걸린 시간도 잰다. LBT 는 채널이 바쁘면 송신을 미루거나 포기하면서
   *    그 사실을 알려주지 않는데, 그러면 AUX 가 예상 공중 점유보다 훨씬
   *    오래 LOW 로 남는다. 즉 이 값이 LBT 가 일했다는 유일한 증거다. */
  {
    uint32_t t0 = HAL_GetTick();
    LoRa_WaitAuxIdle(LORA_AUX_TIMEOUT_MS);
    lora_last_tx_busy_ms = HAL_GetTick() - t0;
    if (lora_last_tx_busy_ms > lora_max_tx_busy_ms)
    {
      lora_max_tx_busy_ms = lora_last_tx_busy_ms;
    }
  }

  /* AUX 가 없는 경우를 대비한 추정 딜레이. AUX 가 살아 있으면 위에서 이미
   * 기다렸으므로 이 시각은 대부분 이미 지나 있어 아무 비용도 없다. */
  lora_tx_free_tick = HAL_GetTick()
                    + (((uint32_t)(len + 3u) * 8u * 1000u) / LORA_AIRTIME_BPS)
                    + LORA_TX_GAP_MS;
  return st;
}

HAL_StatusTypeDef LoRa_Send(const uint8_t *buf, uint16_t len)
{
  return LoRa_SendTo(LORA_PEER_ADDRESS, LORA_CH_DATA_DEFAULT, buf, len);
}

/* -------------------------------------------------------------------------- */
/* 인터럽트 수신                                                              */
/* -------------------------------------------------------------------------- */

static uint8_t  lora_rx_ring[LORA_RX_RING_SIZE];
static volatile uint16_t lora_rx_head = 0;   /* ISR 이 쓴다 */
static volatile uint16_t lora_rx_tail = 0;   /* 태스크가 쓴다 */
static volatile uint32_t lora_rx_overrun = 0;
static uint8_t  lora_rx_byte;                /* HAL 이 채우는 1바이트 */

void LoRa_StartReceiveIT(void)
{
  /* USART1 은 드라이버가 직접 초기화하므로 NVIC 도 여기서 켠다.
   * 우선순위는 FreeRTOS API 를 부를 수 있는 범위(configMAX_SYSCALL...) 밖으로
   * 올리면 안 된다 -- 이 ISR 은 세마포어를 건드리지 않지만, 나중에 누가
   * 추가할 수 있으므로 다른 ISR 들과 같은 대역에 둔다. */
  HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);
  HAL_UART_Receive_IT(&hlora, &lora_rx_byte, 1);
}

void LoRa_OnByteReceived(void)
{
  uint8_t b = lora_rx_byte;

  /* 다음 바이트를 놓치지 않도록 먼저 재무장한다. */
  HAL_UART_Receive_IT(&hlora, &lora_rx_byte, 1);

  uint16_t next = (uint16_t)((lora_rx_head + 1u) % LORA_RX_RING_SIZE);
  if (next == lora_rx_tail)
  {
    /* 태스크가 못 따라왔다. 새 바이트를 버린다 -- 오래된 것을 밀어내면
     * 프레임 중간이 잘려 더 나쁘다. 이 카운터가 0 이 아니면 링버퍼를
     * 키우거나 소비 주기를 줄여야 한다. */
    lora_rx_overrun++;
    return;
  }
  lora_rx_ring[lora_rx_head] = b;
  lora_rx_head = next;
}

bool LoRa_RingPop(uint8_t *out)
{
  if (lora_rx_tail == lora_rx_head) return false;
  *out = lora_rx_ring[lora_rx_tail];
  lora_rx_tail = (uint16_t)((lora_rx_tail + 1u) % LORA_RX_RING_SIZE);
  return true;
}

uint32_t LoRa_GetRxOverrunCount(void)
{
  return lora_rx_overrun;
}

static uint32_t lora_rx_rearm_count = 0;

uint32_t LoRa_GetRxRearmCount(void)
{
  return lora_rx_rearm_count;
}

bool LoRa_EnsureReceiving(void)
{
  /* 인터럽트 수신은 바이트마다 다시 걸어야 하는데, 그 재무장이 한 번이라도
   * HAL_BUSY 를 돌려받으면 그 순간부터 영영 듣지 않는다. 송신과 타이밍이
   * 겹치거나 오버런 처리 중이면 실제로 일어난다. 게다가 조용히 죽어서
   * 로그조차 안 남는다 -- 실측에서 "첫 프레임만 받고 그 뒤로 무응답" 으로
   * 나타났다.
   *
   * 그래서 원인을 하나씩 막는 대신, 주기적으로 상태를 확인해서 꺼져 있으면
   * 다시 건다. 정상일 때는 아무 비용도 없다. */
  if (hlora.RxState == HAL_UART_STATE_BUSY_RX)
  {
    return false;
  }

  __HAL_UART_CLEAR_OREFLAG(&hlora);
  if (HAL_UART_Receive_IT(&hlora, &lora_rx_byte, 1) == HAL_OK)
  {
    lora_rx_rearm_count++;
    return true;
  }
  return false;
}

uint16_t LoRa_Recv(uint8_t *buf, uint16_t maxlen, uint32_t timeout_ms)
{
  uint16_t n = 0;
  uint32_t start = HAL_GetTick();

  /* Wait for the first byte, then keep reading until a short inter-byte gap
     marks the end of the frame. */
  while ((HAL_GetTick() - start) < timeout_ms && n < maxlen)
  {
    if (HAL_UART_Receive(&hlora, &buf[n], 1, 20) == HAL_OK)
    {
      n++;
      /* frame body: 20ms of silence ends it */
      while (n < maxlen && HAL_UART_Receive(&hlora, &buf[n], 1, 20) == HAL_OK)
      {
        n++;
      }
      break;
    }
  }

  __HAL_UART_CLEAR_OREFLAG(&hlora);
  return n;
}

/* -------------------------------------------------------------------------- */
/* Bring-up self test                                                         */
/* -------------------------------------------------------------------------- */

/* Decode tables from the E22-900T22S(1B) manual section 7.2. */
static const uint32_t uart_baud_tbl[8] = { 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
static const char *parity_tbl[4]   = { "8N1", "8O1", "8E1", "8N1" };
static const char *air_rate_tbl[8] = { "0.3k", "1.2k", "2.4k", "4.8k",
                                       "9.6k", "19.2k", "38.4k", "62.5k" };
static const uint16_t sub_pkt_tbl[4] = { 240, 128, 64, 32 };
static const char *tx_power_tbl[4] = { "22dBm", "17dBm", "13dBm", "10dBm" };

bool LoRa_SelfTest(void)
{
  uint8_t p[LORA_REG_COUNT];
  HAL_StatusTypeDef st;
  uint32_t f_khz;
  uint32_t saved_baud = hlora.Init.BaudRate;

  printf("\r\n=== E22-900T22S(1B) self test ===\r\n");

  /* 설정모드 UART 는 9600 고정이다. 평상시 115200 으로 말하고 있으므로
   * 내려주지 않으면 설정이 정상인 모듈도 [FAIL] 로 나온다. */
  LoRa_SetBaud(LORA_CFG_BAUD);
  LoRa_SetMode(LORA_MODE_CONFIG);
  st = LoRa_ReadRegisters(LORA_REG_ADDH, LORA_REG_COUNT, p);

  if (st != HAL_OK)
  {
    printf("[FAIL] no valid reply (%s)\r\n",
           (st == HAL_TIMEOUT) ? "timeout" : "bad frame");
    printf("  check: VCC 3.3V / GND / module TXD -> PA10 / module RXD -> PA9\r\n");
    printf("  check: config mode is M1=1 M0=0 on the E22 (M1=1 M0=1 is SLEEP)\r\n");
    LoRa_SetMode(LORA_MODE_NORMAL);
    LoRa_SetBaud(saved_baud);
    return false;
  }

  printf("[ OK ] raw:");
  for (int i = 0; i < LORA_REG_COUNT; i++) printf(" %02X", p[i]);
  printf("\r\n");

  f_khz = LORA_CH_FREQ_BASE_KHZ + (uint32_t)p[5] * LORA_CH_STEP_KHZ;

  printf("  address    : 0x%02X%02X\r\n", p[0], p[1]);
  printf("  net id     : 0x%02X\r\n", p[2]);
  printf("  uart       : %lu %s\r\n",
         (unsigned long)uart_baud_tbl[(p[3] >> 5) & 0x07],
         parity_tbl[(p[3] >> 3) & 0x03]);
  printf("  air rate   : %s\r\n", air_rate_tbl[p[3] & 0x07]);
  printf("  sub packet : %u bytes\r\n", sub_pkt_tbl[(p[4] >> 6) & 0x03]);
  printf("  tx power   : %s\r\n", tx_power_tbl[p[4] & 0x03]);
  printf("  channel    : %u  (%lu.%lu MHz)%s\r\n", p[5],
         (unsigned long)(f_khz / 1000u), (unsigned long)((f_khz % 1000u) / 100u),
         (p[5] >= LORA_CH_KC_MIN && p[5] <= LORA_CH_KC_MAX)
            ? "  [in KC band]" : "  [OUTSIDE KC band 1..32]");
  printf("  mode       : %s\r\n", (p[6] & 0x40) ? "fixed (addressed)" : "transparent");
  printf("  rssi byte  : %s\r\n", (p[6] & 0x80) ? "on" : "off");
  printf("  LBT        : %s\r\n", (p[6] & 0x10) ? "on" : "off");

  LoRa_SetMode(LORA_MODE_NORMAL);
  LoRa_SetBaud(saved_baud);
  printf("=== back to normal mode ===\r\n");
  return true;
}

/* -------------------------------------------------------------------------- */
/* Diagnostics                                                                */
/* -------------------------------------------------------------------------- */

void LoRa_SetBaud(uint32_t baud)
{
  HAL_UART_DeInit(&hlora);
  LoRa_UART_Init(baud);
}

bool LoRa_LoopbackTest(void)
{
  const uint8_t pattern[4] = { 0x55, 0xAA, 0x0F, 0xF0 };
  uint8_t got[4] = {0};
  bool ok;

  printf("\r\n--- USART1 loopback (jumper PA9 <-> PA10, module unplugged) ---\r\n");

  LoRa_FlushRx();
  HAL_UART_Transmit(&hlora, (uint8_t *)pattern, 4, 200);
  HAL_UART_Receive(&hlora, got, 4, 200);

  ok = (memcmp(pattern, got, 4) == 0);
  printf("  sent 55 AA 0F F0 / got %02X %02X %02X %02X -> %s\r\n",
         got[0], got[1], got[2], got[3], ok ? "PASS" : "FAIL");

  if (ok)
    printf("  USART1 is fine. The fault is the module, its power, or the wiring.\r\n");
  else
    printf("  USART1 or the PA9/PA10 jumper is the problem, not the module.\r\n");

  return ok;
}

/**
 * Read a pin as a plain input with the requested internal pull, then hand it
 * back to USART1. A pin tied to a powered push-pull output ignores the pull;
 * a pin connected to nothing (or to a high-impedance input) follows it.
 */
static int LoRa_ProbePin(uint16_t pin, uint32_t pull)
{
  GPIO_InitTypeDef g = {0};
  int level;

  g.Pin   = pin;
  g.Mode  = GPIO_MODE_INPUT;
  g.Pull  = pull;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &g);
  HAL_Delay(5);

  level = (int)HAL_GPIO_ReadPin(GPIOA, pin);

  /* restore USART1 alternate function */
  g.Mode      = GPIO_MODE_AF_PP;
  g.Pull      = GPIO_PULLUP;
  g.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &g);

  return level;
}

void LoRa_LineProbe(void)
{
  int rx_dn, rx_up, tx_dn;

  printf("\r\n--- line probe (is anything actually driving the pins?) ---\r\n");

  /* PA10 should be sitting on the module's TXD: a powered UART output idles
     HIGH and will overpower our internal pull-down. */
  rx_dn = LoRa_ProbePin(GPIO_PIN_10, GPIO_PULLDOWN);
  rx_up = LoRa_ProbePin(GPIO_PIN_10, GPIO_PULLUP);

  /* PA9 is our TX and should meet the module's RXD, a high-impedance input,
     so our pull-down must win. If it reads HIGH, PA9 is staring at an output
     instead -- i.e. TXD/RXD are wired straight through. */
  tx_dn = LoRa_ProbePin(GPIO_PIN_9, GPIO_PULLDOWN);

  printf("  PA10 (expects module TXD): pulldown=%d pullup=%d\r\n", rx_dn, rx_up);
  printf("  PA9  (expects module RXD): pulldown=%d\r\n", tx_dn);
  printf("  verdict: ");

  if (rx_dn == 1 && tx_dn == 1)
  {
    /* Both pins beat a ~40k pull-down. One of them is the module's TXD idling
       high; the other is its RXD held up by the module's internal pull-up.
       Which is which cannot be told apart from here -- swap the wires to find
       out. Note the sweep already proves the two pins are not bridged: a
       bridge would have echoed our own bytes straight back.               */
    printf("both lines held high by something resistive.\r\n");
    printf("           see the short test below for what that means.\r\n");
  }
  else if (tx_dn == 1 && rx_dn == 0)
  {
    printf("SWAPPED -- module TXD is on PA9. Cross the two wires.\r\n");
  }
  else if (rx_dn == 1)
  {
    printf("PA10 driven high = module powered, TXD connected, RXD likely open.\r\n");
  }
  else if (rx_up == 1)
  {
    printf("PA10 floats = no power at the module, or TXD joint is open.\r\n");
  }
  else
  {
    printf("PA10 stuck low = TXD shorted to GND (solder bridge).\r\n");
  }
}

/* -------------------------------------------------------------------------- */
/* Bit-banged UART on the swapped pins                                        */
/*                                                                            */
/* USART1 is hard-wired to PA9=TX / PA10=RX on this part, so the only way to  */
/* try the opposite orientation without touching a wire is to drive the pins  */
/* by hand. Timing comes from the DWT cycle counter.                          */
/* -------------------------------------------------------------------------- */

static void bb_enable_dwt(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL   |= DWT_CTRL_CYCCNTENA_Msk;
}

static void bb_wait_until(uint32_t deadline)
{
  while ((int32_t)(DWT->CYCCNT - deadline) < 0) { /* spin */ }
}

static void bb_tx_byte(uint16_t pin, uint8_t b, uint32_t bitcyc)
{
  uint32_t t = DWT->CYCCNT;

  GPIOA->BSRR = (uint32_t)pin << 16;          /* start bit: low */
  t += bitcyc; bb_wait_until(t);

  for (int i = 0; i < 8; i++)                 /* 8 data bits, LSB first */
  {
    if (b & 1u) GPIOA->BSRR = pin;
    else        GPIOA->BSRR = (uint32_t)pin << 16;
    b >>= 1;
    t += bitcyc; bb_wait_until(t);
  }

  GPIOA->BSRR = pin;                          /* stop bit: high */
  t += bitcyc; bb_wait_until(t);
}

static int bb_rx_byte(uint16_t pin, uint32_t bitcyc, uint32_t timeout_cyc)
{
  uint32_t start = DWT->CYCCNT;
  uint32_t t;
  uint8_t  v = 0;

  while (GPIOA->IDR & pin)                    /* wait for the falling edge */
  {
    if ((DWT->CYCCNT - start) > timeout_cyc) return -1;
  }

  t = DWT->CYCCNT + bitcyc + (bitcyc / 2);    /* sample the middle of bit 0 */

  for (int i = 0; i < 8; i++)
  {
    bb_wait_until(t);
    if (GPIOA->IDR & pin) v |= (uint8_t)(1u << i);
    t += bitcyc;
  }

  bb_wait_until(t);                           /* ride out the stop bit */
  return (int)v;
}

/**
 * Bit-bang a config query over one chosen pin pair, bypassing USART1 entirely.
 * @retval number of bytes received.
 */
static int bb_try(uint16_t tx_pin, uint16_t rx_pin, const char *label)
{
  const uint8_t cmd[3] = { 0xC1, 0x00, 0x09 };
  const uint32_t bitcyc = SystemCoreClock / LORA_CFG_BAUD;
  const uint32_t tmo    = SystemCoreClock / 5;      /* 200 ms */
  GPIO_InitTypeDef g = {0};
  uint8_t got[16];
  int n = 0, b;

  g.Pin = tx_pin; g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL; g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &g);
  GPIOA->BSRR = tx_pin;                       /* idle high */

  g.Pin = rx_pin; g.Mode = GPIO_MODE_INPUT; g.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &g);

  HAL_Delay(10);

  __disable_irq();
  for (int i = 0; i < 3; i++) bb_tx_byte(tx_pin, cmd[i], bitcyc);
  while (n < (int)sizeof(got) && (b = bb_rx_byte(rx_pin, bitcyc, tmo)) >= 0)
  {
    got[n++] = (uint8_t)b;
  }
  __enable_irq();

  printf("  %s ->", label);
  if (n == 0)
  {
    printf(" (nothing)\r\n");
  }
  else
  {
    for (int i = 0; i < n; i++) printf(" %02X", got[i]);
    printf("\r\n");
  }
  return n;
}

void LoRa_SwapTest(void)
{
  int normal, swapped;

  printf("\r\n--- bit-banged probe (USART1 bypassed entirely) ---\r\n");

  bb_enable_dwt();
  HAL_UART_DeInit(&hlora);

  normal  = bb_try(GPIO_PIN_9,  GPIO_PIN_10, "normal  (TX=PA9,  RX=PA10)");
  swapped = bb_try(GPIO_PIN_10, GPIO_PIN_9,  "swapped (TX=PA10, RX=PA9) ");

  if (swapped > 0)
    printf("  >>> WIRES ARE SWAPPED. Cross PA9 and PA10 at the module. <<<\r\n");
  else if (normal > 0)
    printf("  >>> Wiring is right but USART1 is at fault, not the module. <<<\r\n");
  else
    printf("  Both orientations silent with USART1 out of the picture:\r\n"
           "  the module is not answering at all. Only one module is needed\r\n"
           "  for this test -- no radio is involved -- so the fault is local:\r\n"
           "  no power at the module, M0/M1 not reaching it, or an open RXD.\r\n");

  /* hand the pins back to USART1 */
  LoRa_UART_Init(LORA_CFG_BAUD);
}

/* -------------------------------------------------------------------------- */
/* Short-to-supply test                                                       */
/*                                                                            */
/* Driving a pin HIGH and reading HIGH proves nothing. Driving it LOW does:   */
/* a healthy pin follows, while a pin bridged to a supply rail stays HIGH.    */
/* The pulse is kept to tens of microseconds -- the bit-banged probe already  */
/* drives these same pins low for a full 104us per start bit.                 */
/* -------------------------------------------------------------------------- */

static int LoRa_DriveLowReadback(GPIO_TypeDef *port, uint16_t pin)
{
  GPIO_InitTypeDef g = {0};
  int level;

  g.Pin   = pin;
  g.Mode  = GPIO_MODE_OUTPUT_PP;
  g.Pull  = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(port, &g);

  port->BSRR = (uint32_t)pin << 16;           /* drive low */
  for (volatile int i = 0; i < 300; i++) { }  /* settle, ~20us */
  level = (port->IDR & pin) ? 1 : 0;

  /* release immediately */
  g.Mode = GPIO_MODE_INPUT;
  g.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(port, &g);

  return level;
}

void LoRa_ShortTest(void)
{
  int a9, a10, b4, b5;

  printf("\r\n--- short-to-supply test (drive LOW, read back) ---\r\n");

  HAL_UART_DeInit(&hlora);

  a9  = LoRa_DriveLowReadback(GPIOA, GPIO_PIN_9);
  a10 = LoRa_DriveLowReadback(GPIOA, GPIO_PIN_10);
  b4  = LoRa_DriveLowReadback(LORA_M0_GPIO_Port, LORA_M0_Pin);
  b5  = LoRa_DriveLowReadback(LORA_M1_GPIO_Port, LORA_M1_Pin);

  printf("  PA9  (RXD) driven low -> reads %d %s\r\n", a9,  a9  ? "*** SHORTED ***" : "ok");
  printf("  PA10 (TXD) driven low -> reads %d %s\r\n", a10, a10 ? "*** SHORTED ***" : "ok");
  printf("  PC7  (M0)  driven low -> reads %d %s\r\n", b4,  b4  ? "*** SHORTED ***" : "ok");
  printf("  PB6  (M1)  driven low -> reads %d %s\r\n", b5,  b5  ? "*** SHORTED ***" : "ok");

  if (a9 || a10 || b4 || b5)
  {
    printf("  >>> A pin is bridged to a supply rail. Reflow that joint. <<<\r\n");
  }
  else
  {
    printf("  All four pins pull low cleanly: no solder bridge to VCC.\r\n");
    printf("  Note: lines that beat a 40k pull-down yet yield instantly to a\r\n");
    printf("  push-pull driver are held by RESISTIVE PULL-UPS, not by a live\r\n");
    printf("  output. A running module would drive TXD high and fight back.\r\n");
    printf("  => the module is most likely NOT powered up. Measure VCC-GND\r\n");
    printf("     at the module pads before changing anything else.\r\n");
  }

  /* restore everything */
  LoRa_GPIO_Init();
  LoRa_UART_Init(LORA_CFG_BAUD);
}

void LoRa_Diag(void)
{
  static const uint32_t sweep[] = { 9600, 115200, 57600, 38400, 19200, 4800, 2400, 1200 };
  uint8_t raw[16];

  printf("\r\n########## LoRa diagnosis ##########\r\n");

  LoRa_LineProbe();
  LoRa_ShortTest();

  /* 1) Are the mode pins actually being driven? Read back the output latch
        and the real pin level -- they differ if something is shorting it. */
  LoRa_SetMode(LORA_MODE_CONFIG);
  printf("[pins] M0(PC7) latch=%d pin=%d | M1(PB6) latch=%d pin=%d  (config = M1:1 M0:0)\r\n",
         (int)((LORA_M0_GPIO_Port->ODR & LORA_M0_Pin) ? 1 : 0),
         (int)HAL_GPIO_ReadPin(LORA_M0_GPIO_Port, LORA_M0_Pin),
         (int)((LORA_M1_GPIO_Port->ODR & LORA_M1_Pin) ? 1 : 0),
         (int)HAL_GPIO_ReadPin(LORA_M1_GPIO_Port, LORA_M1_Pin));

  /* 2) Sweep baud rates. Config mode should answer at 9600, but if the module
        is alive and merely stuck in another mode we may still see noise.     */
  printf("[sweep] sending C1 00 09 at each baud, dumping anything received\r\n");

  for (unsigned i = 0; i < sizeof(sweep) / sizeof(sweep[0]); i++)
  {
    uint8_t cmd[3] = { 0xC1, 0x00, 0x09 };
    uint16_t n = 0;

    LoRa_SetBaud(sweep[i]);
    HAL_Delay(20);
    LoRa_FlushRx();

    HAL_UART_Transmit(&hlora, cmd, 3, 200);

    while (n < sizeof(raw) &&
           HAL_UART_Receive(&hlora, &raw[n], 1, 150) == HAL_OK)
    {
      n++;
    }

    printf("  %6lu :", (unsigned long)sweep[i]);
    if (n == 0)
    {
      printf(" (nothing)");
    }
    else
    {
      for (uint16_t k = 0; k < n; k++) printf(" %02X", raw[k]);
    }
    printf("   err=0x%02lX\r\n", (unsigned long)hlora.ErrorCode);
    hlora.ErrorCode = HAL_UART_ERROR_NONE;
  }

  LoRa_SetBaud(LORA_CFG_BAUD);

  /* Still nothing on the normal orientation -- try it the other way round. */
  LoRa_SwapTest();

  LoRa_SetMode(LORA_MODE_NORMAL);
  printf("####################################\r\n");
}

/**
  ******************************************************************************
  * @file    lora_frame.c
  * @brief   Pi_Server LoRa binary frame 인코더 구현. 규격은 lora_frame.h 참고.
  ******************************************************************************
  */

#include "lora_frame.h"
#include "lora_e22.h"
#include <string.h>
#include <stdio.h>

static uint32_t lora_dropped_frames = 0;
static uint8_t  lora_tx_buf[LORA_FRAME_MAX_LEN];

/* duty cycle 예산: 60초 창에 쓴 공중 점유 시간을 누적한다. 창이 지나면 리셋. */
static uint32_t lora_window_start_tick = 0;
static uint32_t lora_airtime_used_ms   = 0;

uint32_t LoRaFrame_EstimateAirtimeMs(uint16_t frame_len)
{
  /* payload 시간 + 고정 오버헤드(preamble/헤더). 올림으로 안전하게. */
  uint32_t bits = (uint32_t)frame_len * 8u;
  uint32_t ms   = (bits * 1000u + (LORA_AIRTIME_BPS - 1u)) / LORA_AIRTIME_BPS;
  return ms + LORA_AIRTIME_OVERHEAD_MS;
}

uint32_t LoRaFrame_GetAirtimeUsedMs(void)
{
  return lora_airtime_used_ms;
}

/**
 * 이 프레임을 지금 보내도 duty 한도 안인지 본다.
 * 통과시키면 예산을 차감한다.
 */
static bool duty_budget_take(uint32_t airtime_ms)
{
  uint32_t now   = HAL_GetTick();
  uint32_t limit = (LORA_DUTY_BUDGET_MS * LORA_DUTY_SAFETY_PCT) / 100u;

  /* 창이 지났으면 새 창을 연다. HAL_GetTick() 랩어라운드도 뺄셈이라 안전. */
  if ((lora_window_start_tick == 0u) ||
      ((now - lora_window_start_tick) >= LORA_DUTY_WINDOW_MS))
  {
    lora_window_start_tick = now;
    lora_airtime_used_ms   = 0;
  }

  if ((lora_airtime_used_ms + airtime_ms) > limit)
  {
    return false;
  }

  lora_airtime_used_ms += airtime_ms;
  return true;
}

/* CRC-16/CCITT-FALSE. 테이블 없이 비트 단위로 돌린다 -- 프레임이 40 byte
 * 남짓이라 320회 반복이고, 1초에 몇 번 나가지도 않아 테이블을 둘 이유가 없다. */
uint16_t LoRaFrame_Crc16(const uint8_t *data, uint16_t len)
{
  uint16_t crc = 0xFFFFu;

  if (data == NULL)
  {
    return crc;
  }

  for (uint16_t i = 0; i < len; i++)
  {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t bit = 0; bit < 8; bit++)
    {
      if (crc & 0x8000u)
      {
        crc = (uint16_t)((crc << 1) ^ 0x1021u);
      }
      else
      {
        crc = (uint16_t)(crc << 1);
      }
    }
  }

  return crc;
}

uint16_t LoRaFrame_Encode(uint8_t *out, uint16_t out_size,
                          uint8_t msg_type, uint32_t seq,
                          const uint8_t *payload, uint16_t payload_len)
{
  if ((out == NULL) || (payload_len > LORA_FRAME_MAX_PAYLOAD))
  {
    return 0;
  }
  if ((payload == NULL) && (payload_len > 0))
  {
    return 0;
  }

  uint16_t total = (uint16_t)(LORA_FRAME_OVERHEAD + payload_len);
  if (out_size < total)
  {
    return 0;
  }

  out[0] = LORA_FRAME_SOF0;
  out[1] = LORA_FRAME_SOF1;
  out[2] = LORA_FRAME_VERSION;
  out[3] = msg_type;

  /* transport sequence, big-endian */
  out[4] = (uint8_t)((seq >> 24) & 0xFFu);
  out[5] = (uint8_t)((seq >> 16) & 0xFFu);
  out[6] = (uint8_t)((seq >>  8) & 0xFFu);
  out[7] = (uint8_t)( seq        & 0xFFu);

  /* payload length, big-endian */
  out[8] = (uint8_t)((payload_len >> 8) & 0xFFu);
  out[9] = (uint8_t)( payload_len       & 0xFFu);

  if (payload_len > 0)
  {
    memcpy(&out[LORA_FRAME_HEADER_LEN], payload, payload_len);
  }

  /* CRC 범위는 Version(offset 2)부터 payload 끝까지 -- SOF 2 byte는 제외 */
  uint16_t crc = LoRaFrame_Crc16(&out[2],
                                 (uint16_t)(LORA_FRAME_HEADER_LEN - 2 + payload_len));

  out[LORA_FRAME_HEADER_LEN + payload_len]     = (uint8_t)((crc >> 8) & 0xFFu);
  out[LORA_FRAME_HEADER_LEN + payload_len + 1] = (uint8_t)( crc       & 0xFFu);

  return total;
}

HAL_StatusTypeDef LoRaFrame_SendLine(uint8_t msg_type, uint32_t seq,
                                     const char *line, bool critical)
{
  if (line == NULL)
  {
    return HAL_ERROR;
  }

  /* 끝의 CR/LF 제거. UART 경로는 CRLF가 필요하지만 프레임엔 길이 필드가
   * 있어서 불필요하고, 공중 점유만 늘린다. */
  size_t len = strlen(line);
  while ((len > 0) && ((line[len - 1] == '\r') || (line[len - 1] == '\n')))
  {
    len--;
  }
  if (len > LORA_FRAME_MAX_PAYLOAD)
  {
    return HAL_ERROR;
  }

  /* 인코딩은 무조건 먼저 한다. 송신이 막히더라도 어떤 바이트가 나갈 뻔했는지
   * 보여야 브링업 때 프레임 포맷을 대조할 수 있다. */
  uint16_t total = LoRaFrame_Encode(lora_tx_buf, sizeof(lora_tx_buf),
                                    msg_type, seq,
                                    (const uint8_t *)line, (uint16_t)len);
  if (total == 0u)
  {
    return HAL_ERROR;
  }

  HAL_StatusTypeDef st;
  const char *why;
  uint32_t airtime = LoRaFrame_EstimateAirtimeMs(total);

  if (!LoRa_IsConfigured())
  {
    /* 전파법 인터록. 설정이 확인되지 않은 모듈은 공장 기본값(채널 2 = RFID
     * 리더 전용, 22 dBm)일 수 있으므로 송신 자체를 막는다. 화재도 예외 없다 --
     * 위법 송신을 하느니 안 보내는 게 맞다. */
    st  = HAL_ERROR;
    why = "NOCFG";
  }
  else if (!duty_budget_take(airtime))
  {
    /* duty cycle 예산 소진. critical 이라도 예산은 못 넘긴다 -- 한도를 넘긴
     * 송신은 화재 경보라도 위법이기 때문이다. 대신 예산 자체가 넉넉해서
     * (60초에 960ms) 정상 트래픽으로는 거의 닿지 않는다. 여기 걸린다면
     * 송신 주기 정책이 잘못된 것이므로 drop 카운터를 볼 것. */
    lora_dropped_frames++;
    st  = HAL_BUSY;
    why = "DROP ";
  }
  else
  {
    st = LoRa_Send(lora_tx_buf, total);
    if (st == HAL_OK)
    {
      why = "OK   ";
    }
    else
    {
      /* 실제로 안 나갔으니 예산을 돌려준다. */
      if (lora_airtime_used_ms >= airtime) lora_airtime_used_ms -= airtime;
      why = "TXERR";
    }
  }

  (void)critical;   /* 예산제로 바뀌면서 우회 경로가 없어졌다 */

#if LORA_DEBUG_HEX
  /* 어느 경로로 갔든 항상 찍는다 -- 안 나가는 이유를 봐야 진단이 된다. */
  printf("# LORA %s %uB air=%lums/%lums drop=%lu:", why, (unsigned)total,
         (unsigned long)lora_airtime_used_ms,
         (unsigned long)((LORA_DUTY_BUDGET_MS * LORA_DUTY_SAFETY_PCT) / 100u),
         (unsigned long)lora_dropped_frames);
  for (uint16_t i = 0; i < total; i++)
  {
    printf(" %02X", lora_tx_buf[i]);
  }
  printf("\r\n");
#else
  (void)why;
#endif

  return st;
}

uint32_t LoRaFrame_GetDroppedCount(void)
{
  return lora_dropped_frames;
}

/* -------------------------------------------------------------------------- */
/* 수신 디코더                                                                */
/* -------------------------------------------------------------------------- */

/* 링버퍼에서 꺼낸 바이트를 여기 쌓아 프레임 단위로 잘라낸다. 헤더가 다 와야
 * 길이를 알 수 있으므로 부분 프레임을 들고 있을 곳이 필요하다. */
static uint8_t  rx_asm[LORA_FRAME_MAX_LEN];
static uint16_t rx_len = 0;
static uint32_t lora_rx_crc_errors = 0;

uint32_t LoRaFrame_GetRxCrcErrorCount(void)
{
  return lora_rx_crc_errors;
}

/* 앞에서 n 바이트를 버리고 나머지를 당긴다. 동기를 다시 잡을 때 쓴다. */
static void rx_discard(uint16_t n)
{
  if (n >= rx_len) { rx_len = 0; return; }
  memmove(rx_asm, &rx_asm[n], (size_t)(rx_len - n));
  rx_len = (uint16_t)(rx_len - n);
}

uint16_t LoRaFrame_Poll(char *out, uint16_t out_size,
                        uint8_t *msg_type, uint32_t *seq)
{
  uint8_t b;

  /* 링버퍼를 비우면서 조립 버퍼에 채운다. */
  while (LoRa_RingPop(&b))
  {
    if (rx_len < sizeof(rx_asm))
    {
      rx_asm[rx_len++] = b;
    }
    else
    {
      /* 조립 버퍼가 꽉 찼는데 프레임이 안 나왔다 = 동기를 잃었다.
       * 통째로 버리는 대신 앞을 밀어 다음 SOF 를 찾을 여지를 남긴다. */
      rx_discard(1);
      rx_asm[rx_len++] = b;
    }
  }

  for (;;)
  {
    /* SOF 찾기 */
    uint16_t i = 0;
    while ((i + 1u) < rx_len &&
           !(rx_asm[i] == LORA_FRAME_SOF0 && rx_asm[i + 1u] == LORA_FRAME_SOF1))
    {
      i++;
    }
    if (i > 0) rx_discard(i);

    if (rx_len < LORA_FRAME_HEADER_LEN) return 0;   /* 헤더가 아직 덜 왔다 */
    if (!(rx_asm[0] == LORA_FRAME_SOF0 && rx_asm[1] == LORA_FRAME_SOF1)) return 0;

    /* 헤더 타당성 검사. 여기서 거르지 않으면 쓰레기에서 우연히 나온 AA 55 로
     * 엉뚱한 길이를 읽어 한참을 기다리게 된다. */
    uint16_t plen = (uint16_t)((rx_asm[8] << 8) | rx_asm[9]);
    /* 받을 때는 version 0x01/0x02 를 다 받는다.
     *
     * 보낼 때만 0x02 를 쓰고 받을 때는 넓게 받아야, STM 과 Pi 를 같은 순간에
     * 배포하지 않아도 된다. Pi 의 LoRaDriver 는 지금 encode() 에서 0x01 을
     * 찍으므로, 여기서 0x02 만 고집하면 하행이 통째로 막힌다.
     * ALERT 명령 페이로드 문법은 v1.0/v1.1 사이에 바뀌지 않았으므로 안전하다. */
    bool sane = (rx_asm[2] == 0x01u || rx_asm[2] == 0x02u) &&
                (rx_asm[3] >= 0x01u && rx_asm[3] <= 0x03u) &&
                (plen > 0u) && (plen <= LORA_FRAME_MAX_PAYLOAD);
    if (!sane)
    {
      rx_discard(1);      /* 이 AA 55 는 가짜였다. 다음 것을 찾는다 */
      continue;
    }

    uint16_t total = (uint16_t)(plen + LORA_FRAME_OVERHEAD);
    if (rx_len < total) return 0;                   /* payload 가 아직 덜 왔다 */

    uint16_t want = (uint16_t)((rx_asm[total - 2u] << 8) | rx_asm[total - 1u]);
    uint16_t got  = LoRaFrame_Crc16(&rx_asm[2], (uint16_t)(total - 4u));

    if (want != got)
    {
      lora_rx_crc_errors++;
      rx_discard(2);      /* 이 프레임은 버리되, 뒤에 진짜가 있을 수 있다 */
      continue;
    }

    uint16_t n = plen;
    if (n > (uint16_t)(out_size - 1u)) n = (uint16_t)(out_size - 1u);
    memcpy(out, &rx_asm[LORA_FRAME_HEADER_LEN], n);
    out[n] = '\0';

    if (msg_type) *msg_type = rx_asm[3];
    if (seq) *seq = ((uint32_t)rx_asm[4] << 24) | ((uint32_t)rx_asm[5] << 16) |
                    ((uint32_t)rx_asm[6] << 8)  | (uint32_t)rx_asm[7];

    rx_discard(total);
    return n;
  }
}

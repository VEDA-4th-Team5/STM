/**
  ******************************************************************************
  * @file    lora_e22.h
  * @brief   EBYTE E22-900T22S(1B) driver (UART transparent transmission).
  *
  * NOTE: this part is an E22, NOT an E220. The two families differ in the M0/M1
  * mode encoding and in the register map, so E220 code does not work here:
  *   - E220 puts configuration mode at M1=1,M0=1; on the E22 that combination
  *     is DEEP SLEEP and the module answers nothing at all.
  *   - The E22 inserts NETID at 0x02, shifting REG0..REG3 up by one address.
  *
  * Hardware notes for this board (NUCLEO-F401RE):
  *   USART1 TX  PA9  (D8)  ---> module RXD
  *   USART1 RX  PA10 (D2)  <--- module TXD
  *   GPIO       PB4  (D5)  ---> module M0
  *   GPIO       PB5  (D4)  ---> module M1
  *
  * AUX is not soldered on our modules, so every state wait is a fixed delay
  * instead of an AUX poll. See LORA_MODE_SWITCH_DELAY_MS.
  ******************************************************************************
  */

#ifndef __LORA_E22_H
#define __LORA_E22_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

/* Mode pins ---------------------------------------------------------------- */
/*
 * AUX (module pin 9) -> PA8 (D7).
 *
 * 모듈이 바쁠 때 LOW, idle 로 돌아오면 HIGH 인 순수 출력이다. 이걸 안 보면
 * 송신 완료를 알 수 없어서 추정 딜레이로 때워야 하는데, 그러면 연속 송신
 * 때 앞 프레임이 아직 공중에 있는 상태로 다음 프레임을 밀어 넣게 된다.
 *
 * 입력에 풀업을 건다. 선이 안 붙어 있으면 항상 HIGH = "항상 준비됨" 으로
 * 읽혀서 AUX 없던 시절 동작(고정 딜레이)으로 조용히 되돌아간다. 즉 배선이
 * 틀려도 지금보다 나빠지지 않는다. 실제로 붙었는지는 송신 로그의
 * aux= 표시로 확인한다 -- 붙어 있으면 송신 중 LOW 가 관측된다.
 */
#define LORA_AUX_ENABLE     1
#define LORA_AUX_GPIO_Port  GPIOA
#define LORA_AUX_Pin        GPIO_PIN_8
#define LORA_AUX_TIMEOUT_MS 2000u   /* 이 안에 idle 로 안 오면 포기하고 진행 */

#define LORA_M0_GPIO_Port   GPIOB
#define LORA_M0_Pin         GPIO_PIN_4
#define LORA_M1_GPIO_Port   GPIOB
#define LORA_M1_Pin         GPIO_PIN_5

/* Timing (no AUX pin -> fixed delays) -------------------------------------- */
#define LORA_MODE_SWITCH_DELAY_MS   100u  /* datasheet needs ~2ms, 100 is safe */
#define LORA_CFG_REPLY_TIMEOUT_MS   500u

/* Configuration mode UART is fixed at 9600 8N1 whatever REG0 says.           */
#define LORA_CFG_BAUD               9600u

/**
 * Operating modes. The value encodes the pins directly: bit0 = M0, bit1 = M1.
 * Taken from the E22-900T22S(1B) manual section 6.
 */
typedef enum {
  LORA_MODE_NORMAL     = 0,   /* M1=0 M0=0 : transparent TX/RX          */
  LORA_MODE_WOR        = 1,   /* M1=0 M0=1 : wake-on-radio              */
  LORA_MODE_CONFIG     = 2,   /* M1=1 M0=0 : register read/write        */
  LORA_MODE_DEEP_SLEEP = 3    /* M1=1 M0=1 : asleep, answers nothing    */
} LoRa_Mode_t;

/* Register addresses (E22 series -- note NETID at 0x02) */
#define LORA_REG_ADDH       0x00
#define LORA_REG_ADDL       0x01
#define LORA_REG_NETID      0x02
#define LORA_REG_REG0       0x03  /* uart baud | parity | air data rate      */
#define LORA_REG_REG1       0x04  /* sub-packet | rssi noise | tx power      */
#define LORA_REG_REG2       0x05  /* channel                                 */
#define LORA_REG_REG3       0x06  /* rssi byte | fixed/transparent | LBT|WOR */
#define LORA_REG_CRYPT_H    0x07  /* write only, reads back as 0             */
#define LORA_REG_CRYPT_L    0x08  /* write only, reads back as 0             */
#define LORA_REG_COUNT      9

/**
 * Channel to frequency for the 1B (KC/Korea) variant, taken from the KC RF
 * test report: channels 1..32 map linearly onto 917.1 .. 923.3 MHz.
 *   f(MHz) = 916.9 + channel * 0.2
 * This is NOT the 850.125 + channel formula used by the plain E22-900T22S.
 */
#define LORA_CH_FREQ_BASE_KHZ   916900u
#define LORA_CH_STEP_KHZ           200u
#define LORA_CH_KC_MIN               1u
#define LORA_CH_KC_MAX              32u

/**
 * Transmit power codes for REG1 bits 1:0 (E22-900T22S).
 */
typedef enum {
  LORA_PWR_22DBM = 0,   /* 158 mW */
  LORA_PWR_17DBM = 1,   /*  50 mW */
  LORA_PWR_13DBM = 2,   /*  20 mW */
  LORA_PWR_10DBM = 3    /*  10 mW */
} LoRa_Power_t;

/**
 * REG0(0x03) bits 2:0 -- air data rate. 매뉴얼 표에서 확인한 값.
 * !! 통신하려면 양쪽 노드가 반드시 같아야 한다 ("Both parties must be the same").
 * 빠를수록 거리가 짧아지는 대신 공중 점유 시간이 줄어 duty cycle 여유가 커진다.
 */
typedef enum {
  LORA_AIR_0K3  = 0,
  LORA_AIR_1K2  = 1,
  LORA_AIR_2K4  = 2,   /* 공장 기본값 */
  LORA_AIR_4K8  = 3,
  LORA_AIR_9K6  = 4,
  LORA_AIR_19K2 = 5,
  LORA_AIR_38K4 = 6,
  LORA_AIR_62K5 = 7
} LoRa_AirRate_t;

/**
 * REG0(0x03) bits 7:5 -- 모듈의 UART(전선) 속도. 전파 위 속도가 아니라
 * MCU 와 모듈 사이 속도이므로 duty cycle 과는 무관하다.
 * 매뉴얼상 양쪽 노드가 서로 달라도 통신에는 지장이 없다.
 */
typedef enum {
  LORA_UART_1200   = 0,
  LORA_UART_2400   = 1,
  LORA_UART_4800   = 2,
  LORA_UART_9600   = 3,   /* 공장 기본값 */
  LORA_UART_19200  = 4,
  LORA_UART_38400  = 5,
  LORA_UART_57600  = 6,
  LORA_UART_115200 = 7
} LoRa_UartBaud_t;

/*
 * 이 프로젝트의 확정 설정 (전파법 검토 결과, KC RF 시험성적서 원문 대조 완료)
 *
 *   채널 30 (922.9 MHz)  : 채널 20~32 는 200 mW 까지 허용 -- 10 mW 는 1/20 수준
 *   출력 10 dBm          : E22 최저 단계
 *   LBT on               : 송신 전 5ms 캐리어 센싱, -65 dBm 임계
 *   에어레이트 62.5k     : 프레임 공중 점유가 2.4k 의 약 1/15 로 줄어
 *                          10초 주기에서 duty 가 1.5% -> 0.1% 가 된다
 *   UART 115200          : Pi 기본값(SENSOR_UART_BAUD)과 맞춤
 */
#define LORA_AIR_PROJECT   LORA_AIR_62K5
#define LORA_UART_PROJECT  LORA_UART_115200

/*
 * 모듈 굽기 모드 (평상시 0).
 *
 * Pi 에 붙는 모듈은 MCU 가 없어 스스로 설정하지 못한다. 이 보드에 잠깐 물려
 * C0(영구 저장)로 구워서 넘겨야 한다. 평상시 경로는 persist=false(C2, 휘발성)
 * 라 전원을 끊으면 공장값(채널 2 = RFID 전용, 22dBm)으로 돌아가므로 그대로는
 * 못 쓴다.
 *
 * 1 로 빌드하면 부팅 시 굽고 되읽어 확인한 뒤 **센서 동작으로 넘어가지 않고
 * 그 자리에서 멈춘다.** 굽기는 한 번이면 되므로 부팅마다 반복해 모듈 플래시
 * 수명을 깎을 이유가 없고, 켜둔 채 잊어버려도 보드가 센서 노드로 동작하지
 * 않아 즉시 알아차릴 수 있다.
 *
 * 굽고 나면 모듈을 Pi 로 옮기고 이 값을 0 으로 되돌려 재빌드할 것.
 */
#define LORA_BURN_MODULE   0

/*
 * 노드 주소.
 *
 * 처음엔 투명전송 모드였고, 그때는 주소가 통신에 영향이 없다고 보고 순수
 * 라벨로 썼다. 그건 틀렸다 -- 투명 모드는 송·수신 주소가 같아야 동작해서,
 * 세 모듈을 서로 다르게 구운 것이 곧바로 "전파는 도달하는데 UART 로는
 * 아무것도 안 올라오는" 증상이 됐다.
 *
 * 그래서 고정점(fixed-point) 모드로 전환했다. 프레임 앞에 목적지
 * <ADDH ADDL CH> 3 byte 를 붙이면 모듈이 그걸 떼어내고 해당 주소로 보낸다.
 * 주소가 진짜로 의미를 갖고, 나중에 STM2 가 붙어도 링크 계층에서 구분된다.
 */
#define LORA_ADDR_STM1     0x0001u
#define LORA_ADDR_STM2     0x0002u
#define LORA_ADDR_RPI      0x0010u

/* 이 보드의 주소. 부팅할 때마다 모듈에 써 넣으므로, 어떤 모듈을 꽂든
 * STM1 보드에 꽂힌 모듈은 0x0001 이 된다. 모듈을 바꿔 끼울 때마다
 * "이게 몇 번이더라" 를 따질 필요가 없어진다. */
#define LORA_MY_ADDRESS    LORA_ADDR_STM1
/* 기본 목적지. 센서 데이터는 전부 Pi 로 간다. */
#define LORA_PEER_ADDRESS  LORA_ADDR_RPI

#define LORA_BURN_ADDRESS  LORA_ADDR_RPI    /* 굽기 전에 대상에 맞게 바꿀 것 */
#define LORA_BURN_NETID    0x00u

/*
 * LBT(Listen Before Talk). KC 기술기준 항목이라 운용 시에는 1 이어야 한다.
 *
 * 다만 LBT 는 채널이 -65dBm 이상이면 송신을 미루거나 포기하면서 그 사실을
 * STM32 에 알려주지 않는다. 즉 "LORA OK" 가 찍혀도 전파가 안 나갔을 수 있어,
 * 링크가 안 붙을 때 원인 후보에서 배제하려면 0 으로 내려 확인해야 한다.
 * duty 가 한도 2% 대비 1.3% 라 잠깐 꺼도 다른 기술기준은 계속 충족한다.
 */
#define LORA_LBT_ENABLE    1

/* 연속 송신 사이에 두는 여유(ms). 공중 점유 추정치에 이만큼 더 쉰다.
 * AUX 를 읽어 idle 을 확인하는 방식으로 바꾸면 필요 없어진다. */
#define LORA_TX_GAP_MS     40u

/* 무선 암호화 키(CRYPT_H/L). 모든 모듈이 같은 값이어야 통신된다.
 * write-only 라 읽어서 확인할 수 없으므로 반드시 명시적으로 굽는다. */
#define LORA_CRYPT_KEY     0x0000u

/*
 * Korean channel plan for this band, from the KC RF test report annex:
 *   ch 1,3,4,6,7,9,10,12,13,15,16,18 ......  3 mW
 *   ch 2,5,8,11,14,17 ..................... RFID readers (4 W); 10 mW otherwise
 *   ch 19..23 ............................. 10 mW
 *   ch 24,25 .............................. 25 mW
 *   ch 26..32 ............................ 200 mW   <- general data links
 * Duty cycle: <=10 mW 2%, 10-25 mW 1%, >25 mW 0.5%.
 *
 * The factory default channel 2 is an RFID reader channel, so it is the wrong
 * place for this project. Use LORA_CH_DATA_MIN..LORA_CH_KC_MAX.
 */
#define LORA_CH_DATA_MIN            26u   /* 922.1 MHz */
#define LORA_CH_DATA_DEFAULT        30u   /* 922.9 MHz */

extern UART_HandleTypeDef hlora;

/* API ---------------------------------------------------------------------- */
void        LoRa_Init(void);                       /* GPIO + USART1 @ 9600 8N1 */
void        LoRa_SetMode(LoRa_Mode_t mode);
void        LoRa_FlushRx(void);
void        LoRa_SetBaud(uint32_t baud);

/* Config mode helpers. Caller must already be in LORA_MODE_CONFIG. */
HAL_StatusTypeDef LoRa_ReadRegisters(uint8_t start, uint8_t len, uint8_t *out);
HAL_StatusTypeDef LoRa_WriteRegisters(uint8_t start, uint8_t len, const uint8_t *data);

/* Normal mode data path. */
HAL_StatusTypeDef LoRa_Send(const uint8_t *buf, uint16_t len);

/**
 * @brief  고정점 모드로 특정 주소에 보낸다.
 * @param  addr     목적지 노드 주소 (LORA_ADDR_*)
 * @param  channel  목적지 채널 (보통 LORA_CH_DATA_DEFAULT)
 *
 * LoRa_Send() 는 이 함수를 LORA_PEER_ADDRESS 로 호출하는 얇은 껍데기다.
 * 나중에 STM2 나 다른 노드로 직접 보낼 일이 생기면 이쪽을 쓴다.
 */
HAL_StatusTypeDef LoRa_SendTo(uint16_t addr, uint8_t channel,
                              const uint8_t *buf, uint16_t len);

/**
 * @brief  AUX 가 HIGH(idle) 로 돌아올 때까지 기다린다.
 * @retval true  idle 확인 / false  timeout (호출자는 그냥 진행한다)
 *
 * AUX 미배선이면 풀업 때문에 즉시 true 다. 그 경우 이 함수는 아무 일도 하지
 * 않으므로, 호출부는 AUX 유무와 상관없이 동일하게 쓰면 된다.
 */
bool LoRa_WaitAuxIdle(uint32_t timeout_ms);

/**
 * @brief  지금까지 송신 중 AUX 가 LOW 로 떨어지는 것을 본 적이 있는가.
 *
 * true 면 AUX 선이 실제로 살아 있다는 증거다. 계속 false 면 미배선이거나
 * 엉뚱한 핀에 붙은 것이고, 그때 타이밍은 여전히 추정 딜레이에 의존한다.
 */
bool LoRa_AuxDetected(void);

/**
 * @brief  직전/최대 송신 소요 시간(ms). AUX 가 LOW 로 있던 시간이다.
 *
 * 프레임 하나의 공중 점유는 62.5k 에서 약 5 ms 다. 이 값이 그보다 크게 길면
 * LBT 가 채널이 비기를 기다린 것이고, 계속 커지면 채널이 혼잡하다는 뜻이다.
 * LBT 는 송신을 미루거나 포기해도 알려주지 않으므로 이 수치로만 알 수 있다.
 * AUX 미배선이면 항상 0 이라 아무 정보도 주지 못한다.
 */
uint32_t LoRa_GetLastTxBusyMs(void);
uint32_t LoRa_GetMaxTxBusyMs(void);
uint16_t          LoRa_Recv(uint8_t *buf, uint16_t maxlen, uint32_t timeout_ms);

/**
 * @brief  Bring-up check: enters config mode, reads all 9 parameter bytes,
 *         dumps them decoded over printf, then returns to normal mode.
 * @retval true when the module answered with a valid 0xC1 frame.
 */
bool        LoRa_SelfTest(void);

/**
 * @brief  Set the RF channel, transmit power and LBT, then return to normal mode.
 *         Handles entering/leaving configuration mode itself.
 * @param  channel  RF channel; see the channel plan above.
 * @param  power    transmit power code.
 * @param  lbt      listen-before-talk on/off.
 * @param  persist  true writes with C0 (survives power down), false uses C2.
 * @retval HAL_OK when the module echoed the new settings back.
 */
HAL_StatusTypeDef LoRa_Configure(uint8_t channel, LoRa_Power_t power,
                                 bool lbt, bool persist);

/**
 * @brief  채널/출력/LBT 에 더해 REG0(UART baud + 에어레이트)까지 설정한다.
 *
 *         설정모드 UART 는 REG0 값과 무관하게 항상 9600 이므로, 이 함수가
 *         진입 전에 로컬 USART1 을 9600 으로 내리고 빠져나올 때 새 baud 로
 *         올린다. 호출한 쪽은 baud 를 신경 쓰지 않아도 된다.
 *
 * !! 에어레이트는 양쪽 노드가 같아야 한다. 한쪽만 바꾸면 통신이 끊긴다.
 *    Pi 쪽 모듈도 같은 값으로 맞춘 뒤에 적용할 것.
 *
 * @param  persist  true 면 C0(영구 저장), false 면 C2(전원 끄면 소멸)
 */
HAL_StatusTypeDef LoRa_ConfigureFull(uint8_t channel, LoRa_Power_t power, bool lbt,
                                     LoRa_AirRate_t air, LoRa_UartBaud_t uart,
                                     bool persist);

/**
 * @brief  LoRa_UartBaud_t 코드를 실제 baud 숫자로 바꾼다.
 */
uint32_t LoRa_BaudCodeToRate(LoRa_UartBaud_t code);

/**
 * @brief  전파법 인터록. LoRa_ConfigureFull() 이 되읽기 대조까지 통과했을 때만
 *         true. false 인 동안 모듈은 공장 기본값(채널 2 = RFID 리더 전용,
 *         22 dBm = 158 mW)일 수 있고 그 상태로 송신하면 위법이므로,
 *         lora_frame.c 가 이 값을 보고 송신을 막는다.
 */
bool LoRa_IsConfigured(void);

/**
 * @brief  E22/E220 판별. 같은 M1/M0 조합이 두 계열에서 다른 모드를 뜻하므로,
 *         두 인코딩으로 각각 설정모드 진입을 시도해 어느 쪽이 응답하는지 본다.
 *         지난번에 이 차이 때문에 모듈이 딥슬립에 빠져 하루를 날렸다.
 */
void LoRa_DetectVariant(void);

/**
 * @brief  일반 모드로 내린 뒤에도 모듈이 설정 명령에 응답하는지 본다.
 *         응답하면 M1 이 모듈에 닿지 않아 설정 모드에 갇혀 있다는 뜻이고,
 *         그러면 송신 바이트가 전파 대신 설정 명령으로 해석돼 버려진다.
 */
void LoRa_CheckNormalMode(void);

/**
 * @brief  레지스터 0x00~0x08 전체를 USART2 에 찍는다.
 *
 * Pi 쪽 모듈은 MCU 가 없어 SSH 로만 덤프를 뜰 수 있고, 그 값과 byte 단위로
 * 대조해야 "전파는 오가는데 복조가 안 되는" 원인을 좁힐 수 있다.
 * CRYPT(0x07/0x08)는 write-only 라 항상 00 00 으로 읽힌다 -- 정상이다.
 */
void LoRa_DumpRegisters(void);

/**
 * @brief  주소/NETID/채널/출력/LBT/에어레이트/UART 를 한 번에 **영구 저장**(C0)한다.
 *
 *         Pi 쪽 모듈처럼 MCU 없이 동작할 모듈을 준비하는 용도. 평상시 경로가
 *         쓰는 C2(휘발성)와 달리 전원을 끊어도 유지된다.
 *         쓰기 후 되읽어 대조하며, 하나라도 어긋나면 실패로 본다.
 *
 * @param  address  ADDH/ADDL 에 새길 16bit 주소 (투명전송에서는 라벨 용도)
 * @param  netid    NETID
 * @retval HAL_OK   쓰기와 되읽기 대조까지 통과
 */
HAL_StatusTypeDef LoRa_BurnModule(uint16_t address, uint8_t netid);

/* Diagnostics (kept from bring-up -- see LoRa_Diag for the full sweep). */
bool        LoRa_LoopbackTest(void);
void        LoRa_LineProbe(void);
void        LoRa_SwapTest(void);
void        LoRa_ShortTest(void);
void        LoRa_Diag(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_E22_H */

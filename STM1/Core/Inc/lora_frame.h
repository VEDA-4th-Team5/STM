/**
  ******************************************************************************
  * @file    lora_frame.h
  * @brief   Pi_Server 가 기대하는 LoRa binary frame 인코더.
  *
  * 규격 출처: Pi_Server `docs/UART_LORA_PROTOCOL.md` (develop 기준).
  * Pi 쪽에 `LoRaDriver` / `parking-link-tool` 이 이미 구현되어 있으므로
  * 새로 설계하지 않고 그 규격을 그대로 따른다.
  *
  *   Offset  Size  Field
  *   ------  ----  -------------------------------------------------
  *      0      1   SOF 0xAA
  *      1      1   SOF 0x55
  *      2      1   Version 0x01
  *      3      1   Message type
  *      4      4   Transport sequence   (big-endian)
  *      8      2   Payload length       (big-endian, 최대 512)
  *     10      N   Payload
  *   10+N      2   CRC16-CCITT          (big-endian)
  *
  * CRC 범위는 Version(offset 2)부터 payload 마지막 byte까지.
  * init 0xFFFF, poly 0x1021, 반전 없음, 최종 XOR 없음 (CRC-16/CCITT-FALSE).
  * 다중 byte 정수는 전부 network byte order.
  *
  * SensorEvent payload 는 기존 센서 문자열을 그대로 넣는다:
  *   SENSOR:HALL01:OCCUPIED:15
  * 프레임에 길이 필드가 있으므로 CRLF 는 붙이지 않는다(문서 예시와 동일).
  * Pi 파서가 어차피 trim 하므로 붙어 있어도 파싱은 되지만, 공중 점유
  * 시간을 2 byte만큼 낭비할 이유가 없다.
  ******************************************************************************
  */

#ifndef __LORA_FRAME_H
#define __LORA_FRAME_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

#define LORA_FRAME_SOF0        0xAAu
#define LORA_FRAME_SOF1        0x55u
/* 0x01 = v1.0 페이로드 (노드 ID 없음, 스트림별 seq, energy 없음)
 * 0x02 = v1.1 페이로드 (노드 ID 추가, 노드 단일 seq, 화재 energy 동봉)
 *
 * 버전 바이트를 올려두면 수신측이 구/신을 동시에 받을 수 있다. STM 과 Pi 를
 * 같은 순간에 배포할 필요가 없어지고, 한쪽만 롤백해도 링크가 안 깨진다.
 * 페이로드 의미가 바뀔 때마다 여기를 올릴 것. */
#define LORA_FRAME_VERSION     0x02u

#define LORA_FRAME_HEADER_LEN  10u   /* SOF..길이 필드까지 */
#define LORA_FRAME_CRC_LEN      2u
#define LORA_FRAME_OVERHEAD    (LORA_FRAME_HEADER_LEN + LORA_FRAME_CRC_LEN)

/* 규격상 payload 상한은 512지만, E22 서브패킷이 240 byte라 그 안에 들어가야
 * 한 번에 나간다. 센서 문자열은 40 byte 내외라 여유가 크다. */
#define LORA_FRAME_MAX_PAYLOAD 200u
#define LORA_FRAME_MAX_LEN     (LORA_FRAME_MAX_PAYLOAD + LORA_FRAME_OVERHEAD)

typedef enum {
  LORA_MSG_SENSOR_EVENT  = 0x01,
  LORA_MSG_ALERT_COMMAND = 0x02,
  LORA_MSG_HEARTBEAT     = 0x03
} LoRaMsgType_t;

/**
 * 전파법 duty cycle 예산 관리.
 *
 * 이전 구현은 "10초에 1프레임" 고정 간격이었는데 두 가지가 틀렸다.
 *
 *  1) 상수가 에어레이트 2.4k 기준으로 계산돼 있었다. 지금은 62.5k 라
 *     같은 프레임의 공중 점유가 약 1/15 이고, 필요한 간격은 10초가 아니라
 *     0.5초 수준이다. 20배 과보수적이었다.
 *  2) 고정 간격은 버스트를 처리하지 못한다. 홀 4채널이 동시에 바뀌면
 *     4프레임이 연달아 나가야 하는데, 앞의 하나만 통과하고 나머지는
 *     "실제 상태 변화"인데도 버려졌다.
 *
 * 그래서 간격이 아니라 예산으로 바꿨다. 60초 창 안에서 누적 공중 점유가
 * 한도를 넘지 않는 한 통과시킨다. 버스트는 예산 안에서 그대로 나가고,
 * 예산이 바닥나야 비로소 막힌다.
 *
 * 한도: 10 dBm 은 duty 2% -> 60초 중 1200 ms.
 * 보수적으로 80% 만 쓴다(LBT 재시도와 추정 오차 여유).
 */
#define LORA_DUTY_WINDOW_MS      60000u
#define LORA_DUTY_BUDGET_MS       1200u   /* 60s 의 2% */
#define LORA_DUTY_SAFETY_PCT        80u   /* 예산의 80% 만 사용 */

/**
 * 공중 점유 시간 추정용 에어레이트(bps)와 고정 오버헤드(ms).
 *
 * LoRa 는 preamble/헤더가 붙으므로 payload 만으로는 과소평가된다.
 * 실측 전까지는 고정 오버헤드를 넉넉히 잡아 안전한 쪽으로 틀리게 한다.
 * 62.5k 에서 40 byte 프레임이면 5 ms + 8 ms = 13 ms 로 계산된다.
 */
#define LORA_AIRTIME_BPS         62500u
#define LORA_AIRTIME_OVERHEAD_MS     8u

/**
 * 1 = LoRa 로 실제로 나간 프레임을 USART2(PuTTY)에 hex 로 찍는다.
 *
 * LoRa 트래픽은 USART1 로 나가서 PuTTY 에 안 보이므로, 무선 쪽이 제대로
 * 나가는지 확인하려면 이게 필요하다. Pi 의 parking-link-tool encode 출력과
 * 바이트 단위로 대조할 수 있다.
 *
 * !! Pi 안전하지 않다. USART2 는 Pi 데이터 링크라 이 줄이 섞이면 파서가
 *    오염된다. 벤치에서 PuTTY 로 볼 때만 1 로 두고 연동 시 0 으로 되돌릴 것.
 */
#define LORA_DEBUG_HEX 0

/* 1 = LoRa 로 들어온 바이트를 USART2 에 찍는다.
 *
 * Pi 링크가 LoRa 로 옮겨가면서 USART2 는 콘솔 전용이 됐으므로 켜둬도
 * 프로토콜을 오염시키지 않는다. 수신량이 적어 부담도 없고, 하행(ALERT 명령)
 * 이 실제로 도착하는지 확인할 수 있는 유일한 수단이라 남겨둔다. */
#define LORA_DEBUG_RX 1

/**
 * @brief  프레임을 버퍼에 인코딩한다. I/O 없음 -- 단위 검증용으로도 쓴다.
 * @param  out         출력 버퍼
 * @param  out_size    출력 버퍼 크기
 * @param  msg_type    LoRaMsgType_t
 * @param  seq         transport sequence
 * @param  payload     payload 시작 주소
 * @param  payload_len payload 길이
 * @retval 인코딩된 총 byte 수. 0 이면 인자 오류 또는 버퍼 부족.
 */
uint16_t LoRaFrame_Encode(uint8_t *out, uint16_t out_size,
                          uint8_t msg_type, uint32_t seq,
                          const uint8_t *payload, uint16_t payload_len);

/**
 * @brief  CRC16-CCITT (init 0xFFFF, poly 0x1021).
 */
uint16_t LoRaFrame_Crc16(const uint8_t *data, uint16_t len);

/**
 * @brief  문자열 payload 를 프레임으로 감싸 LoRa 로 송신한다.
 * @param  msg_type    LoRaMsgType_t
 * @param  seq         transport sequence (payload 안의 seq 와 같게 쓰는 것을
 *                     Pi 문서가 권장)
 * @param  line        널종단 문자열. 끝의 CR/LF 는 자동으로 잘라낸다.
 * @param  critical    true 면 duty 리미터를 우회한다(화재 등).
 * @retval HAL_OK 송신됨 / HAL_BUSY 리미터에 걸려 버려짐 / 그 외 오류
 */
HAL_StatusTypeDef LoRaFrame_SendLine(uint8_t msg_type, uint32_t seq,
                                     const char *line, bool critical);

/**
 * @brief  duty 리미터에 걸려 버려진 프레임 수. 전송 정책 튜닝용.
 */
uint32_t LoRaFrame_GetDroppedCount(void);

/**
 * @brief  현재 60초 창에서 쓴 공중 점유 시간(ms). 0 이면 예산 여유가 가득.
 */
uint32_t LoRaFrame_GetAirtimeUsedMs(void);

/**
 * @brief  프레임 길이로부터 공중 점유 시간을 추정한다(ms).
 */
uint32_t LoRaFrame_EstimateAirtimeMs(uint16_t frame_len);

/**
 * @brief  LoRa 수신 링버퍼를 소비해 완성된 프레임을 꺼낸다. 논블로킹.
 *
 * SOF(AA 55) 를 찾고 version/type/len 을 검사한 뒤 CRC 까지 맞아야 통과시킨다.
 * 쓰레기 바이트에서 우연히 나온 AA 55 를 프레임으로 오인하지 않도록,
 * 헤더가 말이 안 되면 1바이트만 버리고 다시 동기를 잡는다.
 *
 * @param  out       payload 를 받을 버퍼 (널종단해서 돌려준다)
 * @param  out_size  버퍼 크기
 * @param  msg_type  받은 프레임의 메시지 타입 (NULL 가능)
 * @param  seq       받은 프레임의 transport sequence (NULL 가능)
 * @retval payload 길이. 0 이면 아직 완성된 프레임이 없다.
 */
uint16_t LoRaFrame_Poll(char *out, uint16_t out_size,
                        uint8_t *msg_type, uint32_t *seq);

/** @brief CRC 불일치로 버린 프레임 수. 링크 품질 지표. */
uint32_t LoRaFrame_GetRxCrcErrorCount(void);

#ifdef __cplusplus
}
#endif

#endif /* __LORA_FRAME_H */

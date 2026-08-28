#ifndef __SENSOR_PROTOCOL_H
#define __SENSOR_PROTOCOL_H

#include <stdint.h>
#include "board_config.h"

/* 센서 페이로드 규격 v1.1 (프레임 version = 0x02)
 *
 *   SENSOR:<node>:<sensor>:<state>:<seq>\r\n
 *   FIRE:<node>:<sensor>:<state>:<seq>:<energy>\r\n
 *
 *   SENSOR:STM1:HALL01:OCCUPIED:142
 *   FIRE:STM1:FLAME01:DETECTED:144:43.70
 *
 * v1.0 대비 바뀐 것:
 *
 *  1) <node> 추가 -- 어느 보드에서 온 이벤트인지 링크 계층에서 알 수 없었다.
 *     LoRa 고정점 헤더에 주소가 있지만 모듈이 떼어내므로 수신측에는 안 온다.
 *     STM2 가 붙으면 센서 이름이 겹치지 않는다는 약속에만 기대게 되는데,
 *     그건 규약이 아니다.
 *
 *  2) <seq> 를 노드 단일 카운터로 통일 -- 예전엔 홀/화재가 각각 셌다.
 *     Pi 의 시퀀스 가드는 단일 스트림으로 보고 있어서, 홀 141 다음 화재 1 이
 *     오면 역행으로 판정했다. 이제 프레임 transport seq 와 항상 같은 값이다.
 *
 *     !! 재부팅하면 이 값이 1 로 돌아간다. 수신측은 "seq 가 크게 감소 =
 *        노드 재부팅"으로 보고 시퀀스 상태를 초기화해야 한다. 그렇게 하지
 *        않으면 재부팅 후 그 노드가 영구히 막힌다(Pi 쪽 기존 결함).
 *
 *  3) <energy> 추가 -- 1~20Hz 대역 에너지. 화재 프레임에만 붙는다.
 *     Qt 대시보드 시각화용. 판정 자체는 STM32 가 계속 책임진다.
 *
 * 센서 ID 는 Pi 의 config/parking_slots.json(HALL01~04) 과
 * .env.fire.local 의 FIRE_SENSOR_SLOT_MAP(FLAME01) 에 맞춘 값이다.
 *
 * 전송 주기 (전파법 duty 예산 안에서 설계됨. 최악 1.36%, 한도 2%):
 *   - HALL  : 상태 변화 즉시 + 30초 하트비트.
 *             단 채널별 SENSOR_HALL_COOLDOWN_MS 쿨다운이 걸린다(아래 참고).
 *   - FIRE  : 상태 변화 즉시 + 30초 하트비트.
 *             DETECTED 인 동안에는 SENSOR_FLAME_TELEMETRY_MS 마다 energy 갱신.
 *
 * 어느 쪽이든 전선에 나가는 값은 *디바운스가 끝난* 상태다. Pi 는 재디바운스
 * 하지 않는다(팀 계약: 디바운스는 STM32 책임).
 */

/* 전송 경로 선택 (EVDA-46).
 *   SENSOR_TX_UART : USART2 로 문자열을 그대로 보낸다. 지금까지 검증된 경로.
 *   SENSOR_TX_LORA : 같은 문자열을 Pi 규격 LoRa binary frame 에 실어 보낸다.
 *                    규격은 Pi_Server docs/UART_LORA_PROTOCOL.md, 구현은
 *                    lora_frame.c 참고.
 * 둘 다 1로 두면 같은 이벤트가 유선·무선 양쪽으로 나가므로 A/B 비교가 된다.
 * LoRa 를 켜려면 main.c 에서 LoRa_Init() 이 먼저 호출되어야 한다.
 *
 * !! LoRa 를 켜기 전에 읽을 것: 홀센서는 1초마다 4채널을 전부 보낸다.
 * 그대로 무선으로 흘리면 전파법 duty cycle 한도를 크게 넘는다. lora_frame.c
 * 의 리미터가 초과분을 버리므로 위법 송신은 막히지만, 그만큼 홀 상태가
 * 유실된다. 무선 전환 시 전송 주기 정책을 다시 정해야 한다. */
#define SENSOR_TX_UART 1
#define SENSOR_TX_LORA 1

/* 1 = human-readable ANSI dashboard on UART for watching in PuTTY (fire/
 * occupied highlighted, screen redraws in place). 0 = raw SENSOR:/FIRE:
 * lines the Raspberry Pi parser expects. These are mutually exclusive on
 * the same wire -- only flip this to 1 for local PuTTY-only testing before
 * the Pi is actually wired up; must be 0 for real Pi integration. */
#define SENSOR_DEBUG_UI 0

/* 1 = also print the per-window flame energy/delta as a comment line, so the
 * thresholds can be retuned without a debugger. Not Pi-safe: keep at 0 for
 * integration.
 *
 * 지금은 1 이다 -- FLAME_RAW_THRESHOLD(2500) 를 시험 환경 실측으로 확정하기
 * 위해 켜뒀다. 출력 형식은 baseline 을 없애면서 이렇게 바뀌었다:
 *
 *   # raw=352.1 base=2500.0 e=0.6210 d=-2147.9 hit=0 votes=0 -> CLEARED
 *          ^raw DC   ^임계값        ^에너지  ^임계 대비 여유
 *
 * base 는 이제 고정 임계값이고, d 는 raw - 임계값이라 음수면 안전한 것이다.
 * !! 임계 확정하면 다시 0 으로 되돌릴 것. 매 초 UART 로 한 줄씩 나가므로
 *    Pi 연동 상태로 두면 파서에 잡음이 섞인다. */
#define FLAME_DEBUG_ENERGY 1

/* ==========================================================================
 * 보드 구성
 * ==========================================================================
 *
 * 여기서 정하지 않는다. board_config.h 의 BOARD_SELECT 한 줄이 정본이고,
 * 아래는 그것을 이 파일이 쓰는 이름으로 옮겨 놓은 것뿐이다.
 *
 * 보드를 바꾸려면 board_config.h 만 열면 된다. 노드 ID 는 링크 진단·재부팅
 * 감지·하행 라우팅에 쓰인다.
 */
#define SENSOR_NODE_ID     BOARD_NODE_ID
#define SENSOR_HALL_COUNT  BOARD_HALL_COUNT
#define SENSOR_HALL_BASE   BOARD_HALL_BASE
#define SENSOR_HAS_FLAME   BOARD_HAS_FLAME

/* 슬롯 인덱스(0-based) -> 전선에 나가는 센서 번호 */
#define SENSOR_HALL_ID(idx)  ((unsigned)(SENSOR_HALL_BASE + (idx)))

/*
 * 홀 채널별 최소 재전송 간격(ms).
 *
 * 첫 상태 변화는 항상 즉시 나간다. 이 쿨다운은 *그 다음* 변화부터 적용되며,
 * 쿨다운 중에 생긴 변화는 버리지 않고 최신 상태만 기억했다가 쿨다운이
 * 끝날 때 한 번 보낸다(합치기). 그래서 채터링이 나도 프레임 수는 늘지 않고,
 * 수신측 상태는 최대 이 시간 안에 반드시 맞아진다.
 *
 * 값의 근거 -- 4채널이 계속 떨어도 duty 예산을 넘지 않는 선:
 *   4채널 x (60초 / 5초) x 13.5ms = 648ms
 *   + 화재 연소 중 텔레메트리 169ms = 817ms  (내부 예산 960ms, 법정 1200ms)
 *
 * 이게 없으면 채터링 시 duty 리미터가 프레임을 *버리고*, 버려진 게 상태
 * 변화라면 수신측은 다음 하트비트(30초)까지 틀린 상태를 들고 있게 된다.
 * 전파법을 지키려다 데이터 정합성을 잃는 셈이라 그쪽이 더 나쁘다.
 */
#define SENSOR_HALL_COOLDOWN_MS      5000u

/*
 * DETECTED 인 동안 energy 를 다시 보내는 간격(ms).
 *
 * 연속 스트리밍은 불가능하다 -- 1초 간격이면 분당 780ms 로 duty 예산의
 * 81% 를 화재 하나가 먹는다. 대신 평상시에는 조용하고(30초 하트비트),
 * 불이 났을 때만 촘촘해지도록 했다. Qt 는 그때 5초 간격 곡선을 얻는다.
 */
#define SENSOR_FLAME_TELEMETRY_MS    5000u

/*
 * DETECTED 엣지 재전송 (EVDA-124).
 *
 * 노드가 둘이 되면서 half-duplex 충돌로 프레임이 씹힐 여지가 생겼다.
 * 추정 충돌률은 프레임당 0.5% 수준(LBT 적용 전)으로 낮지만, 하필 그 순간이
 * 화재면 다음 하트비트까지 서버가 모른다.
 *
 * 하트비트를 더 줄이는 것보다 이쪽이 훨씬 싸다. 하트비트는 아무 일 없을
 * 때도 계속 비용을 내지만, 재전송은 불이 났을 때만 낸다.
 *   추가 비용 = 2회 x 약 14ms = 28ms, 그것도 화재 순간에만
 *
 * 세 번이 모두 충돌할 확률은 사실상 0 이다. CLEARED 는 재전송하지 않는다 --
 * 그건 하트비트가 다시 알려주는 상태값이다.
 */
#define SENSOR_FLAME_REPEAT_COUNT     2u    /* 최초 전송에 더해 몇 번 더 */
#define SENSOR_FLAME_REPEAT_GAP_MS  200u

/*
 * 하트비트 주기(ms). "이 노드가 살아 있다"를 알리는 간격이자, 프레임이
 * 유실됐을 때 수신측이 틀린 상태를 들고 있는 최대 시간이다.
 *
 * ★ 세 곳이 이 값에 묶여 있다. 따로 고치면 조용히 반쪽만 동작한다.
 *   1) 홀 하트비트 타이머 주기
 *   2) 홀 하트비트의 "최근에 보낸 채널 건너뛰기" 창
 *   3) 화재 CLEARED 상태의 전송 간격 (틱 수로 환산)
 *
 * 특히 2번을 안 맞추면 타이머는 도는데 매번 아무것도 안 보낸다.
 *
 * 30초에서 10초로 줄였다. 노드가 둘이 되면서 충돌로 프레임이 씹힐 여지가
 * 생겼는데, 씹히면 다음 하트비트까지 수신측 상태가 틀린 채로 남는다.
 * 그 창을 30초로 두는 것은 길다.
 *
 * duty 예산 (프레임 46 byte = 공중 점유 약 14 ms, 내부 예산 960 ms/60초):
 *
 *   조용할 때 STM1 은 3프레임(HALL01/02 + FLAME01)을 매 주기 보낸다.
 *     30초 ->  84 ms/60초  (예산의  9%, 법정 duty 0.14%)
 *     10초 -> 252 ms/60초  (예산의 26%, 법정 duty 0.42%)   <- 현재
 *      5초 -> 504 ms/60초  (예산의 53%, 법정 duty 0.84%)
 *
 *   바쁠 때는 하트비트가 최근 전송분을 건너뛰므로 여기에 더해지지 않는다.
 *   최악(홀 채터링 336 ms + 화재 텔레메트리 168 ms)은 그대로 약 504 ms 다.
 *
 * 5 초까지 내려도 법정 한도(2%)에는 한참 못 미치지만 권하지 않는다.
 * 홀 쿨다운이 5초라 그 구간은 어차피 개선되지 않고, 조용할 때 배경
 * 트래픽만 두 배가 된다.
 *
 * !! 화재 유실 대책으로 이 값을 줄이지 말 것. 무딘 도구다. DETECTED 를
 *    짧은 간격으로 몇 번 재전송하는 쪽이 훨씬 싸다(EVDA-124).
 */
#define SENSOR_HEARTBEAT_MS          10000u

/*
 * 노드별 하트비트 시작 오프셋(ms).
 *
 * 두 보드를 같이 켜면 하트비트 타이머가 정렬돼서, 매 주기 같은 시점에
 * 부딪힌다. LoRa 는 half-duplex 라 그러면 한쪽이 계속 지는 형태가 될 수
 * 있다 -- 확률적 충돌보다 나쁘다. 특정 노드만 만성적으로 유실된다.
 *
 * LBT 가 보통 갈라주지만, 둘이 정확히 같은 순간에 캐리어 센싱을 하면
 * 둘 다 "비었다"고 보고 같이 나간다. LBT 가 못 막는 유일한 경우다.
 *
 * 반 주기씩 어긋내면 그 상황이 구조적으로 안 생긴다. 비용은 0 이다.
 *   STM1 -> 0 ms / STM2 -> 5000 ms
 */
/* 캐스트를 쓰지 않는다. #if 는 캐스트를 평가하지 못해서, (uint32_t) 를
 * 넣으면 전처리 단계에서 깨진다. */
#define SENSOR_HEARTBEAT_STAGGER_MS \
  ((BOARD_SELECT - 1) * (SENSOR_HEARTBEAT_MS / 2u))

/* slot_index is 0-3; sent to the Pi as HALL01..HALL04 (1-based). */
void SensorProtocol_SendHallStatus(uint8_t slot_index, uint8_t occupied);
/* verdict: 1 = DETECTED, 0 = CLEARED. energy 는 1~20Hz 대역 에너지로,
 * 이제 전선에 실려 나간다(Qt 시각화용). 판정은 여전히 STM32 가 한다. */
void SensorProtocol_SendFlameStatus(uint8_t verdict, float energy);

/*
 * 직전 화재 라인을 **같은 seq 로 바이트 단위 동일하게** 다시 보낸다(EVDA-124).
 *
 * DETECTED 는 놓치면 그 순간이 다시 오지 않는다. LoRa 는 half-duplex 라
 * 두 노드가 겹치면 프레임이 통째로 유실되는데, 송신측은 그 사실을 알 수
 * 없다(LBT 는 포기해도 알려주지 않고, HAL_UART_Transmit 은 항상 성공을
 * 돌려준다). 그래서 확인 대신 몇 번 더 보낸다.
 *
 * 같은 seq 이므로 수신측 시퀀스 가드가 중복으로 걸러낸다 - 수신측 변경이
 * 필요 없다. */
void SensorProtocol_ResendLastFlame(void);
void SensorProtocol_SendFlameEnergyDebug(float raw_avg, float baseline, float energy,
                                         float delta, uint8_t raw_verdict,
                                         uint8_t votes, uint8_t verdict);

/* Dashboard mode only (SENSOR_DEBUG_UI=1) -- no-ops when disabled. */
void SensorDashboard_Init(void);
void SensorDashboard_UpdateHall(uint8_t slot_index, uint8_t occupied);
void SensorDashboard_UpdateFlame(uint8_t verdict, float energy);

#endif /* __SENSOR_PROTOCOL_H */

/**
  ******************************************************************************
  * @file    board_config.h
  * @brief   어느 보드용으로 빌드할지 고르는 곳. **여기 한 줄만 바꾸면 된다.**
  *
  *   BOARD_SELECT 를 BOARD_STM1 또는 BOARD_STM2 로 두고 다시 빌드한다.
  *
  * 두 보드는 같은 펌웨어다. 예전에는 STM1/ 과 STM2/ 로 프로젝트를 통째로
  * 복사해서 썼는데, 전체 diff 를 떠 보니 실제로 다른 것은 파일 2개의 4줄
  * 뿐이었다. 그 4줄 때문에 428개 파일이 두 벌 있었던 셈이다.
  *
  * 복사본이 위험한 이유는 용량이 아니라 **조용히 갈라진다는 것**이다.
  * 한쪽에서 버그를 고치고 다른 쪽을 잊어도 빌드는 멀쩡히 통과하고, 나중에
  * "왜 STM2 만 이러지"를 처음부터 다시 디버깅하게 된다. 실제로 리뷰에서
  * 홀 하트비트/snprintf 관련 버그 3건이 나왔고, 복사본 구조였다면 전부
  * 두 번씩 고쳐야 했다.
  *
  * ---------------------------------------------------------------------------
  * 굽기 전에 확인하는 법
  *
  * 부팅 배너 첫 줄에 여기서 고른 보드가 찍힌다.
  *
  *     # STM2  build Aug 14 2026 16:20:11
  *     # payload v1.1 (frame ver 0x02) / node STM2
  *
  * 잘못 구웠으면 여기서 바로 보인다. 두 보드를 연달아 구울 때 이 줄을
  * 확인하지 않으면, 같은 주소로 송신하는 두 노드를 만들어 놓고 링크가
  * 왜 이상한지 한참 찾게 된다.
  ******************************************************************************
  */

#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#define BOARD_STM1  1
#define BOARD_STM2  2

/* ===========================================================================
 *
 *      ★ 바꿀 곳은 여기 한 줄뿐이다 ★
 *
 * =========================================================================== */
#ifndef BOARD_SELECT
#define BOARD_SELECT  BOARD_STM1
#endif
/* ========================================================================= */


/* --- 아래는 위 선택에서 자동으로 따라온다. 손댈 필요 없다. --------------- */

#if   BOARD_SELECT == BOARD_STM1
  /* 홀 2 + LED 2 + 화재 1 */
  #define BOARD_NODE_ID     "STM1"
  #define BOARD_HALL_COUNT  2u
  #define BOARD_HALL_BASE   1u    /* -> HALL01, HALL02 */
  #define BOARD_HAS_FLAME   1

#elif BOARD_SELECT == BOARD_STM2
  /* 홀 2 + LED 2 */
  #define BOARD_NODE_ID     "STM2"
  #define BOARD_HALL_COUNT  2u
  #define BOARD_HALL_BASE   3u    /* -> HALL03, HALL04 */
  #define BOARD_HAS_FLAME   0

#else
  #error "BOARD_SELECT 가 BOARD_STM1 / BOARD_STM2 중 하나가 아니다. board_config.h 를 확인할 것."
#endif

/*
 * 센서 ID 는 노드를 가로질러 전역 고유하게 매긴다 (HALL01~04).
 * 노드 이름으로 구분하는 방식(양쪽 다 HALL01/02)도 가능하지만 그러지
 * 않았다. Pi 의 config/parking_slots.json 이 이미 HALL01~04 를 전역 고유
 * 키로 쓰고 있어서, 전역 고유로 두면 Pi 쪽 매핑을 손대지 않아도 된다.
 *
 * 그래서 핀 라벨과 센서 번호가 어긋난다 -- STM2 에서 HALL1_D0(PA8) 이
 * HALL03 이다. 핀 라벨은 보드 기준(이 보드의 1번), 센서 ID 는 주차장
 * 전체 기준(3번 자리)이라 그렇다.
 *
 * 보드를 하나 더 늘리려면 위 #elif 블록을 복사해 BOARD_HALL_BASE 를
 * 5u 로 두고, lora_e22.h 의 주소 표에 STM3 을 추가하면 된다.
 */

#endif /* __BOARD_CONFIG_H */

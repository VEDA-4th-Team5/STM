#ifndef __ALERT_COMMAND_H
#define __ALERT_COMMAND_H

#include <stdint.h>   /* AlertCommand_SubmitLine 의 uint16_t */

/* Receives the Pi's AlertCommand lines over USART2 and drives the four
 * plate-illumination LEDs (see docs/UART_LORA_PROTOCOL.md's "번호판 조명 LED
 * 명령" section and docs/STM1_plate_led_design_EVDA-194.md).
 *
 * Wire format (one line per command, CRLF terminated):
 *   ALERT:HALL01:LED:ON:12
 *   ALERT:HALL01:LED:OFF:12
 * The sensor id is one of HALL01..HALL04 (same slot mapping sensor_protocol.c
 * uses on the way out). Everything after the ON/OFF token (the sequence) is
 * accepted but not interpreted -- there's no ACK channel, so STM32 has
 * nothing to reply with sequence-matched anyway.
 *
 * No task/ISR calls HAL_GPIO_WritePin directly from outside this file --
 * that keeps the "already on / already off is a no-op, unknown id is
 * ignored" rules in one place. */

/* Creates the per-LED failsafe timers. Call once from MX_FREERTOS_Init(),
 * after osKernelInitialize() and before osKernelStart() -- same spot the
 * project's other osTimerNew() calls live (freertos.c, RTOS_TIMERS). */
void AlertCommand_Init(void);

/* Arms the first HAL_UART_Receive_IT byte. Call once from main.c's
 * USER CODE BEGIN RTOS_PERIPH_START, after MX_USART2_UART_Init() and after
 * AlertCommand_Init() (the failsafe timers must exist before a line can
 * arrive and try to start one). */
void AlertCommand_StartReceive(void);

/* Called from HAL_UART_RxCpltCallback (main.c, ISR context) with the byte
 * that was just received. Appends it to the line buffer, re-arms the next
 * single-byte receive, and on a completed line releases the semaphore
 * AlertCommand_Task() blocks on. Kept to buffer-append + semaphore-release
 * only -- no parsing, no GPIO, no timer calls -- because osTimerStart isn't
 * guaranteed ISR-safe in this project's CMSIS-RTOS2/FreeRTOS wrapper (the
 * existing ISR callbacks in main.c only ever call osSemaphoreRelease for the
 * same reason). */
void AlertCommand_OnByteReceived(void);

/*
 * 완성된 명령 한 줄을 파서에 밀어 넣는다. USART2 바이트 조립을 거치지 않는
 * 두 번째 입력원(LoRa)용이다.
 *
 * 두 경로를 다 지원하는 이유:
 *   - LoRa   : Pi 가 실제로 붙는 경로. USART2(PA2/PA3)는 ST-Link 가상 COM
 *              포트라 Pi 가 물릴 수 없다.
 *   - USART2 : PuTTY 로 명령을 손으로 넣어 시험할 수 있다. 무선을 걷어내고
 *              LED 로직만 따로 검증할 때 유용하다.
 *
 * LoRa 프레임은 길이 필드가 있어 CRLF 를 싣지 않으므로, 바이트 단위 조립
 * 대신 완성된 줄을 그대로 넘긴다.
 *
 * ISR 에서 불러도 안전하다(내부에서 짧게 인터럽트를 막고 복사한다).
 */
void AlertCommand_SubmitLine(const char *line, uint16_t len);

/* TaskCommandRX's body (freertos.c). Blocks on the line-ready semaphore,
 * then parses and acts on one line per wake: GPIO writes and
 * osTimerStart/osTimerStop all happen here, in task context. Never
 * returns. */
void AlertCommand_Task(void);

#endif /* __ALERT_COMMAND_H */

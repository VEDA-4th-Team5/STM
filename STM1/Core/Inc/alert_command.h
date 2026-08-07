#ifndef __ALERT_COMMAND_H
#define __ALERT_COMMAND_H

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

/* TaskCommandRX's body (freertos.c). Blocks on the line-ready semaphore,
 * then parses and acts on one line per wake: GPIO writes and
 * osTimerStart/osTimerStop all happen here, in task context. Never
 * returns. */
void AlertCommand_Task(void);

#endif /* __ALERT_COMMAND_H */

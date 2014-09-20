/*
	Copyright 2012-2014 Benjamin Vedder	benjamin@vedder.se

	This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
    */

/*
 * app_uartcomm.c
 *
 *  Created on: 2 jul 2014
 *      Author: benjamin
 */

#include "app.h"
#include "ch.h"
#include "hal.h"
#include "hw.h"
#include "mcpwm.h"
#include "packet.h"
#include "commands.h"

#include <string.h>

// Settings
#define BAUDRATE					115200
#define PACKET_HANDLER				1
#define SERIAL_RX_BUFFER_SIZE		1024

// Threads
static msg_t uart_thread(void *arg);
static msg_t packet_process_thread(void *arg);
static WORKING_AREA(uart_thread_wa, 1024);
static WORKING_AREA(packet_process_thread_wa, 4096);
static Thread *process_tp;

// Variables
static volatile systime_t last_uart_update_time;
static volatile systime_t timeout_msec = 1000;
static uint8_t serial_rx_buffer[SERIAL_RX_BUFFER_SIZE];
static int serial_rx_read_pos = 0;
static int serial_rx_write_pos = 0;
static int is_running = 0;

// Private functions
static void process_packet(unsigned char *data, unsigned char len);
static void send_packet_wrapper(unsigned char *data, unsigned char len);
static void send_packet(unsigned char *data, unsigned char len);

/*
 * This callback is invoked when a transmission buffer has been completely
 * read by the driver.
 */
static void txend1(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked when a transmission has physically completed.
 */
static void txend2(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * This callback is invoked on a receive error, the errors mask is passed
 * as parameter.
 */
static void rxerr(UARTDriver *uartp, uartflags_t e) {
	(void)uartp;
	(void)e;
}

/*
 * This callback is invoked when a character is received but the application
 * was not ready to receive it, the character is passed as parameter.
 */
static void rxchar(UARTDriver *uartp, uint16_t c) {
	(void)uartp;
	serial_rx_buffer[serial_rx_write_pos++] = c;

	if (serial_rx_write_pos == SERIAL_RX_BUFFER_SIZE) {
		serial_rx_write_pos = 0;
	}

	chEvtSignal(process_tp, (eventmask_t) 1);
}

/*
 * This callback is invoked when a receive buffer has been completely written.
 */
static void rxend(UARTDriver *uartp) {
	(void)uartp;
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
		txend1,
		txend2,
		rxend,
		rxchar,
		rxerr,
		BAUDRATE,
		0,
		USART_CR2_LINEN,
		0
};

static void process_packet(unsigned char *data, unsigned char len) {
	commands_set_send_func(send_packet_wrapper);
	commands_process_packet(data, len);
	last_uart_update_time = chTimeNow();
}

static void send_packet_wrapper(unsigned char *data, unsigned char len) {
	packet_send_packet(data, len, PACKET_HANDLER);
}

static void send_packet(unsigned char *data, unsigned char len) {
	// Wait for the previous transmission to finish.
	while (HW_UART_DEV.txstate == UART_TX_ACTIVE) {
		chThdSleep(1);
	}

	// Copy this data to a new buffer in case the provided one is re-used
	// after this function returns.
	static uint8_t buffer[300];
	memcpy(buffer, data, len);

	uartStartSend(&HW_UART_DEV, len, buffer);
}

void app_uartcomm_start(void) {
	packet_init(send_packet, process_packet, PACKET_HANDLER);
	chThdCreateStatic(uart_thread_wa, sizeof(uart_thread_wa), NORMALPRIO - 1, uart_thread, NULL);
	chThdCreateStatic(packet_process_thread_wa, sizeof(packet_process_thread_wa), NORMALPRIO, packet_process_thread, NULL);
}

void app_uartcomm_configure(uint32_t baudrate, uint32_t timeout) {
	uart_cfg.speed = baudrate;
	timeout_msec = timeout;

	if (is_running) {
		uartStart(&HW_UART_DEV, &uart_cfg);
	}
}

static msg_t uart_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("UARTCOMM");

	uartStart(&HW_UART_DEV, &uart_cfg);
	palSetPadMode(HW_UART_TX_PORT, HW_UART_TX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);
	palSetPadMode(HW_UART_RX_PORT, HW_UART_RX_PIN, PAL_MODE_ALTERNATE(HW_UART_GPIO_AF) |
			PAL_STM32_OSPEED_HIGHEST |
			PAL_STM32_PUDR_PULLUP);

	is_running = 1;

	systime_t time = chTimeNow();

	for(;;) {
		time += MS2ST(40);

		if (timeout_msec != 0 && chTimeElapsedSince(last_uart_update_time) > MS2ST(timeout_msec)) {
			mcpwm_release_motor();
		}

		chThdSleepUntil(time);
	}

	return 0;
}

static msg_t packet_process_thread(void *arg) {
	(void)arg;

	chRegSetThreadName("uartcomm process");

	process_tp = chThdSelf();

	for(;;) {
		chEvtWaitAny((eventmask_t) 1);

		while (serial_rx_read_pos != serial_rx_write_pos) {
			packet_process_byte(serial_rx_buffer[serial_rx_read_pos++], PACKET_HANDLER);

			if (serial_rx_read_pos == SERIAL_RX_BUFFER_SIZE) {
				serial_rx_read_pos = 0;
			}
		}
	}

	return 0;
}
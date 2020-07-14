/*
 * Copyright (C) 2020 Nishchay Agrawal
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sps30_uart
 * @brief       UART Device driver for the Sensirion SPS30 particulate matter sensor
 * @author      Nishchay Agrawal <f2016088@pilani.bits-pilani.ac.in>
 * @file
 */
#include <string.h>
#include <stdio.h>

#include "xtimer.h"
#include "od.h"

#include "sps30_uart.h"

#define ENABLE_DEBUG	(1)
#include "debug.h"

#define SPS30_UART_DEV			(dev->params.uart_dev)
#define SPS30_UART_BAUD_RATE	(dev->params.baud_rate)
#define SPS30_UART_ADDRESS		(dev->params.uart_addr)	

static void _receive_callback(void *arg, uint8_t data);
static int _send_rcv_cmd (sps30_uart_t *dev, uint8_t *send_buf, size_t send_len, uint8_t *recv_buf, size_t *recv_len);
static int _is_valid_checksum (uint8_t *data, size_t len);

int sps30_uart_init(sps30_uart_t *dev, sps30_uart_params_t *params)
{
	dev->params = *params;

	mutex_init(&dev->dev_lock);
	mutex_init(&dev->cb_lock);

	int ret = uart_init(SPS30_UART_DEV, SPS30_UART_BAUD_RATE, _receive_callback, dev);

	/* Just for testing purposes rightnow, needs to be replaced with soft reset once everything starts working */
	ret = sps30_uart_send_cmd(dev, SPS30_UART_RESET);
	DEBUG("[sps30_uart_init] Init done.\n");
	return ret;
}

int sps30_uart_send_cmd(sps30_uart_t *dev, uint8_t cmd)
{
	uint8_t checksum = ~(SPS30_UART_ADDRESS + cmd);
	/* TODO: send data to device commands */
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, cmd, 0, checksum, SPS30_UART_FRAME_TAIL};
	uint8_t recv_buf[SPS30_UART_MAX_BUF_LEN];
	size_t recv_len = 0;

	// DEBUG("[sps30_uart_send_cmd] will try to send \n");
	// od_hex_dump(send_cmd, sizeof(send_cmd), OD_WIDTH_DEFAULT);
	int ret = _send_rcv_cmd(dev, send_cmd, sizeof(send_cmd), recv_buf, &recv_len);
	DEBUG("[sps30_uart_send_cmd] Received data of length %d\n", recv_len);
	od_hex_dump(recv_buf, recv_len, OD_WIDTH_DEFAULT);
	return ret;
}

int sps30_uart_start_measurement(sps30_uart_t *dev)
{
	uint8_t checksum = ~(SPS30_UART_ADDRESS + SPS30_UART_START + 2 + 0x01 + SPS30_UART_MEASURE_FLOAT);
	/* TODO: send data to device commands */
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_START, 2, 0x01, SPS30_UART_MEASURE_FLOAT, checksum, SPS30_UART_FRAME_TAIL};
	uint8_t recv_buf[SPS30_UART_MAX_BUF_LEN];
	size_t recv_len = 0;

	// od_hex_dump(send_cmd, sizeof(send_cmd), OD_WIDTH_DEFAULT);
	int ret = _send_rcv_cmd(dev, send_cmd, sizeof(send_cmd), recv_buf, &recv_len);
	od_hex_dump(recv_buf, recv_len, OD_WIDTH_DEFAULT);
	return ret;
}

int sps30_uart_read_measurement(sps30_uart_t *dev)
{
	(void) dev;
	return 0;
}


static void _receive_callback(void *arg, uint8_t data)
{
	sps30_uart_t *dev = (sps30_uart_t *)arg;

	/* Check is valid frame head or not at end of receive buffer */
	if ((dev->pos == 0 && data != SPS30_UART_FRAME_HEAD) ||
		dev->pos == SPS30_UART_MAX_BUF_LEN) {
		return ;
	}
	dev->rcv_buf[dev->pos] = data;

	if (dev->pos > 0 && data == SPS30_UART_FRAME_TAIL) {
		mutex_unlock(&dev->cb_lock);
	}

	dev->pos++;
	return ;
}

static int _send_rcv_cmd (sps30_uart_t *dev, uint8_t *send_buf, size_t send_len, uint8_t *recv_buf, size_t *recv_len)
{
	// DEBUG("[sps30_uart_snd_rcv_cmd] _send_rcv_cmd\n");
	mutex_lock(&dev->dev_lock);
	// DEBUG("[sps30_uart_snd_rcv_cmd] dev_lock obtained.\n");
	dev->pos = 0;
	/* Reset the buffer if filled from before */
	if (dev->rcv_buf[0] == SPS30_UART_FRAME_HEAD) {
		memset(dev->rcv_buf, 0, SPS30_UART_MAX_BUF_LEN);
	}

	mutex_lock(&dev->cb_lock);
	// DEBUG("[sps30_uart_snd_rcv_cmd] cb_lock obtained before write.\n");

	uart_write(SPS30_UART_DEV, send_buf, send_len);
	// DEBUG("[sps30_uart_snd_rcv_cmd] uart write complete.\n");

	mutex_lock(&dev->cb_lock);
	// DEBUG("[sps30_uart_snd_rcv_cmd] cb_lock obtained after write.\n");

	if (dev->pos != 0) {
		if (_is_valid_checksum(dev->rcv_buf, dev->pos) == SPS30_CRC_ERROR) {
			DEBUG("[sps30_uart_snd_rcv_cmd] checksum error\n");
			return SPS30_CRC_ERROR;
		}
		DEBUG("[sps30_uart_snd_rcv_cmd] Received data of length %d\n", dev->pos);
		od_hex_dump(dev->rcv_buf,  dev->pos, OD_WIDTH_DEFAULT);
		memcpy(recv_buf, dev->rcv_buf, dev->pos);
		*recv_len = dev->pos;
		dev->pos = 0;
		mutex_unlock(&dev->cb_lock);
	}

	mutex_unlock(&dev->dev_lock);

	return SPS30_OK;
}

static int _is_valid_checksum (uint8_t *data, size_t len)
{
	uint8_t checksum = 0;
	for (size_t i = 1 ; i < (len-1); i++) {
		checksum += data[i];
	}
	return checksum == 0xFF ? SPS30_OK : SPS30_CRC_ERROR;
}
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
#include "byteorder.h"

#include "sps30_uart.h"

#define ENABLE_DEBUG	(1)
#include "debug.h"

#define SPS30_UART_DEV			(dev->params.uart_dev)
#define SPS30_UART_BAUD_RATE	(dev->params.baud_rate)
#define SPS30_UART_ADDRESS		(dev->params.uart_addr)

#define SPS30_UART_DATA_IDX(X) 	(SPS30_UART_RCV_DATA_IDX + X)

static void _receive_callback(void *arg, uint8_t data);
static int _send_cmd (sps30_uart_t *dev, uint8_t *send_buf, size_t send_len);
static int _rcv_cmd	(sps30_uart_t *dev, uint8_t *recv_buf, size_t *recv_len);
static size_t _get_num_stuff_bytes(uint8_t *buf, int len);
static int _stuff_bytes(uint8_t *init_buf, int len, int stuff_bytes, uint8_t *buf);
static uint8_t _insert_checksum (uint8_t *data, size_t len);
static bool _requires_byte_stuffing (uint8_t data);
static int _is_valid_checksum (uint8_t *data, size_t len);
static inline float _reinterpret_float(uint32_t raw_val);
static float _raw_val_to_float(const uint8_t *buffer);

int sps30_uart_init(sps30_uart_t *dev, const sps30_uart_params_t *params)
{
	dev->params = *params;

	mutex_init(&dev->dev_lock);
	mutex_init(&dev->cb_lock);

	int ret = uart_init(SPS30_UART_DEV, SPS30_UART_BAUD_RATE, _receive_callback, dev);
	ret = sps30_uart_reset(dev);
	dev->state = IDLE_MODE;

	return ret;
}

int sps30_uart_send_cmd(sps30_uart_t *dev, uint8_t cmd)
{
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, cmd, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	return ret;
}

int sps30_uart_start_measurement(sps30_uart_t *dev)
{
	if (dev->state != IDLE_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_START, 2, SPS30_UART_SUBCOMMAND_MEASURE, SPS30_UART_MEASURE_FLOAT, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	dev->state = MEASUREMENT_MODE;
	return ret;
}

int sps30_uart_stop_measurement(sps30_uart_t *dev)
{
	if (dev->state != MEASUREMENT_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_STOP, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	dev->state = IDLE_MODE;
	return ret;
}

int sps30_uart_read_measurement(sps30_uart_t *dev, sps30_uart_data_t *result)
{
	if (dev->state != MEASUREMENT_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_READ, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}

	result->mc_pm1   = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(0)]);
    result->mc_pm2_5 = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(4)]);
    result->mc_pm4   = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(8)]);
    result->mc_pm10  = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(12)]);
    result->nc_pm0_5 = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(16)]);
    result->nc_pm1   = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(20)]);
    result->nc_pm2_5 = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(24)]);
    result->nc_pm4   = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(28)]);
    result->nc_pm10  = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(32)]);
   	result->ps       = _raw_val_to_float(&recv_buf[SPS30_UART_DATA_IDX(36)]);

	return ret;
}

int sps30_uart_read_ac_interval(sps30_uart_t *dev, uint32_t *seconds)
{
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_AUTO_CLEAN_FUNC, 1, SPS30_UART_SUBCOMMAND_AUTOCLEAN, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}

	*seconds = byteorder_bebuftohl(recv_buf+SPS30_UART_RCV_DATA_IDX);

	return ret;
}

int sps30_uart_write_ac_interval(sps30_uart_t *dev, uint32_t seconds)
{
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_AUTO_CLEAN_FUNC, 5, SPS30_UART_SUBCOMMAND_AUTOCLEAN, 0, 0, 0, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	byteorder_htobebufl(send_cmd + 5, seconds);
	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	return ret;
}

int sps30_uart_clean_fan(sps30_uart_t *dev)
{
	if(dev->state != MEASUREMENT_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_CLEAN_FAN, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}

	/* Wait for fan cleaning to complete */
	xtimer_sleep(11);
	return ret;
}

/* Expects a preallocated string(max len 32 bytes) to be sent to function */
int sps30_uart_read_product_type(sps30_uart_t *dev, char *str, size_t *len)
{
	(void) str;
	(void) len;
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_DEV_INFO, 1, SPS30_UART_DEV_INFO_PRODUCT, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	
	if (recv_buf[4] != 0) {
		*len = recv_buf[4];
		memcpy(str, (char *)(recv_buf + SPS30_UART_RCV_DATA_IDX), *len);
	}
	return SPS30_OK;
}

/* Expects a preallocated string(max len 32 bytes) to be sent to function */
int sps30_uart_read_serial_number(sps30_uart_t *dev, char *str, size_t *len)
{
	(void ) str;
	(void ) len;
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_DEV_INFO, 1, SPS30_UART_DEV_INFO_SERIAL, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	
	if (recv_buf[4] != 0) {
		*len = recv_buf[4];
		memcpy(str, (char *)(recv_buf + SPS30_UART_RCV_DATA_IDX), *len);
	}

	return SPS30_OK;
}

/* Expects a preallocated string(max len 7 bytes) to be sent to function */
int sps30_uart_read_version(sps30_uart_t *dev, char *str, size_t *len)
{
	(void) str;
	(void) len;	
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_READ_VERSION, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	
	if (recv_buf[4] != 0) {
		*len = recv_buf[4];
		memcpy(str, (char *)(recv_buf + SPS30_UART_RCV_DATA_IDX), *len);
	}

	return SPS30_OK;
}

int sps30_uart_read_status_reg(sps30_uart_t *dev, bool clear_reg, uint32_t *status_register)
{
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_STATUS_REG, 1, clear_reg?1:0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}

	*status_register = byteorder_bebuftohl(recv_buf + SPS30_UART_RCV_DATA_IDX);
	return SPS30_OK;
}

int sps30_uart_reset(sps30_uart_t *dev)
{
	DEBUG("inside reset\n");
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_RESET, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	dev->state = IDLE_MODE;
	return ret;
}

int sps30_uart_sleep(sps30_uart_t *dev)
{
	if (dev->state != IDLE_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_SLEEP, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}
	dev->state = SLEEP_MODE;

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	dev->state = SLEEP_MODE;
	return ret;
}

int sps30_uart_wake(sps30_uart_t *dev)
{
	if (dev->state != SLEEP_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_WAKE, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, sizeof(send_cmd));

	int ret = _send_cmd(dev, send_cmd, sizeof(send_cmd));
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}
	dev->state = IDLE_MODE;

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	dev->state = IDLE_MODE;
	return ret;
}

/* Took care of byte stuffing in incoming bytes here only */
static void _receive_callback(void *arg, uint8_t data)
{
	sps30_uart_t *dev = (sps30_uart_t *)arg;

	/* Check is valid frame head or not at end of receive buffer */
	if ((dev->pos == 0 && data != SPS30_UART_FRAME_HEAD) ||
		dev->pos == SPS30_UART_MAX_BUF_LEN) {
		return ;
	}

	/* Remove byte stuffing from received data */
	if (data == 0x5E && dev->rcv_buf[dev->pos-1] == 0x7D) {
		dev->pos--;
		dev->rcv_buf[dev->pos] = 0x7E;
	}
	else if (data == 0x5D && dev->rcv_buf[dev->pos-1] == 0x7D) {
		dev->pos--;
		dev->rcv_buf[dev->pos] = 0x7D;
	}
	else if (data == 0x31 && dev->rcv_buf[dev->pos-1] == 0x7D) {
		dev->pos--;
		dev->rcv_buf[dev->pos] = 0x11;
	}
	else if (data == 0x33 && dev->rcv_buf[dev->pos-1] == 0x7D) {
		dev->pos--;
		dev->rcv_buf[dev->pos] = 0x13;
	}
	else {
		dev->rcv_buf[dev->pos] = data;
	}

	/* Think of a way of preventing deadlock if the expected response doesn't arrive from the device */
	if (dev->pos > 0 && data == SPS30_UART_FRAME_TAIL) {
		mutex_unlock(&dev->cb_lock);
	}

	dev->pos++;
	return ;
}

static int _send_cmd (sps30_uart_t *dev, uint8_t *send_buf, size_t send_len)
{
	size_t num_stuff_bytes = _get_num_stuff_bytes(send_buf, send_len);
	if (num_stuff_bytes > 0) {
		uint8_t _temp_send_cmd_buf[send_len + num_stuff_bytes];
		_stuff_bytes(send_buf, send_len, num_stuff_bytes,_temp_send_cmd_buf);
		send_len = send_len + num_stuff_bytes;
		mutex_lock(&dev->dev_lock);
		dev->pos = 0;
		/* Reset the buffer if filled from before */
		if (dev->rcv_buf[0] == SPS30_UART_FRAME_HEAD) {
			memset(dev->rcv_buf, 0, SPS30_UART_MAX_BUF_LEN);
		}

		mutex_lock(&dev->cb_lock);

		if (_temp_send_cmd_buf[2] == SPS30_UART_WAKE) {
			uint8_t wake_if[] = {0xFF}; 
			uart_write(SPS30_UART_DEV, wake_if, sizeof(wake_if));	
		}
		uart_write(SPS30_UART_DEV, _temp_send_cmd_buf, send_len);
		/* Obtaining lock to indicate receiving data complete in response to sent command */
		mutex_lock(&dev->cb_lock);
	}
	else {
		mutex_lock(&dev->dev_lock);
		dev->pos = 0;
		/* Reset the buffer if filled from before */
		if (dev->rcv_buf[0] == SPS30_UART_FRAME_HEAD) {
			memset(dev->rcv_buf, 0, SPS30_UART_MAX_BUF_LEN);
		}

		mutex_lock(&dev->cb_lock);

		if (send_buf[2] == SPS30_UART_WAKE) {
			uint8_t wake_if[] = {0xFF}; 
			uart_write(SPS30_UART_DEV, wake_if, sizeof(wake_if));	
		}
		uart_write(SPS30_UART_DEV, send_buf, send_len);
		/* Obtaining lock to indicate receiving data complete in response to sent command */
		mutex_lock(&dev->cb_lock);
	}
	return SPS30_OK;
}

/* Separate function to allocate only those many bytes as required for the data received */
static int _rcv_cmd (sps30_uart_t *dev, uint8_t *recv_buf, size_t *recv_len)
{
	if (dev->pos != 0) {
		if (_is_valid_checksum(dev->rcv_buf, dev->pos) == SPS30_CRC_ERROR) {
			DEBUG("[sps30_uart_rcv_cmd] checksum error\n");
			dev->pos = 0;
			mutex_unlock(&dev->cb_lock);
			mutex_unlock(&dev->dev_lock);
			return SPS30_CRC_ERROR;
		}
		if (recv_buf != NULL) {
			/* Think about just copying the data part since all other parts are anyways irrelevant to us */
			memcpy(recv_buf, dev->rcv_buf, dev->pos);
			*recv_len = dev->pos;
		}
		dev->pos = 0;
		mutex_unlock(&dev->cb_lock);
	}

	mutex_unlock(&dev->dev_lock);
	/* Wait 10ms before next command can be sent */
	xtimer_usleep(10000);

	return SPS30_OK;
}

static size_t _get_num_stuff_bytes(uint8_t *buf, int len)
{
	int num_byte_stuffs = 0;

	/* Calculate number of byte stuffings required */
	for (int i = 1; i < len-1; i++) {
		if (_requires_byte_stuffing(buf[i])) {
			num_byte_stuffs++;
		}
	}
	return num_byte_stuffs;
}

/* 
 *	Handles Byte-stuffing before sending command to sensor
 *
 *	Returns final length of array after byte stuffing
 */
static int _stuff_bytes(uint8_t *init_buf, int len, int stuff_bytes, uint8_t *buf)
{
	buf[len-1+stuff_bytes] = init_buf[len-1];
	buf[0] = init_buf[0];
	for (int i = len-2; i >= 1; i--) {
		switch(init_buf[i]) {
			case 0x7E:
				stuff_bytes--;
				buf[i+stuff_bytes] = 0x7D;
				buf[i+stuff_bytes+1] = 0x5E;
				break;
			case 0x7D:
				stuff_bytes--;
				buf[i+stuff_bytes] = 0x7D;
				buf[i+stuff_bytes+1] = 0x5D;
				break;
			case 0x11:
				stuff_bytes--;
				buf[i+stuff_bytes] = 0x7D;
				buf[i+stuff_bytes+1] = 0x31;
				break;
			case 0x13:
				stuff_bytes--;
				buf[i+stuff_bytes] = 0x7D;
				buf[i+stuff_bytes+1] = 0x33;
				break;
			default:
				buf[i+stuff_bytes] = init_buf[i];
				break;
		}
	}
	return 0;
}

/* Return true if byte stuffing was required */
static uint8_t _insert_checksum (uint8_t *data, size_t len)
{
	uint8_t checksum = data[len-2];
	for(size_t i = 1; i < len - 1u; i++) {
		checksum += data[i];
	}
	data[len-2] = ~checksum;
	return 0;
}

static bool _requires_byte_stuffing (uint8_t data)
{
	return (data == 0x7E || data == 0x7D || data == 0x11 || data == 0x13)?true:false;
}

static int _is_valid_checksum (uint8_t *data, size_t len)
{
	uint8_t checksum = 0;
	for (size_t i = 1 ; i < len-1u; i++) {
		checksum += data[i];
	}
	return checksum == 0xFF ? SPS30_OK : SPS30_CRC_ERROR;
}

/**
 *      convert IEEE754 stored in uint32_t to actual float value
 *
 *      @param  raw_val   Value to convert
 *      @return           Converted value
 */
static inline float _reinterpret_float(uint32_t raw_val)
{
    union {
        float float_val;
        uint32_t int_val;
    } to_float = { .int_val = raw_val };

    return to_float.float_val;
}

/**
 *      convert value from buffer storing IEEE754 represented
 *      float to actual float value
 *
 *      @param  buffer   buffer with value
 *      @return          Converted value
 */
static float _raw_val_to_float(const uint8_t *buffer)
{
    uint32_t tmp = byteorder_bebuftohl(buffer);

    return _reinterpret_float(tmp);
}
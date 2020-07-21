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

#define SPS30_UART_CMD_SIZE		(6 + send_cmd[3])

static void _receive_callback(void *arg, uint8_t data);
static int _send_cmd (sps30_uart_t *dev, uint8_t *send_buf, size_t send_len);
static int _rcv_cmd	(sps30_uart_t *dev, uint8_t *recv_buf, size_t *recv_len);
static size_t _preprocess_send_buf (uint8_t *buf, int len);
// static bool _check_copy_from_rcv_buf (uint8_t *data);
static uint8_t _insert_checksum (uint8_t *data, size_t len);
static bool _requires_byte_stuffing (uint8_t data);
static int _is_valid_checksum (uint8_t *data, size_t len);

gpio_t test_pin = GPIO_PIN(PORT_B, 2);

int sps30_uart_init(sps30_uart_t *dev, const sps30_uart_params_t *params)
{
	dev->params = *params;

	mutex_init(&dev->dev_lock);
	mutex_init(&dev->cb_lock);

	gpio_init(test_pin, GPIO_OUT);
	gpio_set(test_pin);
	int ret = uart_init(SPS30_UART_DEV, SPS30_UART_BAUD_RATE, _receive_callback, dev);

	ret = sps30_uart_reset(dev);
	dev->state = IDLE_MODE;
	DEBUG("[sps30_uart_init] Init done.\n");

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
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_START, 2, SPS30_UART_SUBCOMMAND_MEASURE, SPS30_UART_MEASURE_FLOAT, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
	if (ret != SPS30_OK) {
		DEBUG("[sps30_uart] send wake command failed with error %d\n", ret);
		return ret;
	}
	dev->state = MEASUREMENT_MODE;

	uint8_t recv_buf[dev->pos];
	ret = _rcv_cmd(dev, recv_buf, &recv_len);
	if (recv_buf[SPS30_UART_RCV_STATE_IDX] != 0x00) {
		DEBUG("[sps30_uart] Received error from sensor, %02x\n", recv_buf[SPS30_UART_RCV_STATE_IDX]);
		return SPS30_UART_ERROR;
	}
	return ret;
}

int sps30_uart_stop_measurement(sps30_uart_t *dev)
{
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_STOP, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	return ret;
}

int sps30_uart_read_measurement(sps30_uart_t *dev, sps30_uart_data_t *result)
{
	(void) result;
	if (dev->state != MEASUREMENT_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_READ, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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

	/* Extract measurements and store in result struct */
	return ret;
}

int sps30_uart_read_ac_interval(sps30_uart_t *dev, uint32_t *seconds)
{
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_AUTO_CLEAN_FUNC, 1, SPS30_UART_SUBCOMMAND_AUTOCLEAN, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	//use ntohl() to send seconds instead of doing bits opeartions
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_AUTO_CLEAN_FUNC, 5, SPS30_UART_SUBCOMMAND_AUTOCLEAN, (seconds << 24 | 0x00), (seconds << 16 | 0x00),
							(seconds << 8 | 0x00), (seconds | 0x00), 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;
	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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

int sps30_uart_start_fan_clean(sps30_uart_t *dev)
{
	if(dev->state != MEASUREMENT_MODE) {
		DEBUG("[sps30_uart] Not allowed in this mode\n");
		return SPS30_UART_NOT_ALLOWED_IN_MODE;
	}
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_CLEAN_FAN, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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

int sps30_uart_read_product_type(sps30_uart_t *dev, char *str, size_t len)
{
	(void) str;
	(void) len;
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_DEV_INFO, 1, SPS30_UART_DEV_INFO_PRODUCT, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	od_hex_dump(recv_buf, recv_len, OD_WIDTH_DEFAULT);
	return SPS30_OK;
}

int sps30_uart_read_serial_number(sps30_uart_t *dev, char *str, size_t len)
{
	(void ) str;
	(void ) len;
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_DEV_INFO, 1, SPS30_UART_DEV_INFO_SERIAL, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	
	od_hex_dump(recv_buf, recv_len, OD_WIDTH_DEFAULT);
	return SPS30_OK;
}

int sps30_uart_read_version(sps30_uart_t *dev, char *str, size_t len)
{
	(void) str;
	(void) len;	
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_READ_VERSION, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	
	od_hex_dump(recv_buf, recv_len, OD_WIDTH_DEFAULT);
	return SPS30_OK;
}

int sps30_uart_read_status_reg(sps30_uart_t *dev, bool clear_reg)
{
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_STATUS_REG, 1, clear_reg?1:0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	od_hex_dump(recv_buf, recv_len, OD_WIDTH_DEFAULT);
	return SPS30_OK;
}

int sps30_uart_reset(sps30_uart_t *dev)
{
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_RESET, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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

int sps30_uart_sleep(sps30_uart_t *dev)
{
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_SLEEP, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	return ret;
}

int sps30_uart_wake(sps30_uart_t *dev)
{
	uint8_t send_cmd[SPS30_UART_MAX_CMD_LEN] = {SPS30_UART_FRAME_HEAD, SPS30_UART_ADDRESS, SPS30_UART_WAKE, 0, 0, SPS30_UART_FRAME_TAIL};
	size_t recv_len = 0;

	_insert_checksum(send_cmd, SPS30_UART_CMD_SIZE);

	int ret = _send_cmd(dev, send_cmd, SPS30_UART_CMD_SIZE);
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
	// gpio_toggle(test_pin);
	size_t stuffed_size = _preprocess_send_buf(send_buf, send_len);
	mutex_lock(&dev->dev_lock);
	// DEBUG("[sps30_uart_snd_rcv_cmd] dev_lock obtained.\n");
	dev->pos = 0;
	/* Reset the buffer if filled from before */
	if (dev->rcv_buf[0] == SPS30_UART_FRAME_HEAD) {
		memset(dev->rcv_buf, 0, SPS30_UART_MAX_BUF_LEN);
	}

	mutex_lock(&dev->cb_lock);
	// DEBUG("[sps30_uart_snd_cmd] cb_lock obtained before write.\n");

	if (send_buf[2] == SPS30_UART_WAKE) {
		uint8_t wake_if[] = {0xFF}; 
		uart_write(SPS30_UART_DEV, wake_if, sizeof(wake_if));	
	}
	gpio_toggle(test_pin);
	uart_write(SPS30_UART_DEV, send_buf, stuffed_size);
	// xtimer_usleep(4000);
	// DEBUG("[sps30_uart_snd_rcv_cmd] uart write complete.\n");
	/* Obtaining lock to indicate receiving data complete in response to sent command */
	// gpio_toggle(test_pin);
	mutex_lock(&dev->cb_lock);
	gpio_toggle(test_pin);
	// DEBUG("[sps30_uart_snd_rcv_cmd] cb_lock obtained after write.\n");
	return SPS30_OK;
}

/* Separate function to allocate only those many bytes as required for the data received */
static int _rcv_cmd (sps30_uart_t *dev, uint8_t *recv_buf, size_t *recv_len)
{
	// DEBUG("[sps30_uart_rcv_cmd] start\n");
	if (dev->pos != 0) {
		if (_is_valid_checksum(dev->rcv_buf, dev->pos) == SPS30_CRC_ERROR) {
			DEBUG("[sps30_uart_rcv_cmd] checksum error\n");
			dev->pos = 0;
			mutex_unlock(&dev->cb_lock);
			mutex_unlock(&dev->dev_lock);
			return SPS30_CRC_ERROR;
		}
		// DEBUG("[sps30_uart_rcv_cmd] Received data of length %d\n", dev->pos);
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

/* 
 *	Handles Byte-stuffing before sending command to sensor
 *
 *	Returns final length of array after byte stuffing
 *
 *	Note: Assumes that the buf is big enough to handle extra bytes of byte stuffing.
 * 	For now, this is handled by declaring the array of MAX_LEN
 */
static size_t _preprocess_send_buf(uint8_t *buf, int len)
{
	int num_byte_stuffs = 0, temp = 0;

	/* Calculate number of byte stuffings required */
	for (int i = 1; i < len-1; i++) {
		if (_requires_byte_stuffing(buf[i])) {
			num_byte_stuffs++;
		}
	}
	temp = num_byte_stuffs;
	buf[len-1+num_byte_stuffs] = buf[len-1];

	/* Move all elements ahead in one pass */
	for (int i = len-2; i >= 1; i--) {
		switch(buf[i]) {
			case 0x7E:
				temp--;
				buf[i+temp] = 0x7D;
				buf[i+temp+1] = 0x5E;
				break;
			case 0x7D:
				temp--;
				buf[i+temp] = 0x7D;
				buf[i+temp+1] = 0x5D;
				break;
			case 0x11:
				temp--;
				buf[i+temp] = 0x7D;
				buf[i+temp+1] = 0x31;
				break;
			case 0x13:
				temp--;
				buf[i+temp] = 0x7D;
				buf[i+temp+1] = 0x33;
				break;
			default:
				buf[i+temp] = buf[i];
				break;
		}
	}

	return (num_byte_stuffs+len);
}

// /* Checks if the data received needs to be copied to the receive buffer */
// static bool _check_copy_from_rcv_buf (uint8_t *data)
// {
// 	switch(data[2]) {
// 		case SPS30_UART_START:
//  		case SPS30_UART_STOP:
//  		case SPS30_UART_SLEEP:
//  		case SPS30_UART_WAKE:
//  		case SPS30_UART_CLEAN_FAN:
//  		case SPS30_UART_RESET:
//  			return false;	
//  		case SPS30_UART_READ:
//  		case SPS30_UART_AUTO_CLEAN_FUNC:
//  		case SPS30_UART_DEV_INFO:
//  		case SPS30_UART_READ_VERSION:
//  		case SPS30_UART_STATUS_REG:
//  			return true;
// 	}
// 	return true;
// }

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

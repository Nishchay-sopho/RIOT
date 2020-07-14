#ifndef SPS30_UART_H
#define SPS30_UART_H

#include "periph/uart.h"
#include "mutex.h"
#include "sps30_uart_internal.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uart_t uart_dev;
	uint32_t baud_rate;
	uint8_t uart_addr;
}sps30_uart_params_t;

typedef struct {
	mutex_t dev_lock;
	mutex_t cb_lock;
	uint8_t last_cmd;
	uint8_t rcv_buf[SPS30_UART_MAX_BUF_LEN];
	uint8_t pos;
	sps30_uart_params_t params;
}sps30_uart_t;

typedef struct {
    float mc_pm1;       /**< Mass concentration of PM 1.0 [µg/m^3] */
    float mc_pm2_5;     /**< Mass concentration of PM 2.5 [µg/m^3] */
    float mc_pm4;       /**< Mass concentration of PM 4.0 [µg/m^3] */
    float mc_pm10;      /**< Mass concentration of PM 10 [µg/m^3] */
    float nc_pm0_5;     /**< Number concentration of PM 0.5 [µg/m^3] */
    float nc_pm1;       /**< Number concentration of PM 1.0 [µg/m^3] */
    float nc_pm2_5;     /**< Number concentration of PM 2.5 [µg/m^3] */
    float nc_pm4;       /**< Number concentration of PM 4.0 [µg/m^3] */
    float nc_pm10;      /**< Number concentration of PM 10 [µg/m^3] */
    float ps;           /**< Typical particle size [µm] */
} sps30_uart_data_t;

typedef enum {
    SPS30_OK = 0,      /**< Everything went fine */
    SPS30_CRC_ERROR,   /**< The CRC check of received data failed */
    SPS30_NO_NEW_DATA, /**< No new data measurements found */
    SPS30_UART_ERROR   /**< Some UART operation failed */
} sps30_uart_error_code_t;

int sps30_uart_init(sps30_uart_t *dev, sps30_uart_params_t *params);
int sps30_uart_send_cmd(sps30_uart_t *dev, uint8_t cmd);
int sps30_uart_start_measurement(sps30_uart_t *dev);
int sps30_uart_read_measurement(sps30_uart_t *dev);

#ifdef __cplusplus
}
#endif

#endif
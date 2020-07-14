/*
 * Copyright (C) 2020 Nishchay Agrawal
 *
 * This file is subject to the terms and conditions of the GNU Lesser
 * General Public License v2.1. See the file LICENSE in the top level
 * directory for more details.
 */

/**
 * @ingroup     drivers_sensors
 * @{
 * @file
 * @brief       Device driver params interface for the SPS30 uart sensor.
 *
 * @author      Nishchay Agrawal <f2016088@pilani.bits-pilani.ac.in>
 * @}
 */
#ifndef SPS30_UART_PARAMS_H
#define SPS30_UART_PARAMS_H

#include "periph/uart.h"
#include "sps30_uart.h"

#include "saul_reg.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SPS30_UART_ADDR				(0x00)

#define SPS30_UART_BAUD_RATE 		 115200

#ifndef SPS30_PARAM_UART_DEV
#define SPS30_PARAM_UART_DEV         (UART_DEV(2))
#endif
#ifndef SPS30_PARAM_UART_ADDR
#define SPS30_PARAM_UART_ADDR        SPS30_UART_ADDR
#endif
#ifndef SPS30_PARAM_UART_BAUD_RATE
#define SPS30_PARAM_UART_BAUD_RATE   SPS30_UART_BAUD_RATE
#endif

#ifndef SPS30_UART_PARAMS
#define SPS30_UART_PARAMS            { .uart_dev = SPS30_PARAM_UART_DEV,   \
                                       .uart_addr = SPS30_PARAM_UART_ADDR, \
                                       .baud_rate = SPS30_PARAM_UART_BAUD_RATE }
#endif

#ifndef SPS30_UART_SAUL_INFO
#define SPS30_UART_SAUL_INFO             { .name = "sps30_uart" }
#endif

/**
 * @brief   Configure SPS30
 */
static const sps30_uart_params_t sps30_uart_params[] =
{
    SPS30_UART_PARAMS
};

/**
 * @brief   Get the number of configured SPS30 devices
 */
#define SPS30_UART_NUMOF       ARRAY_SIZE(sps30_uart_params)

/**
 * @brief   Configure SAUL registry entries
 */
static const saul_reg_info_t sps30_uart_saul_info[] =
{
    SPS30_UART_SAUL_INFO
};

#ifdef __cplusplus
}
#endif

#endif /* SPS30_UART_PARAMS_H */

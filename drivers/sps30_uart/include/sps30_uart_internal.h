#ifndef SPS30_UART_INTERNAL_H
#define SPS30_UART_INTERNAL_H

#ifdef __cplusplus
extern "C" {
#endif

#define SPS30_UART_MAX_BUF_LEN			(0x40)

#define SPS30_UART_RCV_STATE_IDX		(0x03)
#define SPS30_UART_RCV_DATA_IDX			(0x05)

#define SPS30_UART_FRAME_HEAD			(0x7E)
#define SPS30_UART_FRAME_TAIL			(0x7E)

#define SPS30_UART_START				(0x00)
#define SPS30_UART_STOP					(0x01)
#define SPS30_UART_READ					(0x03)
#define SPS30_UART_SLEEP				(0x10)
#define SPS30_UART_WAKE 				(0x11)
#define SPS30_UART_CLEAN_FAN			(0x56)
#define SPS30_UART_AUTO_CLEAN_FUNC		(0x80)
#define SPS30_UART_DEV_INFO				(0xD0)
#define SPS30_UART_READ_VERSION			(0xD1)
#define SPS30_UART_STATUS_REG			(0xD2)
#define SPS30_UART_RESET				(0xD3)

#define SPS30_UART_SUBCOMMAND_AUTOCLEAN	(0x00)
#define SPS30_UART_SUBCOMMAND_MEASURE	(0x01)

#define SPS30_UART_DEV_INFO_PRODUCT		(0x00)
#define SPS30_UART_DEV_INFO_SERIAL		(0x03)

#define SPS30_UART_MEASURE_FLOAT		(0x03)

#define IDLE_MODE 						(0x01)
#define SLEEP_MODE						(0x02)
#define MEASUREMENT_MODE				(0x03)

#ifdef __cplusplus
}
#endif

#endif
#include <stdio.h>
#include "xtimer.h"
#include "sps30_uart.h"
#include "sps30_uart_params.h"
#include "sps30_uart_internal.h"

sps30_uart_t sps30_dev;
sps30_uart_params_t sps30_params = SPS30_UART_PARAMS;

int main(void)
{
	printf("SPS30 UART test\n");
	int ret = -1;
	xtimer_sleep(3);
	printf("[test_sps30_uart] Initiating device\n");
	ret = sps30_uart_init(&sps30_dev, &sps30_params);
	printf("Output of init %d.\n", ret);

	ret = sps30_uart_start_measurement(&sps30_dev);
	printf("Output of measurement start %d\n", ret);

	ret = sps30_uart_send_cmd(&sps30_dev, SPS30_UART_CLEAN_FAN);
	printf("Output of start fan cleaning%d\n", ret);

	printf("[test_sps30_uart] Completed\n");
	return 0;
}


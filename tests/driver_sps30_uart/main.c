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

	ret = sps30_uart_read_ac_interval(&sps30_dev, NULL);
	printf("Output of read auto clean interval%d\n", ret);

	ret = sps30_uart_start_measurement(&sps30_dev);
	printf("Output of measurement start %d\n", ret);

	ret = sps30_uart_start_fan_clean(&sps30_dev);
	printf("Output of start fan cleaning%d\n", ret);

	xtimer_sleep(1);
	
	// ret = sps30_uart_stop_measurement(&sps30_dev);

	// xtimer_sleep(1);

	// ret = sps30_uart_start_measurement(&sps30_dev);

	// ret = sps30_uart_start_fan_clean(&sps30_dev);
	// printf("Output of start fan cleaning%d\n", ret);

	// ret = sps30_uart_stop_measurement(&sps30_dev);

	// printf("[test_sps30_uart] Completed\n");
	return 0;
}


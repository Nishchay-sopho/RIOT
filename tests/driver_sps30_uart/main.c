#include <stdio.h>
#include "xtimer.h"
#include "sps30_uart.h"
#include "sps30_uart_params.h"
#include "sps30_uart_internal.h"

#define AUTO_CLEAN_INTERVAL 	4800

#define TYPE_MC_STR  "MC PM"
#define TYPE_NC_STR  "NC PM"
#define TYPE_TPS_STR "TPS"
#define MC_UNIT_STR  "[µg/m³]"
#define NC_UNIT_STR  "[#/cm³]"
#define TPS_UNIT_STR "[µm]"

sps30_uart_t sps30_dev;
sps30_uart_params_t sps30_params = SPS30_UART_PARAMS;

static bool _print_error(const char *msg, sps30_uart_error_code_t ec)
{
    printf("sps30_%s: [%s]\n", msg, (ec == SPS30_OK) ? "OK" :
                              (ec == SPS30_CRC_ERROR ? "CRC_ERROR"
                                                     : "I2C_ERROR"));
    return ec != SPS30_OK;
}

static void _print_val_row(char *typ1, char *typ2, char *unit, float val)
{
    printf("| %-5s %4s:%3"PRIu32".%03"PRIu32" %-8s |\n", typ1, typ2,
           (uint32_t)val, ((uint32_t)((val + 0.0005) * 1000)) % 1000, unit);
}


int main(void)
{
	printf("SPS30 UART test\n");
	int ret = -1;
	uint32_t auto_clean_interval = 0;
	char dev_info[32];
	char version[7];
	size_t len = 0;
	uint32_t status_register = 0;
	bool error = false;
	sps30_uart_data_t result;

	xtimer_sleep(2);	

	ret = sps30_uart_init(&sps30_dev, &sps30_params);
	error |= _print_error("init", ret);

	xtimer_sleep(2);

	ret = sps30_uart_read_product_type(&sps30_dev, dev_info, &len);
	if (ret == SPS30_OK) {
        printf("Product type: %s\n", dev_info);
    } else {
        error |= _print_error("read_product_type", ret);
    }

	ret = sps30_uart_read_serial_number(&sps30_dev, dev_info, &len);
	if (ret == SPS30_OK) {
        printf("Serial: %s\n", dev_info);
    } else {
        error |= _print_error("read_serial_number", ret);
    }

	ret = sps30_uart_read_version(&sps30_dev, version, &len);
	if (ret == SPS30_OK) {
        printf("Version: %s\n", version);
    } else {
        error |= _print_error("read_version", ret);
    }

	ret = sps30_uart_read_status_reg(&sps30_dev, false, &status_register);
	if (ret == SPS30_OK) {
        printf("Status Register: %lu\n", status_register);
    } else {
        error |= _print_error("read_status_register", ret);
    }

	ret = sps30_uart_start_measurement(&sps30_dev);
	error |= _print_error("start_measurement", ret);

	for (int i = 0; i < 10; i++) {
		xtimer_sleep(1);

		ret = sps30_uart_read_measurement(&sps30_dev, &result);
		if (ret == SPS30_OK) {
	        puts("\nv==== SPS30 measurements ====v");
	        _print_val_row(TYPE_MC_STR, "1.0", MC_UNIT_STR, result.mc_pm1);
	        _print_val_row(TYPE_MC_STR, "2.5", MC_UNIT_STR, result.mc_pm2_5);
	        _print_val_row(TYPE_MC_STR, "4.0", MC_UNIT_STR, result.mc_pm4);
	        _print_val_row(TYPE_MC_STR, "10.0", MC_UNIT_STR, result.mc_pm10);
	        _print_val_row(TYPE_NC_STR, "0.5", NC_UNIT_STR, result.nc_pm0_5);
	        _print_val_row(TYPE_NC_STR, "1.0", NC_UNIT_STR, result.nc_pm1);
	        _print_val_row(TYPE_NC_STR, "2.5", NC_UNIT_STR, result.nc_pm2_5);
	        _print_val_row(TYPE_NC_STR, "4.0", NC_UNIT_STR, result.nc_pm4);
	        _print_val_row(TYPE_NC_STR, "10.0", NC_UNIT_STR, result.nc_pm10);
	        _print_val_row(TYPE_TPS_STR, "", TPS_UNIT_STR, result.ps);
	        puts("+----------------------------+");
	        puts("| MC:  Mass Concentration    |");
	        puts("| NC:  Number Concentration  |");
	        puts("| TPS: Typical Particle Size |");
	        printf("^===========%d/10============^\n\n", i+1);
		}
		else {
			error |= _print_error("read_measurement", ret);
		}
	}


	ret = sps30_uart_write_ac_interval(&sps30_dev, AUTO_CLEAN_INTERVAL);
	error |= _print_error("write_ac_interval", ret);

	ret = sps30_uart_read_ac_interval(&sps30_dev, &auto_clean_interval);
	error |= _print_error("read_ac_interval", ret);

	ret = sps30_uart_clean_fan(&sps30_dev);
	error |= _print_error("clean_fan", ret);
	
	ret = sps30_uart_stop_measurement(&sps30_dev);
	error |= _print_error("stop_measurement", ret);

	ret = sps30_uart_sleep(&sps30_dev);
	error |= _print_error("sleep", ret);

	/* To give timer to go into sleep state completely */
	xtimer_sleep(2);

	ret = sps30_uart_wake(&sps30_dev);
	error |= _print_error("wake", ret);

	if (error) {
        puts("sps30 test: [FAILED]");
    }
    else {
        puts("sps30 test: [SUCCESS]");
    }

	return 0;
}


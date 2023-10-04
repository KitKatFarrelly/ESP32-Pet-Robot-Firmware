#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

//project libraries
#include "LED_DRVR.h"
#include "IMU_SPI.h"
#include "ToF_I2C.h"
#include "MTR_DRVR.h"
//#include "UART_CMDS.h"

static const char *TAG = "APP LOG";

//TODO - Define Interrupt pins

#define ESP_INTR_FLAG_DEFAULT 0

//Misc GPIO Definitions

#define BAT_STAT GPIO_NUM_26
#define PWR_IND  GPIO_NUM_33
#define P_SENS   GPIO_NUM_1

void app_main(void)
{
    //GPIO Setup
	
	//TODO - Move
	
	//POWER INDICATOR INPUTS
	
	gpio_config_t io_conf = {};
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the battery stat
    io_conf.pin_bit_mask = (1ULL<<BAT_STAT);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
	
    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the battery stat
    io_conf.pin_bit_mask = (1ULL<<PWR_IND);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	//UART_INIT();
	
	//Initialize all components
	
	LED_INIT();
	
	MTR_INIT();
	
	TOF_INIT();
	
	IMU_INIT();

	//TODO: Setup for Flash and Console Commands (over USB)

	//TODO: Setup for ESP-NOW
	
}


/*
static void bringUpSuite(void)
{
	//test battery to make sure everything is running correctly
	
	int batteryStatus = 0;
	int powerGoodInd = 0;
	
	uint8_t cmd[3] = {0x80, 0x00, 0x00};
	uint8_t imu_dat[3];
	uint8_t cmd_size = 3;
	uint8_t imu_dat_size = 3;
	
	uint8_t tof_reg_addr = 0xE3;
	uint8_t tof_data = 0;
	uint8_t tof_len = 1;
	
	vTaskDelay(300 / portTICK_PERIOD_MS);
	
	lcd_cmd(spi, cmd, cmd_size, imu_dat, imu_dat_size);

	gpio_set_level(MTR_L_IN1, 0);
	gpio_set_level(MTR_L_IN2, 0);
	gpio_set_level(MTR_R_IN1, 0);
	gpio_set_level(MTR_R_IN2, 0);
	gpio_set_level(MTR_STBY, 1);
	for(uint8_t cnt = 0; cnt < 8; cnt++);
	{
		vTaskDelay(300 / portTICK_PERIOD_MS);
		batteryStatus = gpio_get_level(BAT_STAT);
		powerGoodInd = gpio_get_level(PWR_IND);
		//GPIO Test
        gpio_set_level(IND_LED_RED, batteryStatus);
        gpio_set_level(IND_LED_GREEN, !powerGoodInd);
		gpio_set_level(IND_LED_BLUE, cnt % 2);
		gpio_set_level(EMO_LED_RED, (cnt & 1) > 0);
        gpio_set_level(EMO_LED_GREEN, (cnt & 2) > 0);
		gpio_set_level(EMO_LED_BLUE, (cnt & 4) > 0);
		gpio_set_level(MTR_R_IN1, (cnt & 2) > 0);
		gpio_set_level(MTR_L_IN2, (cnt & 2) > 0);
		//SPI Test
		lcd_cmd(spi, cmd, cmd_size, imu_dat, imu_dat_size);
		//I2C Test
		i2c_master_write_read_device(I2C_MASTER_NUM, TOF_SENSOR_ADDR, &tof_reg_addr, 1, &tof_data, tof_len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
		tof_data = tof_data & 0x3F; //need to clear out bits 6 and 7
		ESP_LOGI(TAG, "TOF data is %x", tof_data);
		tof_data = 0;
    }
}
*/
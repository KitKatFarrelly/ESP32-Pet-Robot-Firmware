#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "ToF_I2C.h"
#include "tof_bin_image.h"

//I2C definitions

#define I2C_MASTER_SCL_IO           GPIO_NUM_16      /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           GPIO_NUM_17      /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

#define TOF_SENSOR_ADDR                 0x41        /*!< Slave address of the TOF sensor */

#define TOF_EN   GPIO_NUM_18

//Defines

#define FW_HEADER_LEN 4

//Commands

static uint8_t DOWNLOAD_INIT[5] = {0x08, 0x14, 0x01, 0x29, 0xC1};

static uint8_t SET_FW_ADDR[6] = {0x08, 0x43, 0x02, 0x00, 0x00, 0xBA};

static uint8_t RAM_REMAP[4] = {0x08, 0x11, 0x00, 0xEE};

static const char *TAG = "TOF LOG";

static uint8_t TOF_FIRMWARE_CHECK(void);
static uint8_t TOF_FIRMWARE_DOWNLOAD(void);
static uint8_t TOF_DOWNLOAD_CMD(unsigned long firmware_idx, uint8_t firmware_length);
static uint8_t TOF_WAIT_UNTIL_READY(void);


void TOF_INIT(void)
{
	//I2C SETUP
	
	gpio_set_level(TOF_EN, 1);
	
	int i2c_master_port = I2C_MASTER_NUM;

    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = 0,
        .scl_pullup_en = 0,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    i2c_param_config(i2c_master_port, &i2c_conf);

    ESP_ERROR_CHECK(i2c_driver_install(i2c_master_port, i2c_conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0));

	//TOF ENABLE

	//zero-initialize the config structure.
    gpio_config_t io_conf = {};
	//interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //bit mask of the battery stat
    io_conf.pin_bit_mask = (1ULL<<TOF_EN);
    //set as input mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
	
	if(TOF_FIRMWARE_CHECK())
	{
		ESP_LOGI(TAG, "TOF app initialized successfully.");
	}
}

esp_err_t TOF_READ(uint8_t* TOF_OUT, uint8_t dat_size)
{
	return i2c_master_read_from_device(I2C_MASTER_NUM, TOF_SENSOR_ADDR, TOF_OUT, dat_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t TOF_READ_WRITE(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size)
{
	return i2c_master_write_read_device(I2C_MASTER_NUM, TOF_SENSOR_ADDR, TOF_IN, in_dat_size, TOF_OUT, out_dat_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

esp_err_t TOF_WRITE(uint8_t* TOF_IN, uint8_t dat_size)
{
	return i2c_master_write_to_device(I2C_MASTER_NUM, TOF_SENSOR_ADDR, TOF_IN, dat_size, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

static uint8_t TOF_FIRMWARE_CHECK(void)
{
	//Check that firmware is correct version. Otherwise download new bootloader
	uint8_t tof_reg_addr = 0xE0;
	uint8_t tof_data = 0;
	while((tof_data & 0xCF) != 0x41) // wait until it is b01xx_0001
	{
		if(TOF_READ_WRITE(&tof_data, 1, &tof_reg_addr, 1) == ESP_OK)
		{
			ESP_LOGI(TAG, "TOF enable return is %x", tof_data);
		}
		else
		{
			return 0;
		}
	}
	tof_reg_addr = 0x00;
	if(TOF_READ_WRITE(&tof_data, 1, &tof_reg_addr, 1) == ESP_OK)
	{
		ESP_LOGI(TAG, "TOF appid is %x", tof_data);
	}
	else
	{
		return 0;
	}
	
	if(tof_data == 0x03)
	{
		ESP_LOGI(TAG, "TOF app is running.");
	}
	else if(tof_data == 0x80)
	{
		ESP_LOGI(TAG, "Bootloader is running, installing firmware.");
		if(!TOF_FIRMWARE_DOWNLOAD()) return 0;
	}
	else
	{
		ESP_LOGE(TAG, "Something bad happened while checking app id.");
		return 0;
	}
	
	return 1;
}

static uint8_t TOF_FIRMWARE_DOWNLOAD(void)
{
	//TODO:
	//1. Put TMF8828 into bootloader mode
	//2. Go to correct location in memory
	//3. Loop writing Firmware into TMF8828 via i2c_conf
	//4. Restart TMF8288 into application mode
	
	// Step 1:

	ESP_LOGI(TAG, "Sending Download Init Command");

	if(TOF_WRITE(DOWNLOAD_INIT, 5) != ESP_OK) return 0;

	if(!TOF_WAIT_UNTIL_READY()) return 0;
	
	// Step 2:

	ESP_LOGI(TAG, "Sending FW ADDR Command");

	if(TOF_WRITE(SET_FW_ADDR, 5) != ESP_OK) return 0;
	
	if(!TOF_WAIT_UNTIL_READY()) return 0;
	
	// Step 3:

	ESP_LOGI(TAG, "Sending Firmware Data");
	
	unsigned long firmware_idx = 0;
	uint8_t firmware_length = 64;
	
	while(firmware_idx < tof_bin_image_length)
	{
		unsigned long data_remaining = tof_bin_image_length - firmware_idx;
		if(data_remaining)
		if(!TOF_DOWNLOAD_CMD(firmware_idx, firmware_length)) return 0;
		firmware_idx += firmware_length;
		
		if(!TOF_WAIT_UNTIL_READY()) return 0;
	}

	// Step 4:

	ESP_LOGI(TAG, "Sending RAM Remap Command");

	TOF_WRITE(RAM_REMAP, 5);

	vTaskDelay(5 / portTICK_PERIOD_MS); //wait about 5 milliseconds for reboot before checking that App is running

	ESP_LOGI(TAG, "Checking that firmware is running");

	if(!TOF_FIRMWARE_CHECK())
	{
		ESP_LOGE(TAG, "Bootloader Download failed.");
		
		return 0;
	}

	return 1;
}

static uint8_t TOF_DOWNLOAD_CMD(unsigned long firmware_idx, uint8_t firmware_length)
{
	uint8_t packet_len = (firmware_length + FW_HEADER_LEN);
	
	uint8_t *cmd_and_data = malloc(packet_len * sizeof(uint8_t));
	
	*cmd_and_data = 0x08;
	
	*(cmd_and_data + 1) = 0x41;
	
	uint8_t checksum = 0x41;
	
	*(cmd_and_data + 2) = firmware_length;
	
	checksum += firmware_length;
	
	unsigned long current_fw_idx = firmware_idx;
	
	ESP_LOGI(TAG, "Creating Data Packet. Packet length is %u. Current FW index is %lx.", packet_len, firmware_idx);

	int i = 0;
	
	for(i = 3; i < packet_len - 1; i++)
	{
		*(cmd_and_data + i) = tof_bin_image[current_fw_idx];
		checksum += tof_bin_image[current_fw_idx];
		current_fw_idx++;
	}
	
	ESP_LOGI(TAG, "Checksum is %x.", ~checksum);
	
	*(cmd_and_data + i) = ~checksum;
	
	esp_err_t i2c_write_err = TOF_WRITE(cmd_and_data, packet_len);
	free(cmd_and_data);
	
	if(i2c_write_err == ESP_OK)
	{
		return 1;
	}
	else
	{
		ESP_LOGI(TAG, "Failed to send firmware data i2c packet.");
		return 0;
	}
}

static uint8_t TOF_WAIT_UNTIL_READY(void)
{
	//Check that command was received properly. Otherwise return failed
	uint8_t tof_reg_addr = 0x41;
	uint8_t tof_data[3] = {0, 0, 0};
	for(int i = 0; i < 5; i++) //Attempt 5 times to read return before giving up
	{
		if(TOF_READ_WRITE(tof_data, 3, &tof_reg_addr, 1) == ESP_OK)
		{
			ESP_LOGI(TAG, "TOF enable return is %x, %x, %x", tof_data[0], tof_data[2], tof_data[2]);
			if(tof_data[0] && (tof_data[2] != 0xFF)) 
			{
				ESP_LOGE(TAG, "Return code was unexpected.");
			}
			else
			{
				return 1;
			}
		}
		else
		{
			ESP_LOGE(TAG, "Failed to send i2c command.");
			return 0;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	ESP_LOGE(TAG, "Failed to receive correct return code.");
	return 0;
}
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "driver/gpio.h"

#include "ToF_I2C.h"
#include "tof_bin_image.h"
#include "MESSAGE_QUEUE.h"
#include "FLASH_SPI.h"

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
#define TOF_INTR GPIO_NUM_15

//Defines

#define FW_HEADER_LEN 4

#define tmf8821_fac_cal		"tmf8821_fac"
#define tmf8828_fac_cal_1	"tmf8828_fac_1"
#define tmf8828_fac_cal_2	"tmf8828_fac_2"
#define tmf8828_fac_cal_3	"tmf8828_fac_3"
#define tmf8828_fac_cal_4	"tmf8828_fac_4"

//Commands

static uint8_t DOWNLOAD_INIT[5] = {0x08, 0x14, 0x01, 0x29, 0xC1};

static uint8_t SET_FW_ADDR[6] = {0x08, 0x43, 0x02, 0x00, 0x00, 0xBA};

static uint8_t RAM_REMAP[4] = {0x08, 0x11, 0x00, 0xEE};

static uint8_t CHECK_CONFIG_PAGE_LOADED[4] = {0x16, 0x00, 0xBC, 0x00};

static const char *TAG = "TOF LOG";

// Internal Functions

static uint8_t TOF_FIRMWARE_CHECK(void);
static uint8_t TOF_FIRMWARE_DOWNLOAD(void);
static uint8_t TOF_DOWNLOAD_CMD(unsigned long firmware_idx, uint8_t firmware_length);
static uint8_t TOF_WAIT_UNTIL_READY(void);
static uint8_t TOF_WAIT_UNTIL_READY_APP(uint8_t delay_between_attempts);
static uint8_t TOF_CHECK_REGISTERS(uint8_t* read_reg, uint8_t* comp_reg, uint8_t size);
static esp_err_t TOF_READ_WRITE_APP(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size, uint8_t wait_ms);
static esp_err_t TOF_WRITE_APP(uint8_t* TOF_IN, uint8_t dat_size, uint8_t wait_ms);
static uint8_t TOF_SET_FACTORY_CAL_BLOB_NAME(uint8_t iter, char* blob_name);

// Internal Variables

static bool s_is_tmf8828_mode = false;
static TOF_DATA_t* s_ring_buffer_ptr = NULL;
static uint8_t s_ring_buffer_size = 0;
static uint8_t s_measurement_iter = 0;
static uint8_t s_measurement_buffer[4][132] = {0};
static uint8_t s_measurement_flags = 0;
static uint8_t s_current_config = 0;

// Interrupt Handler

static void TOF_MEASUREMENT_INTR_HANDLE(void);

// Task Handling

// Message Handler
static void TOF_MESSAGE_HANDLER(void);

// Task to Convert Read Buffer to a distance array
static uint8_t TOF_CONVERT_READ_BUFFER_TO_ARRAY(void);


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

	//TOF INTERRUPT

	//interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the battery stat
    io_conf.pin_bit_mask = (1ULL<<TOF_INTR);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
	
	s_is_tmf8828_mode = false;
	s_ring_buffer_ptr = NULL;
	s_ring_buffer_size = 0;
	s_measurement_iter = 0;
	s_measurement_flags = 0;
	s_current_config = 0;
	
	if(!TOF_FIRMWARE_CHECK())
	{
		ESP_LOGI(TAG, "TOF app initialized successfully.");
	}

	// Check that we are running TMF8828 firmware
	uint8_t mode_addr = 0x10;
	uint8_t mode_data = 0;
	if(TOF_READ_WRITE_APP(&mode_addr, 1, &mode_data, 1, 5) == ESP_OK)
	{
		ESP_LOGI(TAG, "Mode is %x", mode_data);
		if(mode_data == 0x00)
		{
			uint8_t write_data[2] = {0x08, 0x6C};
			TOF_WRITE_APP(write_data, 2, 5);
			TOF_WAIT_UNTIL_READY_APP(3);
		}
		s_is_tmf8828_mode = true; //assume that we were succssful in setting tmf8828 mode
	}

	//TOF INTERRUPT HANDLER
	gpio_isr_handler_add(TOF_INTR, TOF_MEASUREMENT_INTR_HANDLE, NULL);
}

uint8_t TOF_LOAD_CONFIG(uint8_t config)
{
	uint8_t write_data[3] = {0, 0, 0};
	uint8_t read_data[4] = {0, 0, 0, 0};
	
	//Steps:

	//Load Config Page
	write_data[0] = 0x08;
	write_data[1] = 0x16;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	//Check command was executed
	if(TOF_WAIT_UNTIL_READY_APP(3)) return 1;
	write_data[0] = 0x20;
	if(TOF_READ_WRITE_APP(write_data, 1, read_data, 4, 5) != ESP_OK) return 1;
	if(TOF_CHECK_REGISTERS(read_data, CHECK_CONFIG_PAGE_LOADED, 4) > 1)
	{
		return 1;
	}

	//Setup each config register according to input setting
	//TODO: Maybe add ability to save configurations dynamically?
	switch(config)
	{
		case 0:
			//example case where measurement period is set to 100 milliseconds
			write_data[0] = 0x24;
			write_data[1] = 0x64;
			write_data[3] = 0x00;
			if(TOF_WRITE_APP(write_data, 3, 5) != ESP_OK) return 1;

		default:
			//maybe set a default configuration?
			break;
	}

	//Write Command to Write Config Page
	write_data[0] = 0x08;
	write_data[1] = 0x15;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	//Check Command was executed
	if(TOF_WAIT_UNTIL_READY_APP(3)) return 1;

	//Write Interrupt Settings
	//For example setting interrupts for results with this
	write_data[0] = 0xE2;
	write_data[1] = 0x02;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	//Clear pending interrupts
	write_data[0] = 0xE1;
	write_data[1] = 0xFF;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	s_current_config = config;

	//Return if successful
	return 0;
}

uint8_t TOF_RESET(void)
{
	ESP_LOGI(TAG, "Resetting ToF into bootloader mode");
	uint8_t write_data[2] = {0xE0, 0x01};
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;
	write_data[0] = 0xF0; 
	write_data[0] = 0x80;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;
	if(TOF_FIRMWARE_CHECK()) return 1;
	return 0;
}

uint8_t TOF_FACTORY_CALIBRATION(void)
{
	uint8_t number_of_factory_calibrations = (s_is_tmf8828_mode) ? 4 : 1;
	uint8_t write_data[2] = {0, 0};
	
	// Steps:

	// Reset Factory Calibration Counter
	write_data[0] = 0x08;
	write_data[1] = 0x1F;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	for(int i = 0; i < number_of_factory_calibrations; i++)
	{
		// Start Factory Calibration
		write_data[0] = 0x08;
		write_data[1] = 0x20;
		if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

		// Check command was executed
		if(TOF_WAIT_UNTIL_READY_APP(1000)) return 1;
	}

	return 0;
}

uint8_t TOF_STORE_FACTORY_CALIBRATION(void)
{
	uint8_t number_of_factory_calibrations = (s_is_tmf8828_mode) ? 4 : 1;
	uint8_t write_data[2] = {0, 0};
	uint8_t read_data[64] = {0};
	uint8_t factory_cal_blob[0xC0];
	char fac_cal_blob_name[20] = {0};

	// Steps:
	
	// Reset Factory Calibration Counter
	write_data[0] = 0x08;
	write_data[1] = 0x1F;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	for(int i = 0; i < number_of_factory_calibrations; i++)
	{
		//determine blob name
		if(TOF_SET_FACTORY_CAL_BLOB_NAME(i, fac_cal_blob_name)) return 1;
		
		// Load Factory Calibration Page
		write_data[0] = 0x08;
		write_data[1] = 0x19;
		if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

		// Check command was executed
		if(TOF_WAIT_UNTIL_READY_APP(3)) return 1;

		// Read out Factory Calibration
		for(int j = 0; j < 3; j++)
		{
			write_data[0] = 0x20 + (j * 0x40);
			if(TOF_READ_WRITE_APP(write_data, 1, read_data, 0x40, 5) == ESP_OK)
			{
				memcpy(&factory_cal_blob[j * 0x40], read_data, 0x40);
			}
			else
			{
				return 1;
			}
		}

		if(FLASH_WRITE_TO_BLOB(MAIN_PARTITION, "tof", fac_cal_blob_name, factory_cal_blob, 0xC0)) return 1;

		if(i < number_of_factory_calibrations - 1)
		{
			// Write Page Config to go to next Calibration
			write_data[0] = 0x08;
			write_data[1] = 0x15;
			if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

			// Check command was executed
			if(TOF_WAIT_UNTIL_READY_APP(3)) return 1;
		}
	}

	return 0;
}

uint8_t TOF_LOAD_FACTORY_CALIBRATION(void)
{
	uint8_t number_of_factory_calibrations = (s_is_tmf8828_mode) ? 4 : 1;
	uint8_t write_data[65] = {0};
	uint8_t factory_counter = 0;
	uint8_t factory_calibration[0xC0];
	char fac_cal_blob_name[20] = {0};

	// Steps:
	
	// Reset Factory Calibration Counter
	write_data[0] = 0x08;
	write_data[1] = 0x1F;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

	for(int i = 0; i < number_of_factory_calibrations; i++)
	{
		//determine blob name
		if(TOF_SET_FACTORY_CAL_BLOB_NAME(i, fac_cal_blob_name)) return 1;
		
		// Load Factory Calibration Page
		write_data[0] = 0x08;
		write_data[1] = 0x19;
		if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

		// Check command was executed
		if(TOF_WAIT_UNTIL_READY_APP(3)) return 1;

		// Get Factory Calibration from memory

		if(FLASH_DOES_KEY_EXIST(MAIN_PARTITION, "tof", fac_cal_blob_name) != 0xC0) return 1;

		//Need to get pointer from SPI memory API
		factory_calibration = FLASH_READ_FROM_BLOB(MAIN_PARTITION, "tof", fac_cal_blob_name, 0xC0);

		if(factory_calibration == NULL) return 1;
		
		// write factory calibration to memory
		while(factory_counter < 0xC0)
		{
			uint8_t dat_size = 0xC0 - factory_counter;
			if(dat_size > 0x40)
			{
				dat_size = 0x40;
			}
			write_data[0] = 0x24 + (factory_counter);
			memncpy((write_data + 1), (factory_calibration + factory_counter + 4), dat_size);
			if(TOF_WRITE_APP(write_data, dat_size + 1, 5) != ESP_OK) return 1;
			factory_counter += dat_size;
		}

		// Free memory that held factory calibration
		free(factory_calibration);

		// Write Page Config to go to next Calibration
		write_data[0] = 0x08;
		write_data[1] = 0x15;
		if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;

		// Check command was executed
		if(TOF_WAIT_UNTIL_READY_APP(3)) return 1;
	}
}

uint8_t TOF_RETURN_CALIBRATION_STATUS(void)
{
	uint8_t tof_reg_addr = 0x07;
	uint8_t tof_data = 0;
	if(TOF_READ_WRITE_APP(&tof_data, 1, &tof_reg_addr, 1, 5) == ESP_OK)
	{
		return tof_data;
	}
	else
	{
		return 1;
	}
}

uint8_t TOF_START_MEASUREMENTS(TOF_DATA_t* TOF_DATA_PTR, uint8_t ring_buf_size)
{
	uint8_t write_data[2] = {0, 0};
	write_data[0] = 0x08;
	write_data[1] = 0x10;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;
	s_ring_buffer_ptr = TOF_DATA_PTR;
	s_ring_buffer_size = ring_buf_size;
	return 0;
}

uint8_t TOF_STOP_MEASUREMENTS(void)
{
	uint8_t write_data[2] = {0, 0};
	write_data[0] = 0x08;
	write_data[1] = 0xFF;
	if(TOF_WRITE_APP(write_data, 2, 5) != ESP_OK) return 1;
	s_ring_buffer_ptr = NULL;
	s_ring_buffer_size = 0;
	return 0;
}

static esp_err_t TOF_READ_WRITE_APP(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size, uint8_t wait_ms)
{
	esp_err_t err = TOF_READ_WRITE(TOF_OUT, out_dat_size, TOF_IN, in_dat_size);
	vTaskDelay(wait_ms / portTICK_PERIOD_MS);
	return err;
}

static esp_err_t TOF_WRITE_APP(uint8_t* TOF_IN, uint8_t dat_size, uint8_t wait_ms)
{
	esp_err_t err = TOF_WRITE(TOF_IN, dat_size);
	vTaskDelay(wait_ms / portTICK_PERIOD_MS);
	return err;
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

static uint8_t TOF_SET_FACTORY_CAL_BLOB_NAME(uint8_t iter, char* blob_name)
{
	size_t fac_cal_strlen = 0;
	switch(iter)
	{
		case 0:
		{
			if(number_of_factory_calibrations > 1)
			{
				fac_cal_strlen = strlen(tmf8828_fac_cal_1);
				memcpy(blob_name, tmf8828_fac_cal_1, fac_cal_strlen);
				blob_name[fac_cal_strlen] = '_';
				blob_name[fac_cal_strlen] = (s_current_config + '0');
				blob_name[fac_cal_strlen] = '\0';
				return 0;
			}
			else
			{
				fac_cal_strlen = strlen(tmf8821_fac_cal);
				memcpy(blob_name, tmf8821_fac_cal, fac_cal_strlen);
				blob_name[fac_cal_strlen] = '_';
				blob_name[fac_cal_strlen] = (s_current_config + '0');
				blob_name[fac_cal_strlen] = '\0';
				return 0;
			}
		}
		case 1:
		{
			fac_cal_strlen = strlen(tmf8828_fac_cal_2);
			memcpy(blob_name, tmf8828_fac_cal_2, fac_cal_strlen);
			blob_name[fac_cal_strlen] = '_';
			blob_name[fac_cal_strlen] = (s_current_config + '0');
			blob_name[fac_cal_strlen] = '\0';
			return 0;
		}
		case 2:
		{
			fac_cal_strlen = strlen(tmf8828_fac_cal_3);
			memcpy(blob_name, tmf8828_fac_cal_3, fac_cal_strlen);
			blob_name[fac_cal_strlen] = '_';
			blob_name[fac_cal_strlen] = (s_current_config + '0');
			blob_name[fac_cal_strlen] = '\0';
			return 0;
		}
		case 3:
		{
			fac_cal_strlen = strlen(tmf8828_fac_cal_3);
			memcpy(blob_name, tmf8828_fac_cal_3, fac_cal_strlen);
			blob_name[fac_cal_strlen] = '_';
			blob_name[fac_cal_strlen] = (s_current_config + '0');
			blob_name[fac_cal_strlen] = '\0';
			return 0;
		}
		default:
		{
			ESP_LOGE(TAG, "Storing invalid number of calibrations, exiting");
			return 1;
		}
	}
}

static uint8_t TOF_FIRMWARE_CHECK(void)
{
	//Check that firmware is correct version. Otherwise download new bootloader
	uint8_t tof_reg_addr = 0xE0;
	uint8_t tof_data[3] = {0, 0, 0};
	while((tof_data[0] & 0xCF) != 0x41) // wait until it is b01xx_0001
	{
		if(TOF_READ_WRITE(tof_data, 1, &tof_reg_addr, 1) == ESP_OK)
		{
			ESP_LOGI(TAG, "TOF enable return is %x", tof_data[0]);
		}
		else
		{
			return 1;
		}
	}
	tof_reg_addr = 0x00;
	if(TOF_READ_WRITE(tof_data, 3, &tof_reg_addr, 1) == ESP_OK)
	{
		ESP_LOGI(TAG, "TOF appid is %x, %x, %x", tof_data[0], tof_data[1], tof_data[2]);
	}
	else
	{
		return 1;
	}
	
	if(tof_data[0] == 0x03)
	{
		ESP_LOGI(TAG, "TOF app is running.");
		tof_reg_addr = 0x04;
		if(TOF_READ_WRITE(tof_data, 1, &tof_reg_addr, 1) == ESP_OK)
		{
			ESP_LOGI(TAG, "App Status is %x", tof_data[0]);
		}
	}
	else if(tof_data[0] == 0x80)
	{
		ESP_LOGI(TAG, "Bootloader is running, installing firmware.");
		if(TOF_FIRMWARE_DOWNLOAD()) return 1;
	}
	else
	{
		ESP_LOGE(TAG, "Something bad happened while checking app id.");
		tof_reg_addr = 0x04;
		if(TOF_READ_WRITE(tof_data, 1, &tof_reg_addr, 1) == ESP_OK)
		{
			ESP_LOGI(TAG, "App Status is %x", tof_data[0]);
		}
		return 1;
	}
	
	return 0;
}

static uint8_t TOF_FIRMWARE_DOWNLOAD(void)
{
	//TODO:
	//1. Put TMF8828 into bootloader mode
	//2. Go to correct location in memory
	//3. Loop writing Firmware into TMF8828 via i2c_conf
	//4. Restart TMF8288 into application mode
	
	// Step 2:

	ESP_LOGI(TAG, "Sending FW ADDR Command");

	if(TOF_WRITE(SET_FW_ADDR, 6) != ESP_OK) return 1;
	
	if(TOF_WAIT_UNTIL_READY()) return 1;
	
	// Step 3:

	ESP_LOGI(TAG, "Sending Firmware Data");
	
	unsigned long firmware_idx = 0;
	uint8_t firmware_length = 64;
	
	while(firmware_idx < tof_bin_image_length)
	{
		unsigned long data_remaining = tof_bin_image_length - firmware_idx;
		if(data_remaining)
		if(TOF_DOWNLOAD_CMD(firmware_idx, firmware_length)) return 1;
		firmware_idx += firmware_length;
		
		if(TOF_WAIT_UNTIL_READY()) return 1;
	}

	uint8_t write_data[2] = {0xE0, 0x21};
	if(TOF_WRITE(write_data, 2) != ESP_OK) return 1;

	// Step 4:

	ESP_LOGI(TAG, "Sending RAM Remap Command");

	TOF_WRITE(RAM_REMAP, 5);

	vTaskDelay(5 / portTICK_PERIOD_MS); //wait about 5 milliseconds for reboot before checking that App is running

	ESP_LOGI(TAG, "Checking that firmware is running");

	if(TOF_FIRMWARE_CHECK())
	{
		ESP_LOGE(TAG, "Bootloader Download failed.");
		
		return 1;
	}

	return 0;
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
		return 0;
	}
	else
	{
		ESP_LOGI(TAG, "Failed to send firmware data i2c packet.");
		return 1;
	}
}

static uint8_t TOF_WAIT_UNTIL_READY(void)
{
	//Check that command was received properly. Otherwise return failed
	uint8_t tof_reg_addr = 0x08;
	uint8_t tof_data[3] = {0, 0, 0};
	for(int i = 0; i < 5; i++) //Attempt 5 times to read return before giving up
	{
		if(TOF_READ_WRITE(tof_data, 3, &tof_reg_addr, 1) == ESP_OK)
		{
			ESP_LOGI(TAG, "TOF enable return is %x, %x, %x", tof_data[0], tof_data[1], tof_data[2]);
			if(tof_data[0] && (tof_data[2] != 0xFF)) 
			{
				ESP_LOGE(TAG, "Return code was unexpected.");
			}
			else
			{
				return 0;
			}
		}
		else
		{
			ESP_LOGE(TAG, "Failed to send i2c command.");
			return 1;
		}
		vTaskDelay(1 / portTICK_PERIOD_MS);
	}
	ESP_LOGE(TAG, "Failed to receive correct return code.");
	return 1;
}

static uint8_t TOF_WAIT_UNTIL_READY_APP(uint8_t delay_between_attempts)
{
	//Same as TOF_WAIT_UNTIL_READY but does not check for checksum.
	uint8_t tof_reg_addr = 0x08;
	uint8_t tof_data = 0;
	for(int i = 0; i < 5; i++) //Attempt 5 times to read return before giving up
	{
		if(TOF_READ_WRITE_APP(&tof_data, 1, &tof_reg_addr, 1, 5) == ESP_OK)
		{
			ESP_LOGI(TAG, "TOF enable return is %x", tof_data);
			if(tof_data == 0x00 || tof_data == 0x01) 
			{
				return 0;
			}
			else
			{
				ESP_LOGE(TAG, "Return code was unexpected.");
			}
		}
		else
		{
			ESP_LOGE(TAG, "Failed to send i2c command.");
			return 1;
		}
		vTaskDelay(delay_between_attempts / portTICK_PERIOD_MS);
	}
	ESP_LOGE(TAG, "Failed to receive correct return code.");
	return 1;
}

static uint8_t TOF_CHECK_REGISTERS(uint8_t* read_reg, uint8_t* comp_reg, uint8_t size)
{
	uint8_t mismatched_reg_count = 0;
	for(int i = 0; i < size; i++)
	{
		ESP_LOGI(TAG, "register value is %x.", read_reg[i]);
		if(read_reg[i] != comp_reg[i])
		{
			mismatched_reg_count++;
		}
	}
	ESP_LOGI(TAG, "number of mismatched registers is %d.", mismatched_reg_count);
	return mismatched_reg_count;
}

static void TOF_MESSAGE_HANDLER(void)
{
	
}

static uint8_t TOF_CONVERT_READ_BUFFER_TO_ARRAY(void)
{
	return 0;
}

static void TOF_MEASUREMENT_INTR_HANDLE(void)
{
	uint8_t number_of_measurements = (s_is_tmf8828_mode) ? 4 : 1;
	uint8_t write_data[2] = {0, 0};
	uint8_t read_data[1] = {0};

	// Steps:

	//Exit early if we are overwriting the buffer
	if(s_measurement_flags & (0x01 << s_measurement_iter)) return;

	//Read Interrupt Settings
	//For example setting interrupts for results with this
	write_data[0] = 0xE1;
	if(TOF_READ_WRITE_APP(write_data, 1, read_data, 1, 1) != ESP_OK) return;

	ESP_LOGI(TAG, "interrupts that need to be cleared are the following: %u.", read_data[0]);
	
	// Read out Measurement
	write_data[0] = 0x20;
	if(!TOF_READ_WRITE_APP(write_data, 1, s_measurement_buffer[s_measurement_iter], 132, 1) == ESP_OK) return;

	//Set flags for buffers
	s_measurement_flags &= (1 << s_measurement_iter);

	s_measurement_iter++;

	if(s_measurement_iter >= number_of_measurements)
	{
		s_measurement_iter = 0;
	}

	//Clear pending interrupts
	write_data[0] = 0xE1;
	write_data[1] = 0x02;
	if(TOF_WRITE_APP(write_data, 2, 1) != ESP_OK) return;

	//Queue Message to Process Read Buffer

	ESP_LOGI(TAG, "Read measurement successfully.");
}
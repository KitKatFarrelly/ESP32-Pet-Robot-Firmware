#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"
#include "freertos/timers.h"
#endif

#include "IMU_SPI.h"
#include "spi_config_data.h"
#include "MESSAGE_QUEUE.h"

// SPI definitions - TODO: 
#define PIN_NUM_MISO GPIO_NUM_37
#define PIN_NUM_MOSI GPIO_NUM_35
#define PIN_NUM_CLK  GPIO_NUM_36
#define PIN_NUM_CS   GPIO_NUM_34
#define TRANS_SIZE   8
#define DMA_CHAN     2

#define IMU_INT1 	GPIO_NUM_38
#define IMU_INT2 	GPIO_NUM_39

// Defines

#define FW_HEADER_LEN 4
#define IMU_BUF_SIZE 20
#define POSITION_BUF_SIZE 8
#define BURST_BYTE_NUMBER 64

// Important Addresses
#define BMI2_ERROR_REGISTER                           (0x02)
#define BMI2_STATUS_REGISTER                          (0x03)
#define BMI2_ACC_X_LSB_ADDR                           (0x0C)
#define BMI2_GYR_X_LSB_ADDR                           (0x12)
#define BMI2_SENSORTIME_ADDR                          (0x18)
#define BMI2_EVENTS_ADDR                              (0x1B)
#define BMI2_INT_STATUS_1_ADDR                        (0x1D) // 0x40 is gyro data ready, 0x80 is acc data ready
#define BMI2_INTERNAL_STATUS                          (0x21)
#define BMI2_ACC_CONF_ADDR                            (0x40)
#define BMI2_ACC_RANGE_ADDR                           (0x41)
#define BMI2_GYR_CONF_ADDR                            (0x42)
#define BMI2_GYR_RANGE_ADDR                           (0x43)
#define BMI2_ERROR_REG_MAP                            (0x52)
#define BMI2_INT1_IO_CTRL                             (0x53)
#define BMI2_INT2_IO_CTRL                             (0x54)
#define BMI2_INT_LATCH                                (0x55)
#define BMI2_INT1_FEATURES                            (0x56)
#define BMI2_INT2_FEATURES                            (0x57)
#define BMI2_INT_MAP_DATA_ADDR                        (0x58)
#define BMI2_INIT_CTRL_ADDR                           (0x59)
#define BMI2_INIT_ADDR_0                              (0x5B)
#define BMI2_INIT_ADDR_1                              (0x5C)
#define BMI2_INIT_DATA_ADDR                           (0x5E)
#define BMI2_PWR_CONF_ADDR                            (0x7C)
#define BMI2_PWR_CTRL_ADDR                            (0x7D)
#define BMI2_COMMAND_ADDR                             (0x7E)

static const char *TAG = "SPI_LOG";

// static variables
static spi_device_handle_t s_spi_handle = NULL;
static IMU_DATA_RAW_t s_imu_measurement_buffer[IMU_BUF_SIZE] = {0};
static uint8_t s_imu_buf_iter = 0;
TimerHandle_t s_imu_timer = NULL;

// static functions
static void imu_configuration_init(void);
static void imu_check_interrupt_data(TimerHandle_t xTimer);
static void imu_check_interrupt_err(void *arg);

// Externs
component_handle_t imu_public_component = 0;

#ifndef FUNCTIONAL_TESTS

void IMU_READ(uint8_t* IMU_OUT, uint8_t IMU_REG, uint8_t out_size)
{
    esp_err_t ret;
    spi_transaction_ext_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.base.rxlength = out_size * 8;
	t.dummy_bits = 8;
	t.base.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_VARIABLE_DUMMY;
	t.base.cmd = (0x80 | IMU_REG);
    ret=spi_device_polling_transmit(s_spi_handle, (spi_transaction_t*)&t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    for(uint8_t i = 0; i < out_size && i < 4; i++)
	{
		IMU_OUT[i] = t.base.rx_data[i];
	}
}

void IMU_READ_LONG(uint8_t* IMU_OUT, uint8_t IMU_REG, uint8_t out_size)
{
	esp_err_t ret;
    spi_transaction_ext_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.base.rx_buffer = IMU_OUT;
	t.base.rxlength = out_size * 8;
	t.dummy_bits = 8;
	t.base.flags = SPI_TRANS_VARIABLE_DUMMY;
	t.base.cmd = (0x80 | IMU_REG);
    ret=spi_device_polling_transmit(s_spi_handle, (spi_transaction_t*)&t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void IMU_WRITE(const uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	for(uint8_t i = 0; i < in_size && i < 4; i++)
	{
		t.tx_data[i] = *(IMU_IN + i);
	}
	t.length = 8 * in_size;
	t.flags = SPI_TRANS_USE_TXDATA;
	t.cmd = IMU_REG;
    ret=spi_device_polling_transmit(s_spi_handle, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

void IMU_WRITE_LONG(const uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.tx_buffer = IMU_IN;
	t.length = 8 * in_size;
	t.cmd = IMU_REG;
    ret=spi_device_polling_transmit(s_spi_handle, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
}

#endif

void IMU_INIT(void)
{
	//SPI SETUP

#ifndef FUNCTIONAL_TESTS

	//SPI INTERRUPTS

	//zero-initialize the config structure.
    gpio_config_t io_conf = {};
	//interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the battery stat
    io_conf.pin_bit_mask = (1ULL<<IMU_INT1);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	//interrupt of falling edge
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    //bit mask of the battery stat
    io_conf.pin_bit_mask = (1ULL<<IMU_INT2);
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

	//SPI INTERRUPT HANDLERS - again commenting out for now bc they dont work
	/*
	gpio_isr_handler_add(IMU_INT1, imu_check_interrupt_data, NULL);
	gpio_isr_handler_add(IMU_INT2, imu_check_interrupt_err, NULL);
	*/
	s_imu_timer = xTimerCreate("imu_timer", 8 / portTICK_PERIOD_MS, pdTRUE, (void*) 0, imu_check_interrupt_data);
	
	esp_err_t ret;
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=TRANS_SIZE
	};
	spi_device_interface_config_t devcfg={
		.command_bits = 8,						//8 CMD bits
		.address_bits = 0,						//0 ADDR bits
		.clock_speed_hz=4*1000*1000,			//Clock out at 4 MHz
		.mode=0,                                //SPI mode 0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
		.pre_cb = NULL,
		.flags = SPI_DEVICE_HALFDUPLEX,
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(SPI3_HOST, &devcfg, &s_spi_handle);
	ESP_ERROR_CHECK(ret);

#endif

	if(check_is_queue_active(1))
	{
		create_handle_for_component(&imu_public_component);
	}

	// Step 1: Run self test

	// Step 2: If successful, write config file
	imu_configuration_init();
}

uint8_t imu_accel_config(void)
{
	//configure acclerometer on sensor
	// 1. Accelerometer config 100Hz Normal Mode
	uint8_t write_data = 0xA8;
	IMU_WRITE(&write_data, BMI2_ACC_CONF_ADDR, 1);
	// 2. 4G max range
	write_data = 0x01;
	IMU_WRITE(&write_data, BMI2_ACC_RANGE_ADDR, 1);
	return 0;
}

uint8_t imu_gyro_config(void)
{
	//configure gyro on sensor
	// 1. Gyro config 100Hz Normal Mode
	uint8_t write_data = 0xE8;
	IMU_WRITE(&write_data, BMI2_GYR_CONF_ADDR, 1);
	// 2. 2000dps pre and post filter
	write_data = 0x08;
	IMU_WRITE(&write_data, BMI2_GYR_RANGE_ADDR, 1);
	return 0;
}

uint8_t imu_reset(void)
{
	//soft reset
	uint8_t write_data = 0xB6;
	IMU_WRITE(&write_data, BMI2_COMMAND_ADDR, 1);
	return 0;
}

uint8_t imu_check_status(void)
{
	//checks status register of imu
	uint8_t read_data = 0x00;
	IMU_READ(&read_data, BMI2_STATUS_REGISTER, 1);
	return read_data;
}

uint8_t imu_check_error(void)
{
	//checks error register of imu
	uint8_t read_data = 0x00;
	IMU_READ(&read_data, BMI2_ERROR_REGISTER, 1);
	return read_data;
}

uint8_t imu_check_events(void)
{
	//checks events register of imu
	uint8_t read_data = 0x00;
	IMU_READ(&read_data, BMI2_EVENTS_ADDR, 1);
	return read_data;
}

uint8_t imu_set_interrupts(void)
{
	//configure interrupts
	// 1. Error Registration - fatal and internal errors
	uint8_t write_data = 0x1F;
	IMU_WRITE(&write_data, BMI2_ERROR_REG_MAP, 1);
	// 2. INT1 IO - falling edge, push/pull, output enabled
	write_data = 0x04;
	IMU_WRITE(&write_data, BMI2_INT1_IO_CTRL, 1);
	// 3. INT2 IO - falling edge, push/pull, output enabled
	write_data = 0x04;
	IMU_WRITE(&write_data, BMI2_INT2_IO_CTRL, 1);
	// 4. Latching turned off
	write_data = 0x00;
	IMU_WRITE(&write_data, BMI2_INT_LATCH, 1);
	// 5. Enable reading from interrupt - error from int2, data from int1
	write_data = 0x84;
	IMU_WRITE(&write_data, BMI2_INT_MAP_DATA_ADDR, 1);
	return 0;
}

uint8_t imu_set_features(uint8_t feature_flags)
{
	//set features from a list of flags
	return 0;
}

static void imu_configuration_init(void)
{
	uint8_t write_data[2] = {0, 0};
	uint16_t config_index = 0;
	uint16_t burst_len = BURST_BYTE_NUMBER;
	uint16_t config_length = sizeof(bmi270_config_file);
	// Steps:

	// 1. Disable Advanced Power Features
	IMU_WRITE(write_data, BMI2_PWR_CONF_ADDR, 1);
	// 2. Disable Loading Config
	IMU_WRITE(write_data, BMI2_PWR_CONF_ADDR, 1);
	ESP_LOGI(TAG, "starting config write, config length is 0x%x", config_length);
	while(config_index < config_length)
	{
		if(config_index + burst_len > config_length)
		{
			burst_len = config_length - config_index;
		}
		ESP_LOGI(TAG, "index is 0x%x, write length is 0x%x", config_index, burst_len);
		// 3. Set write address to config index / 2
		write_data[0] = (uint8_t)((config_index / 2) & 0x0F);
		write_data[1] = (uint8_t)((config_index / 2) >> 4);
		// 4. Load address into BMI2_INIT_ADDR_0 (lowest 4 bits) and BMI2_INIT_ADDR_1 (upper 8 bits)
		IMU_WRITE(write_data, BMI2_INIT_ADDR_0, 2);
		// 5. write burst of config bytes into BMI2_INIT_DATA_ADDR
		IMU_WRITE_LONG(bmi270_config_file + config_index, BMI2_INIT_DATA_ADDR, burst_len);
		// 6. Increment config index by burst length
		config_index += burst_len;
		// 7. Repeat 3-7 until end of file
	}
	// 8. Enable Loading Config
	write_data[0] = 1;
	write_data[1] = 0;
	IMU_WRITE(write_data, BMI2_INIT_CTRL_ADDR, 1);
	ESP_LOGI(TAG, "init successful");
}

uint8_t imu_start(void)
{
	//start reading data from imu
	uint8_t write_data = 0;
	// Steps:

	// 1. Enable accelerometer, gyro data, disable aux
	write_data = 0x0E;
	IMU_WRITE(&write_data, BMI2_PWR_CTRL_ADDR, 1);
	// 2. Disable adv power saving, enable fifo_self_wakeup
	write_data = 0x02;
	IMU_WRITE(&write_data, BMI2_PWR_CONF_ADDR, 1);
	if(!xTimerIsTimerActive(s_imu_timer))
	{
		xTimerStart(s_imu_timer, 0);
	}
	return 0;
}

uint8_t imu_stop(void)
{
	//stop reading data from imu
	uint8_t write_data = 0x00;
	// Steps:

	// 1. Disable accelerometer, gyro data, disable aux
	IMU_WRITE(&write_data, BMI2_PWR_CTRL_ADDR, 1);
	// 2. Disable reading from interrupt
	write_data = 0x00;
	IMU_WRITE(&write_data, BMI2_INT_MAP_DATA_ADDR, 1);
	// 2. Enable adv power saving
	write_data = 0x02;
	IMU_WRITE(&write_data, BMI2_PWR_CONF_ADDR, 1);
	if(xTimerIsTimerActive(s_imu_timer))
	{
		xTimerStop(s_imu_timer, 0);
	}
	return 0;
}

static void imu_check_interrupt_data(TimerHandle_t xTimer)
{
	//Read interrupt values and if data is available read imu data
	uint8_t read_data = 0x00;

	ESP_LOGI(TAG, "interrupts are GPIO38: %u, GPIO39: %u.", gpio_get_level(IMU_INT1), gpio_get_level(IMU_INT2));
	
	// 1. Read which data is ready
	IMU_READ(&read_data, BMI2_INT_STATUS_1_ADDR, 1);
	if(read_data & 0x84)
	{
		IMU_READ(s_imu_measurement_buffer[s_imu_buf_iter].timestamp, BMI2_SENSORTIME_ADDR, 3);
	}

	// 2. read accel data if available
	if(read_data & 0x80)
	{
		IMU_READ_LONG(s_imu_measurement_buffer[s_imu_buf_iter].acc_data, BMI2_ACC_X_LSB_ADDR, 6);
	}
	// 3. read gyro data if available
	if(read_data & 0x40)
	{
		IMU_READ_LONG(s_imu_measurement_buffer[s_imu_buf_iter].gyr_data, BMI2_GYR_X_LSB_ADDR, 6);
	}
	// 4. send raw imu data to message queue
	if(check_is_queue_active(1))
	{
		message_info_t convert_spi_msg;
		convert_spi_msg.message_data = (void*) &s_imu_measurement_buffer[s_imu_buf_iter];
		convert_spi_msg.message_size = sizeof(IMU_DATA_RAW_t);
		convert_spi_msg.is_pointer = false;
		convert_spi_msg.component_handle = imu_public_component;
		convert_spi_msg.message_type = IMU_MSG_RAW_DATA;
		send_message_to_priority_queue(convert_spi_msg);
	}

	s_imu_buf_iter++;
	if(s_imu_buf_iter >= IMU_BUF_SIZE)
	{
		s_imu_buf_iter = 0;
	}
}

static void imu_check_interrupt_err(void *arg)
{
	//Check error
}

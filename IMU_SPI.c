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
#endif

#include "IMU_SPI.h"

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
#define IMU_DAT_SIZE 0x0F
#define POSITION_BUF_SIZE 8

// Important Addresses
#define BMI2_ACC_X_LSB_ADDR                           (0x0C)
#define BMI2_GYR_X_LSB_ADDR                           (0x12)
#define BMI2_SENSORTIME_ADDR                          (0x18)
#define BMI2_INIT_CTRL_ADDR                           (0x59)
#define BMI2_INIT_ADDR_0                              (0x5B)
#define BMI2_INIT_ADDR_1                              (0x5C)
#define BMI2_INIT_DATA_ADDR                           (0x5E)
#define BMI2_PWR_CONF_ADDR                            (0x7C)

static const char *TAG = "SPI_LOG";

// static variables
static spi_device_handle_t s_spi_handle = NULL;
static uint8_t s_imu_data_buffer[IMU_BUF_SIZE][IMU_DAT_SIZE] = {0};
static component_handle_t s_imu_private_handle = 0;

// static functions
static void imu_configuration_init(void);
static uint8_t imu_write_config_chunk(uint16_t index);
static void imu_check_interrupts(void);
static uint8_t imu_read_accel_gyro_dat(bool read_accel, bool read_gyro);

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
	ESP_LOGI(TAG, "error type is %x.", ret);
    assert(ret==ESP_OK);            //Should have had no issues.
    for(uint8_t i = 0; i < out_size && i < 4; i++)
	{
		IMU_OUT[i] = t.base.rx_data[i];
	}
}

void IMU_READ_LONG(uint8_t* IMU_OUT, uint8_t IMU_REG, uint8_t out_size)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.base.rx_buffer = IMU_OUT;
	t.base.rxlength = out_size * 8;
	t.dummy_bits = 8;
	t.base.flags = SPI_TRANS_USE_RXDATA | SPI_TRANS_VARIABLE_DUMMY;
	t.base.cmd = (0x80 | IMU_REG);
    ret=spi_device_polling_transmit(s_spi_handle, (spi_transaction_t*)&t);  //Transmit!
	ESP_LOGI(TAG, "error type is %x.", ret);
    assert(ret==ESP_OK);            //Should have had no issues.
}

void IMU_WRITE(uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size)
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
	ESP_LOGI(TAG, "error type is %x.", ret);
    assert(ret==ESP_OK);            //Should have had no issues.
}

void IMU_WRITE_LONG(uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.tx_buffer = IMU_IN;
	t.length = 8 * in_size;
	t.cmd = IMU_REG;
    ret=spi_device_polling_transmit(s_spi_handle, &t);  //Transmit!
	ESP_LOGI(TAG, "error type is %x.", ret);
    assert(ret==ESP_OK);            //Should have had no issues.
}

#endif

void IMU_INIT(void)
{
	//SPI SETUP

#ifndef FUNCTIONAL_TESTS
	
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

	// Step 1: Run self test

	// Step 2: If successful, write config file
	imu_configuration_init();
}

void imu_enable_accel(bool enable)
{
	//enable/disable acclerometer on sensor
}

void imu_enable_gyro(bool enable)
{
	//enable/disable gyro on sensor
}

uint8_t imu_reset(void)
{
	//soft reset
	return 0;
}

uint8_t imu_check_status(void)
{
	//checks status register of imu
	return 0;
}

uint8_t imu_check_error(void)
{
	//checks error register of imu
	return 0;
}

uint8_t imu_set_latched_mode(bool enable)
{
	//sets latched mode for imu
	return 0;
}

uint8_t imu_set_features(uint8_t feature_flags)
{
	//set features from a list of flags
	return 0;
}

static void imu_configuration_init(void)
{
	//placeholder
}

uint8_t imu_start(void)
{
	//start reading data from imu
	return 0;
}

uint8_t imu_stop(void)
{
	//stop reading data from imu
	return 0;
}

static uint8_t imu_write_config_chunk(uint16_t index)
{
	//Write config chunk to IMU based on config file
	return 0;
}

static void imu_check_interrupts(void)
{
	//Read interrupt values and if data is available read imu data
}
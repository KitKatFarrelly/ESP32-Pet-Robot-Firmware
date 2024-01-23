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

//SPI definitions - TODO: 
#define PIN_NUM_MISO GPIO_NUM_37
#define PIN_NUM_MOSI GPIO_NUM_35
#define PIN_NUM_CLK  GPIO_NUM_36
#define PIN_NUM_CS   GPIO_NUM_34
#define TRANS_SIZE   8
#define DMA_CHAN     2

static const char *TAG = "SPI LOG";

// static variables
static spi_device_handle_t s_spi_handle = NULL;

//static functions
static void imu_configuration_init(void);

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
    for(uint8_t i = 0; i < out_size; i++)
	{
		ESP_LOGI(TAG, "SPI data %u is %x", i, t.rx_data[i]);
		IMU_OUT[i] = t.rx_data[i];
	}
}

void IMU_WRITE(uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size)
{
	esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
	t.txlength = in_size * 8;
	t.tx_data = IMU_IN;
	t.flags = SPI_TRANS_USE_TXDATA;
	t.cmd = IMU_REG;
    ret=spi_device_polling_transmit(s_spi_handle, &t);  //Transmit!
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
		.clock_speed_hz=4*1000*1000,			//Clock out at 4 MHz
		.mode=0,                                //SPI mode 0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(SPI3_HOST, &devcfg, &s_spi_handle);
	ESP_ERROR_CHECK(ret);

#endif
}

static void imu_configuration_init(void)
{
	//placeholder
}

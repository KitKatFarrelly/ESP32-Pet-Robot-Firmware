#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"
#include "driver/spi_master.h"
#include "driver/spi_common.h"
#include "driver/gpio.h"

#include "IMU_SPI.h"

//SPI definitions - TODO: 
#define PIN_NUM_MISO GPIO_NUM_37
#define PIN_NUM_MOSI GPIO_NUM_35
#define PIN_NUM_CLK  GPIO_NUM_36
#define PIN_NUM_CS   GPIO_NUM_34
#define TRANS_SIZE   8
#define DMA_CHAN     2

static const char *TAG = "SPI LOG";

static void lcd_cmd(spi_device_handle_t spi, uint8_t* cmd, uint8_t in_size, uint8_t* out_data, uint8_t out_size)
{
    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       //Zero out the transaction
    t.length = in_size * 8;
	t.rxlength = out_size * 8;
	t.flags |= SPI_TRANS_USE_RXDATA;
	t.tx_buffer = cmd;
    ret=spi_device_polling_transmit(spi, &t);  //Transmit!
    assert(ret==ESP_OK);            //Should have had no issues.
    for(uint8_t i = 0; i < out_size; i++)
	{
		ESP_LOGI(TAG, "SPI data %u is %x", i, t.rx_data[i]);
		out_data[i] = t.rx_data[i];
	}
}

void IMU_INIT(void)
{
	//SPI SETUP
	
	esp_err_t ret;
	spi_device_handle_t spi;
	spi_bus_config_t buscfg={
		.miso_io_num=PIN_NUM_MISO,
		.mosi_io_num=PIN_NUM_MOSI,
		.sclk_io_num=PIN_NUM_CLK,
		.quadwp_io_num=-1,
		.quadhd_io_num=-1,
		.max_transfer_sz=TRANS_SIZE
	};
	spi_device_interface_config_t devcfg={
		.clock_speed_hz=4*1000*1000,           //Clock out at 4 MHz
		.mode=0,                                //SPI mode 0
		.spics_io_num=PIN_NUM_CS,               //CS pin
		.queue_size=7,                          //We want to be able to queue 7 transactions at a time
	};
	//Initialize the SPI bus
	ret=spi_bus_initialize(SPI3_HOST, &buscfg, DMA_CHAN);
	ESP_ERROR_CHECK(ret);
	//Attach the LCD to the SPI bus
	ret=spi_bus_add_device(SPI3_HOST, &devcfg, &spi);
	ESP_ERROR_CHECK(ret);
}
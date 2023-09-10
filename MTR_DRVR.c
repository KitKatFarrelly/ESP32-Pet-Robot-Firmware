#include <stdio.h>
#include <stdlib.h>
#include "driver/gpio.h"

#include "MTR_DRVR.h"

#define GPIO_MTR_PIN_SEL  ((1ULL<<MTR_STBY) | (1ULL<<MTR_R_IN1) | (1ULL<<MTR_R_IN2) | (1ULL<<MTR_L_IN1) | (1ULL<<MTR_L_IN2))

void MTR_INIT(void)
{
	//Motor Outputs
	
	gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_MTR_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
}
#include <stdio.h>
#include <stdlib.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "driver/gpio.h"
#endif

#include "LED_DRVR.h"

#define GPIO_LED_PIN_SEL  ((1ULL<<IND_LED_BLUE) | (1ULL<<IND_LED_GREEN) | (1ULL<<IND_LED_RED) | (1ULL<<EMO_LED_BLUE) | (1ULL<<EMO_LED_GREEN) | (1ULL<<EMO_LED_RED))

void LED_INIT(void)
{
	//LEDS

#ifndef FUNCTIONAL_TESTS
	
	//zero-initialize the config structure.
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_LED_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

#endif
}
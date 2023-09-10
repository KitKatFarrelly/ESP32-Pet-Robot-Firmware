#ifndef H_LED_DRVR
#define H_LED_DRVR

//GPIO definitions

#define EMO_LED_BLUE  GPIO_NUM_14
#define EMO_LED_GREEN GPIO_NUM_13
#define EMO_LED_RED   GPIO_NUM_12
#define IND_LED_BLUE  GPIO_NUM_11
#define IND_LED_GREEN GPIO_NUM_10
#define IND_LED_RED   GPIO_NUM_9

void LED_INIT(void);

//Sets analog color of a specific LED
void LED_SET_COLOR(gpio_num_t LED_NUM, uint8_t LED_VAL);

//Returns analog color of a specific LED
uint8_t LED_GET_COLOR(gpio_num_t LED_NUM);

#endif
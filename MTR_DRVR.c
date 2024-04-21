#include <stdio.h>
#include <stdlib.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "driver/gpio.h"
#include "driver/ledc.h"
#endif

#include "MTR_DRVR.h"

//PWM Driver Defs

#define MOTOR_PWM_TIMER             LEDC_TIMER_0
#define MOTOR_PWM_MODE              LEDC_LOW_SPEED_MODE
#define MOTOR_PWM_DUTY_RES          LEDC_TIMER_13_BIT // Set duty resolution to 13 bits
#define MOTOR_PWM_FREQUENCY         (5000) // Frequency in Hertz. Set frequency at 4 kHz
#define MOTOR_PWM_NUM_CHANNELS      4

//Motor Definitions

#define MTR_STBY            GPIO_NUM_4
#define MTR_R_IN1           GPIO_NUM_5
#define MTR_R_IN1_CHANNEL   LEDC_CHANNEL_0
#define MTR_R_IN2           GPIO_NUM_6
#define MTR_R_IN2_CHANNEL   LEDC_CHANNEL_1
#define MTR_L_IN1           GPIO_NUM_7
#define MTR_L_IN1_CHANNEL   LEDC_CHANNEL_2
#define MTR_L_IN2           GPIO_NUM_8
#define MTR_L_IN2_CHANNEL   LEDC_CHANNEL_3

typedef struct
{
    ledc_channel_t channel_fwd;
    ledc_channel_t channel_bck;
} mtr_context_t;

static const char *TAG = "MOTOR";

static mtr_direction_t motor_directions[2] = {MTR_DIR_NOT_SET, MTR_DIR_NOT_SET};
static uint16_t motor_duty_cycles[2] = {0, 0};

static void mtr_check_and_update_duty_cycles(bool is_right);
mtr_context_t mtr_get_context_from_handedness(bool is_right);

void MTR_INIT(void)
{
	//Motor Outputs

#ifndef FUNCTIONAL_TESTS
	
	gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<MTR_STBY);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // Prepare and then apply the LEDC PWM timer configuration
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = MOTOR_PWM_MODE,
        .timer_num        = MOTOR_PWM_TIMER,
        .duty_resolution  = MOTOR_PWM_DUTY_RES,
        .freq_hz          = MOTOR_PWM_FREQUENCY,  // Set output frequency at 4 kHz
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Prepare and then apply the LEDC PWM channel configuration
    ledc_channel_config_t ledc_channel[MOTOR_PWM_NUM_CHANNELS] = 
    {
        {
            .speed_mode     = MOTOR_PWM_MODE,
            .channel        = MTR_R_IN1_CHANNEL,
            .timer_sel      = MOTOR_PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MTR_R_IN1,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        },
        {
            .speed_mode     = MOTOR_PWM_MODE,
            .channel        = MTR_R_IN2_CHANNEL,
            .timer_sel      = MOTOR_PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MTR_R_IN2,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        },
        {
            .speed_mode     = MOTOR_PWM_MODE,
            .channel        = MTR_L_IN1_CHANNEL,
            .timer_sel      = MOTOR_PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MTR_L_IN1,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        },
        {
            .speed_mode     = MOTOR_PWM_MODE,
            .channel        = MTR_L_IN2_CHANNEL,
            .timer_sel      = MOTOR_PWM_TIMER,
            .intr_type      = LEDC_INTR_DISABLE,
            .gpio_num       = MTR_L_IN2,
            .duty           = 0, // Set duty to 0%
            .hpoint         = 0
        },
    };
    for (uint8_t ch = 0; ch < MOTOR_PWM_NUM_CHANNELS; ch++) {
        ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel[ch]));
    }
#endif
}

void mtr_set_direction(bool is_right, mtr_direction_t direction)
{
    //set which output channel is being used for motor
    motor_directions[is_right] = direction;
    mtr_check_and_update_duty_cycles(is_right);
}

void mtr_set_duty(bool is_right, uint16_t duty_cycle)
{
    //set LED PWM duty for motor
    motor_duty_cycles[is_right] = duty_cycle;
    mtr_context_t mtr_context = mtr_get_context_from_handedness(is_right);
    switch(motor_directions[is_right])
    {
        case MTR_DIR_FORWARD:
        {
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd, motor_duty_cycles[is_right]));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd));
            ESP_LOGI(TAG, "Set motor %u to forward at speed %u.", is_right, motor_duty_cycles[is_right]);
            break;
        }
        case MTR_DIR_REVERSE:
        {
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_bck, motor_duty_cycles[is_right]));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_bck));
            ESP_LOGI(TAG, "Set motor %u to reverse at speed %u.", is_right, motor_duty_cycles[is_right]);
            break;
        }
        default:
            break;
    }
}

mtr_direction_t mtr_get_direction(bool is_right)
{
    //return the directon of the motor
    return motor_directions[is_right];
}

uint16_t mtr_get_duty(bool is_right)
{
    //get LED PWM duty for motor
    return motor_duty_cycles[is_right];
}

void mtr_set_standby(bool standby)
{
    gpio_set_level(MTR_STBY, standby);
}

bool mtr_get_standby(void)
{
    return (bool) gpio_get_level(MTR_STBY);
}

static void mtr_check_and_update_duty_cycles(bool is_right)
{
    mtr_context_t mtr_context = mtr_get_context_from_handedness(is_right);
    switch(motor_directions[is_right])
    {
        case MTR_DIR_NOT_SET:
        {
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd, 0));
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_bck, 0));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd));
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_bck));
            ESP_LOGI(TAG, "Set motor %u to standby.", is_right);
            break;
        }
        case MTR_DIR_FORWARD:
        {
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd, motor_duty_cycles[is_right]));
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_bck, 0));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd));
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_bck));
            ESP_LOGI(TAG, "Set motor %u to forward at speed %u.", is_right, motor_duty_cycles[is_right]);
            break;
        }
        case MTR_DIR_REVERSE:
        {
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd, 0));
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_bck, motor_duty_cycles[is_right]));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd));
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_bck));
            ESP_LOGI(TAG, "Set motor %u to reverse at speed %u.", is_right, motor_duty_cycles[is_right]);
            break;
        }
        case MTR_DIR_STOPPED:
        {
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd, 8191));
            ESP_ERROR_CHECK(ledc_set_duty(MOTOR_PWM_MODE, mtr_context.channel_bck, 8191));
            // Update duty to apply the new value
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_fwd));
            ESP_ERROR_CHECK(ledc_update_duty(MOTOR_PWM_MODE, mtr_context.channel_bck));
            ESP_LOGI(TAG, "Set motor %u to stopped.", is_right);
            break;
        }
        default:
            break;
    }
}

mtr_context_t mtr_get_context_from_handedness(bool is_right)
{
    mtr_context_t mtr_context = 
    {
        .channel_fwd = MTR_L_IN1_CHANNEL;
        .channel_bck = MTR_L_IN2_CHANNEL;
    }
    if(is_right)
    {
        mtr_context.channel_fwd = MTR_R_IN1_CHANNEL;
        mtr_context.channel_bck = MTR_R_IN2_CHANNEL;
    }
    return mtr_context;
}

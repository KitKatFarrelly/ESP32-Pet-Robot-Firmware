#ifndef H_MTR_DRVR
#define H_MTR_DRVR

typedef enum
{
    MTR_DIR_NOT_SET = 0,
    MTR_DIR_STOPPED = 1,
    MTR_DIR_FORWARD = 2,
    MTR_DIR_REVERSE = 3,
    MTR_DIR_INVALID = 4,
} mtr_direction_t;

void MTR_INIT(void);

void mtr_set_direction(bool is_right, mtr_direction_t direction);

void mtr_set_duty(bool is_right, uint16_t duty_cycle);

mtr_direction_t mtr_get_direction(bool is_right);

uint16_t mtr_get_duty(bool is_right);

void mtr_set_standby(bool standby);

bool mtr_get_standby(void);

#endif
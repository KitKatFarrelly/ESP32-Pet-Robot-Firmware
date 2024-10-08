#ifndef H_IMU_DRVR
#define H_IMU_DRVR

#include "MESSAGE_QUEUE.h"

extern component_handle_t imu_public_component;

typedef struct
{
    uint8_t acc_data[6];
    uint8_t gyr_data[6];
    uint8_t timestamp[3];
    uint8_t flags;
} IMU_DATA_RAW_t;

typedef enum
{
    IMU_MSG_RAW_DATA,
    IMU_MSG_MAX,
} IMU_MESSAGE_TYPES_t;

void IMU_INIT(void);

void IMU_READ(uint8_t* IMU_OUT, uint8_t IMU_REG, uint8_t out_size);

void IMU_WRITE(const uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size);

void IMU_WRITE_LONG(const uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size);

uint8_t imu_accel_config(void);

uint8_t imu_gyro_config(void);

uint8_t imu_reset(void);

uint8_t imu_check_status(void);

uint8_t imu_check_error(void);

uint8_t imu_check_events(void);

uint8_t imu_set_interrupts(void);

uint8_t imu_set_features(uint8_t feature_flags);

uint8_t imu_start(void);

uint8_t imu_stop(void);

#endif
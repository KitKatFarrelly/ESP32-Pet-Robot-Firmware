#ifndef H_IMU_DRVR
#define H_IMU_DRVR

void IMU_INIT(void);

void IMU_READ(uint8_t* IMU_OUT, uint8_t* IMU_REG, uint8_t dat_size);

void IMU_READ_WRITE(uint8_t* IMU_OUT, uint8_t* IMU_REG, uint8_t IMU_CMD, uint8_t dat_size);

#endif
#ifndef H_IMU_DRVR
#define H_IMU_DRVR

void IMU_INIT(void);

void IMU_READ(uint8_t* IMU_OUT, uint8_t IMU_REG, uint8_t out_size);

void IMU_WRITE(uint8_t* IMU_IN, uint8_t IMU_REG, uint8_t in_size);

#endif
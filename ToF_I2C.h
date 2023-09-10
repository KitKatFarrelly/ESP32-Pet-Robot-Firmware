#ifndef H_TOF_DRVR
#define H_TOF_DRVR

void TOF_INIT(void);

esp_err_t TOF_READ(uint8_t* TOF_OUT, uint8_t dat_size);

esp_err_t TOF_READ_WRITE(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size);

esp_err_t TOF_WRITE(uint8_t* TOF_IN, uint8_t dat_size);

#endif
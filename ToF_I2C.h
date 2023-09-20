#ifndef H_TOF_DRVR
#define H_TOF_DRVR

// Initializes firmware on TOF sensor.
void TOF_INIT(void);

// Load TOF settings determined by config value.
// TODO: Change config to enum
uint8_t TOF_LOAD_CONFIG(uint8_t config);

// Performs Factory Calibration.
// If there is an existing factory calibration, load it.
// Return Failure if SPAD map does not match between sensor and factory calibration.
uint8_t TOF_FACTORY_CALIBRATION(void);

// Removes Factory Calibration from Flash and sensor, if one exists.
uint8_t TOF_RESET_FACTORY_CALIBRATION(void);

// Returns a single frame of data from the sensor. Returns data size if successful, 0 if failure.
uint8_t TOF_COLLECT_DATA_FRAME(uint8_t** TOF_DATA_PTR);

// Read Data directly from Sensor without referring to a register.
esp_err_t TOF_READ(uint8_t* TOF_OUT, uint8_t dat_size);

// Read Data from a specified Register.
esp_err_t TOF_READ_WRITE(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size);

// Write Data to a specified Register.
esp_err_t TOF_WRITE(uint8_t* TOF_IN, uint8_t dat_size);

#endif
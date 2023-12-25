#ifndef H_TOF_DRVR
#define H_TOF_DRVR

#include "MESSAGE_QUEUE.h"

typedef struct
{
    uint16_t** depth_pixel_field;
    uint8_t horizontal_size;
    uint8_t vertical_size;
    bool is_populated;
} TOF_DATA_t;

typedef enum
{
    TOF_MSG_INTERNAL_CONVERT_I2C,
    TOF_MSG_NEW_DEPTH_ARRAY,
    TOF_MSG_MAX,
} TOF_MESSAGE_TYPES_t;

component_handle_t ToF_public_component = 0;
TOF_DATA_t* ToF_ring_buffer_ptr = NULL;
uint8_t ToF_ring_buffer_size = 0;

// Initializes firmware on TOF sensor.
void TOF_INIT(void);

// Load TOF settings determined by config value.
// TODO: Change config to enum
uint8_t TOF_LOAD_CONFIG(uint8_t config);

uint8_t TOF_RESET(void);

// Performs Factory Calibration.
// If there is an existing factory calibration, load it.
// Return Failure if SPAD map does not match between sensor and factory calibration.
uint8_t TOF_FACTORY_CALIBRATION(void);

// Store Factory Calibration to Flash Memory
uint8_t TOF_STORE_FACTORY_CALIBRATION(void);

// Load Factory Calibration from Flash Memory
uint8_t TOF_LOAD_FACTORY_CALIBRATION(void);

// Returns the Calibration status of the device
// 0 means successful calibration
// 0x31 means no calibration was set; it is using default calibration
// 0x32 means calibration was set for the wrong SPAD map
// 1 means there was an error in reading from the device
uint8_t TOF_RETURN_CALIBRATION_STATUS(void);

// Tells TOF Sensor to start measuring data, with a pointer to a data ring buffer.
uint8_t TOF_START_MEASUREMENTS(TOF_DATA_t* TOF_DATA_PTR, uint8_t ring_buf_size);

// Tells TOF Sensor to stop measuring data.
uint8_t TOF_STOP_MEASUREMENTS(void);

// Read Data directly from Sensor without referring to a register.
esp_err_t TOF_READ(uint8_t* TOF_OUT, uint8_t dat_size);

// Read Data from a specified Register.
esp_err_t TOF_READ_WRITE(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size);

// Write Data to a specified Register.
esp_err_t TOF_WRITE(uint8_t* TOF_IN, uint8_t dat_size);

#endif
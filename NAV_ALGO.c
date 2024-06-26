#include "NAV_ALGO.h"
#include "MESSAGE_QUEUE.h"
#include "ToF_I2C.h"
#include "IMU_SPI.h"
#include "FLASH_SPI.h"

static callback_handle_t s_nav_tof_handle;
static callback_handle_t s_nav_imu_handle;
static bool s_is_navigation_enabled = false;

static const char *TAG = "NAV_ALG";

static void nav_algo_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data, size_t message_size);
static uint8_t nav_algo_convert_adjusted_confidence_value(uint16_t distance, uint8_t confidence);
//placeholder for imu kalman filter function
static void nav_algo_check_tof_array_against_map(TOF_DATA_t* tof_data);

bool nav_algo_init(void)
{
    s_nav_tof_handle = register_priority_handler_for_messages(nav_algo_queue_handler, ToF_public_component);
    s_nav_imu_handle = register_priority_handler_for_messages(nav_algo_queue_handler, imu_public_component);
    return true;
}

bool nav_algo_enable_navigation(bool enable)
{
    if(enable)
    {
        if(TOF_START_MEASUREMENTS())
        {
            return false;
        }
    }
    else
    {
        if(TOF_STOP_MEASUREMENTS())
        {
            return false;
        }
    }
    s_is_navigation_enabled = enable;
    return s_is_navigation_enabled;
}

bool nav_algo_restart_temp_map(void)
{
    return false;
}

NAV_MAP_T nav_algo_closest_map_to_temp_map(void)
{
    return NULL;
}

NAV_MAP_T nav_algo_start_writing_map(void)
{
    return NULL;
}

bool nav_algo_stop_writing_map(NAV_MAP_T map)
{
    return false;
}

bool nav_algo_save_map(NAV_MAP_T map_to_save)
{
    return false;
}

bool nav_algo_load_map(NAV_MAP_T map_to_load)
{
    return false;
}

NAV_MAP_T nav_algo_get_current_map(void)
{
    return NULL;
}

NAV_CURRENT_POS_T nav_algo_get_current_pos(void)
{
    return NULL;
}

NAV_SUBMAP_T *nav_algo_get_submap(int submap_x, int submap_y)
{
    return NULL;
}

static void nav_algo_queue_handler(component_handle_t component_type, uint8_t message_type, void* message_data, size_t message_size)
{
    if(!s_is_navigation_enabled)
    {
        ESP_LOGI(TAG, "navigation not enabled, ignoring.");
        return;
    }
    if(component_type == ToF_public_component && message_type == TOF_MSG_NEW_DEPTH_ARRAY)
    {
        nav_algo_check_tof_array_against_map((TOF_DATA_t*) message_data);
    }
    else if(component_type == imu_public_component && message_type == IMU_MSG_RAW_DATA)
    {

    }
}

static uint8_t nav_algo_convert_adjusted_confidence_value(uint16_t distance, uint8_t confidence)
{
    uint16_t square_val = (1000 / 6); //whatever the multiplier should be at 1000 mm
    uint16_t multiplier = 1 + ((distance * distance) / (square_val * square_val));
    uint16_t ret_val = multiplier * confidence;
    return (uint8_t) (ret_val & 0xFF);
}

//read tof map and run error vs existing map to estimate movement
//also create and adjust objects on each submap
static void nav_algo_check_tof_array_against_map(TOF_DATA_t* tof_data)
{
    
}
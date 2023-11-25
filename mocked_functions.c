#include "mocked_functions.h"

QueueHandle_t xQueueCreate(uint8_t queue_length, size_t queue_type)
{
    return NULL;
}

void xTaskCreate(void (*func_ptr)(void*), const char* name, size_t stack_depth, void* pvParams, uint8_t priority, void* handle)
{

}

bool xQueueSend(QueueHandle_t queue_ptr, void* message_info, TickType_t time_thing)
{
    return false;
}

bool xQueueReceive(QueueHandle_t queue_ptr, void* message_info, TickType_t time_thing)
{
    return false;
}

void vTaskDelay(TickType_t time_thing)
{
    printf("waited %u ms", time_thing);
}

esp_err_t mock_tof_read(uint8_t* TOF_OUT, uint8_t dat_size)
{
    return ESP_OK;
}

esp_err_t mock_tof_read_write(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size)
{
    return ESP_OK;
}

esp_err_t mock_tof_write(uint8_t* TOF_IN, uint8_t dat_size)
{
    return ESP_OK;
}

void mock_set_tof_read_return(uint8_t* read_buf, uint8_t dat_length)
{

}

esp_err_t nvs_flash_init_partition(const char* partition_name)
{
    return ESP_OK;
}

esp_err_t nvs_flash_erase_partition(const char* partition_name)
{
    return ESP_OK;
}

esp_err_t nvs_get_stats(const char* partition_name, nvs_stats_t* stats_handle)
{
    return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char* partition_name, const char* namespace_var, bool canWrite, nvs_handle_t* handle_ptr)
{
    return ESP_OK;
}

esp_err_t nvs_close(nvs_handle_t handle)
{
    return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t handle, const char* blob_name, uint32_t* serial_data, size_t* serial_size)
{
    return ESP_OK;
}

esp_err_t nvs_set_blob(nvs_handle_t handle, const char* blob_name, uint32_t* serial_data, size_t serial_size)
{
    return ESP_OK;
}
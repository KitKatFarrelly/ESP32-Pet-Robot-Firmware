#ifndef H_mocked_functions
#define H_mocked_functions

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#define portMAX_DELAY 10000 // technically infinite but who cares we're unit testing here

#define portTICK_PERIOD_MS 1

#define NVS_READONLY 0

#define NVS_READWRITE 1

typedef struct
{
    size_t total_entries;
    size_t used_entries;
    size_t free_entries;
} nvs_stats_t;

typedef enum
{
    ESP_OK = 0,
    ESP_ERROR_GENERIC,
    ESP_ERR_NVS_NO_FREE_PAGES,
    ESP_ERR_NVS_NEW_VERSION_FOUND,
    ESP_MAX,
} esp_err_t;

typedef uint8_t gpio_num_t;

typedef void* QueueHandle_t;

typedef uint16_t TickType_t;

typedef uint8_t nvs_handle_t;

#define ESP_LOGE(tag, format, ...); \
printf("[E] %s:", tag); \
printf(format, ##__VA_ARGS__);

#define ESP_LOGI(tag, format, ...); \
printf("[I] %s:", tag); \
printf(format, ##__VA_ARGS__);

void app_main(void);

QueueHandle_t xQueueCreate(uint8_t queue_length, size_t queue_type);

void xTaskCreate(void (*func_ptr)(void*), const char* name, size_t stack_depth, void* pvParams, uint8_t priority, void* handle);

bool xQueueSend(QueueHandle_t queue_ptr, void* message_info, TickType_t time_thing);

bool xQueueReceive(QueueHandle_t queue_ptr, void* message_info, TickType_t time_thing);

void vTaskDelay(TickType_t time_thing);

esp_err_t mock_tof_read(uint8_t* TOF_OUT, uint8_t dat_size);

esp_err_t mock_tof_read_write(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size);

esp_err_t mock_tof_write(uint8_t* TOF_IN, uint8_t dat_size);

void mock_set_tof_read_return(uint8_t* read_buf, uint8_t dat_length);

esp_err_t nvs_flash_init_partition(const char* partition_name);

esp_err_t nvs_flash_erase_partition(const char* partition_name);

esp_err_t nvs_get_stats(const char* partition_name, nvs_stats_t* stats_handle);

esp_err_t nvs_open_from_partition(const char* partition_name, const char* namespace_var, bool canWrite, nvs_handle_t* handle_ptr);

esp_err_t nvs_close(nvs_handle_t handle);

esp_err_t nvs_get_blob(nvs_handle_t handle, const char* blob_name, uint32_t* serial_data, size_t* serial_size);

esp_err_t nvs_set_blob(nvs_handle_t handle, const char* blob_name, uint32_t* serial_data, size_t serial_size);

#endif
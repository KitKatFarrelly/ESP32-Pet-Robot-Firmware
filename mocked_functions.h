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

typedef uint8_t TickType_t;

typedef uint8_t nvs_handle_t;

void app_main(void);

void* xQueueCreate(uint8_t queue_length, size_t queue_type);

#endif
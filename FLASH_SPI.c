#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "FLASH_SPI.h"

// log tag

static const char *TAG = "FLASH";

uint8_t FLASH_INIT_PARTITION(const char* partition_name)
{
    esp_err_t err = nvs_flash_init_partition(partition_name);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_LOGE(TAG, "Failed to initialize partition %s.", partition_name);
        return 1;
    }
    ESP_LOGI(TAG, "Partition %s Initialized.", partition_name);

    return 0;
}

uint8_t FLASH_ERASE_PARTITION(const char* partition_name)
{
    esp_err_t err = nvs_flash_erase_partition(partition_name);
    if (err != ESP_OK)
    {
        return 1;
    }

    return 0;
}

PARTITION_INFO_t FLASH_GET_PARTITION_INFO(const char* partition_name);
{
    PARTITION_INFO_t return_info;

    nvs_stats_t = stats_handle;
    esp_err_t err = nvs_get_stats(partition_name, &stats_handle);
    if (err != ESP_OK)
    {
        return NULL;
    }

    return_info.number_of_entries = stats_handle.total_entries;
    return_info.in_use_entries = stats_handle.used_entries;
    return_info.free_entries = stats_handle.available_entries;

    return return_info;
}

size_t FLASH_DOES_KEY_EXIST(const char* partition_name, const char* namespace, const char* blob_name)
{
    nvs_handle_t does_key_exist_handle;
    esp_err_t err;
    
    err = nvs_open_from_partition(partition_name, namespace, NVS_READONLY, &does_key_exist_handle);
    if (err != ESP_OK) 
    {
        nvs_close(does_key_exist_handle);
        ESP_LOGE(TAG, "unable to open partition.");
        return 0;
    }

    // Read the size of memory space required for blob
    size_t required_size = 0;  // value will default to 0, if not set yet in NVS
    err = nvs_get_blob(does_key_exist_handle, blob_name, NULL, &required_size);
    if (err != ESP_OK) 
    {
        nvs_close(does_key_exist_handle);
        ESP_LOGE(TAG, "key not found.");
        return 0;
    }
    nvs_close(does_key_exist_handle);

    return required_size;
}

uint8_t FLASH_WRITE_TO_BLOB(const char* partition_name, const char* namespace, const char* blob_name, void* data, size_t size)
{
    nvs_handle_t write_handle;
    esp_err_t err;
    
    err = nvs_open_from_partition(partition_name, namespace, NVS_READONLY, &write_handle);
    if (err != ESP_OK) 
    {
        nvs_close(write_handle);
        ESP_LOGE(TAG, "unable to open partition.");
        return 1;
    }

    // Read the size of memory space required for blob
    err = nvs_set_blob(write_handle, blob_name, data, &size);
    if (err != ESP_OK) 
    {
        nvs_close(write_handle);
        ESP_LOGE(TAG, "could not write to blob.");
        return 1;
    }
    nvs_close(write_handle);
    return 0;
}

void* FLASH_READ_FROM_BLOB(const char* partition_name, const char* namespace, const char* blob_name, size_t size)
{
    nvs_handle_t read_handle;
    esp_err_t err;
    void* read_data = malloc(sizeof(uint8_t)*size);
    
    err = nvs_open_from_partition(partition_name, namespace, NVS_READONLY, &read_handle);
    if (err != ESP_OK) 
    {
        nvs_close(read_handle);
        ESP_LOGE(TAG, "unable to open partition.");
        return NULL;
    }

    // Read the size of memory space required for blob
    err = nvs_get_blob(read_handle, blob_name, NULL, &size);
    if (err != ESP_OK) 
    {
        nvs_close(read_handle);
        ESP_LOGE(TAG, "could not read from blob.");
        return NULL;
    }
    nvs_close(read_handle);
    return read_data;
}

const char* FLASH_CREATE_FILE(const char* partition_name, const char* blob_name, unsigned long size)
{
    return NULL;
}

uint8_t FLASH_DELETE_FILE(const char* partition_name)
{
    return 0;
}

void* FLASH_GET_FILE_INFO(const char* partition_name)
{
    return NULL;
}

uint8_t FLASH_WRITE_TO_FILE(const char* partition_name, unsigned long offset, void* data, unsigned long size)
{
    return 0;
}

void* FLASH_READ_FROM_FILE(const char* partition_name, unsigned long offset, unsigned long size)
{
    return NULL;
}
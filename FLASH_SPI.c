#include <stdio.h>
#include <inttypes.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#endif

#include "FLASH_SPI.h"

// log tag

static const char *TAG = "FLASH";

static uint32_t* flash_serialize(uint8_t* input_pointer, size_t input_size);
static uint8_t* flash_deserialize(uint32_t* input_pointer, size_t input_size, size_t output_size);

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

PARTITION_INFO_t FLASH_GET_PARTITION_INFO(const char* partition_name)
{
    PARTITION_INFO_t return_info;
    return_info.number_of_entries = 0;
    return_info.in_use_entries = 0;
    return_info.free_entries = 0;

    nvs_stats_t stats_handle;
    esp_err_t err = nvs_get_stats(partition_name, &stats_handle);
    if (err != ESP_OK)
    {
        return return_info;
    }

    return_info.number_of_entries = stats_handle.total_entries;
    return_info.in_use_entries = stats_handle.used_entries;
    return_info.free_entries = stats_handle.free_entries;

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

uint8_t FLASH_WRITE_TO_BLOB(const char* partition_name, const char* namespace, const char* blob_name, uint8_t* data, size_t size)
{
    nvs_handle_t write_handle;
    esp_err_t err;

    uint32_t* serialized_data = flash_serialize(data, size);
    
    err = nvs_open_from_partition(partition_name, namespace, NVS_READWRITE, &write_handle);
    if (err != ESP_OK) 
    {
        nvs_close(write_handle);
        ESP_LOGE(TAG, "unable to open partition.");
        return 1;
    }

    // Read the size of memory space required for blob
    err = nvs_set_blob(write_handle, blob_name, serialized_data, size);
    if (err != ESP_OK) 
    {
        nvs_close(write_handle);
        ESP_LOGE(TAG, "could not write to blob.");
        return 1;
    }
    nvs_close(write_handle);
    free(serialized_data);
    return 0;
}

uint8_t* FLASH_READ_FROM_BLOB(const char* partition_name, const char* namespace, const char* blob_name, size_t size)
{
    nvs_handle_t read_handle;
    esp_err_t err;

    size_t serial_size = (size / 4);
    if(size % 4)
    {
        serial_size++;
    }

    uint32_t* serial_data = malloc(sizeof(uint32_t)*serial_size);
    
    err = nvs_open_from_partition(partition_name, namespace, NVS_READONLY, &read_handle);
    if (err != ESP_OK) 
    {
        nvs_close(read_handle);
        ESP_LOGE(TAG, "unable to open partition.");
        return NULL;
    }

    // Read the size of memory space required for blob
    err = nvs_get_blob(read_handle, blob_name, serial_data, &size);
    if (err != ESP_OK) 
    {
        nvs_close(read_handle);
        ESP_LOGE(TAG, "could not read from blob.");
        return NULL;
    }
    nvs_close(read_handle);

    uint8_t* read_data = flash_deserialize(serial_data, serial_size, size);

    free(serial_data);
    return read_data;
}

static uint32_t* flash_serialize(uint8_t* input_pointer, size_t input_size)
{
    size_t output_size = (input_size / 4);
    if(input_size % 4)
    {
        output_size++;
    }

    uint32_t* output_pointer = malloc(sizeof(uint32_t)*output_size);

    size_t out_iter = 0;

    for(size_t in_iter = 0; in_iter < input_size; in_iter++)
    {
        if(out_iter >= output_size)
        {
            break;
        }
        else
        {
            output_pointer[out_iter] += (input_pointer[in_iter] << (8 * (in_iter % 4)));
            ESP_LOGI(TAG, "byte serialized: %lx at position %zu", output_pointer[out_iter], (in_iter % 4));
        }
        if(in_iter % 4 == 3)
        {
            out_iter++;
        }
    }

    return output_pointer;
}

static uint8_t* flash_deserialize(uint32_t* input_pointer, size_t input_size, size_t output_size)
{
    uint8_t* output_pointer = malloc(sizeof(uint8_t)*output_size);

    size_t out_iter = 0;

    for(size_t in_iter = 0; in_iter < input_size; in_iter++)
    {
        for(uint8_t j = 0; j < 4; j++)
        {
            if(out_iter >= output_size)
            {
                break;
            }
            else
            {
                uint8_t byte_deserialized = ((input_pointer[in_iter] & (0xFF << (8 * j))) >> (8 * j));
                ESP_LOGI(TAG, "byte deserialized: %x.", byte_deserialized);
                output_pointer[out_iter] = byte_deserialized;
            }
            out_iter++;
        }
    }

    return output_pointer;
}

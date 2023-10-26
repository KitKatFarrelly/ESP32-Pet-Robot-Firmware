#include <stdio.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "FLASH_SPI.h"

#define STORAGE_NAMESPACE "storage"

uint8_t FLASH_INIT_PARTITION(const char* partition_name)
{
    esp_err_t err = nvs_flash_init_partition(partition_name);
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        return 1;
    }

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

    return return_info;
}

uint8_t FLASH_WRITE_TO_PARTITION(const char* partition_name, unsigned long offset, void* data, unsigned long size)
{
    return 0;
}

void* FLASH_READ_FROM_PARTITION(const char* partition_name, unsigned long offset, unsigned long size)
{

}

const char* FLASH_CREATE_FILE(const char* partition_name, const char* blob_name, unsigned long size)
{

}

uint8_t FLASH_DELETE_FILE(const char* partition_name)
{
    return 0;
}

void* FLASH_GET_FILE_INFO(const char* partition_name)
{

}

uint8_t FLASH_WRITE_TO_FILE(const char* partition_name, unsigned long offset, void* data, unsigned long size)
{
    return 0;
}

void* FLASH_READ_FROM_FILE(const char* partition_name, unsigned long offset, unsigned long size)
{

}
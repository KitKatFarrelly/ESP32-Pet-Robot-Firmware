#ifndef H_FLASH_DRVR
#define H_FLASH_DRVR

typedef struct
{
    const char* name;
    unsigned long size;
} FLASH_PARTITION_t;

FLASH_PARTITION_t FLASH_CREATE_PARTITION(unsigned long size);

uint8_t FLASH_DELETE_PARTITION(FLASH_PARTITION_t handle);

void* FLASH_GET_PARTITION_INFO(FLASH_PARTITION_t handle);

uint8_t FLASH_WRITE_TO_PARTITION(FLASH_PARTITION_t handle, unsigned long offset, void* data, unsigned long size);

void* FLASH_READ_FROM_PARTITION(FLASH_PARTITION_t handle, unsigned long offset, unsigned long size);

FLASH_PARTITION_t FLASH_CREATE_FILE(unsigned long size);

uint8_t FLASH_DELETE_FILE(FLASH_PARTITION_t handle);

void* FLASH_GET_FILE_INFO(FLASH_PARTITION_t handle);

uint8_t FLASH_WRITE_TO_FILE(FLASH_PARTITION_t handle, unsigned long offset, void* data, unsigned long size);

void* FLASH_READ_FROM_FILE(FLASH_PARTITION_t handle, unsigned long offset, unsigned long size);

#endif
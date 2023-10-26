#ifndef H_FLASH_DRVR
#define H_FLASH_DRVR

#define MAX_NUMBER_OF_BLOBS 100

typedef struct
{
    uint8_t number_of_entries;
    char blob_names[MAX_NUMBER_OF_BLOBS][16];
    unsigned long size;
} PARTITION_INFO_t;

// NVS STORAGE FUNCTIONS

uint8_t FLASH_INIT_PARTITION(const char* partition_name);

uint8_t FLASH_ERASE_PARTITION(const char* partition_name);

PARTITION_INFO_t FLASH_GET_PARTITION_INFO(const char* partition_name);

uint8_t FLASH_WRITE_TO_PARTITION(const char* partition_name, const char* blob_name, void* data, unsigned long size);

void* FLASH_READ_FROM_PARTITION(const char* partition_name, const char* blob_name, unsigned long size);

// FAT FILESYSTEM FUNCTIONS

char* FLASH_CREATE_FILE(const char* partition_name, unsigned long size);

uint8_t FLASH_DELETE_FILE(const char* partition_name);

void* FLASH_GET_FILE_INFO(const char* partition_name);

uint8_t FLASH_WRITE_TO_FILE(const char* partition_name, unsigned long offset, void* data, unsigned long size);

void* FLASH_READ_FROM_FILE(const char* partition_name, unsigned long offset, unsigned long size);

#endif
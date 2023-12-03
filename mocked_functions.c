#include "mocked_functions.h"

#define MAX_TASK_REGISTRATIONS 10

#define MAX_BLOBS 10

typedef struct
{
    void (*func_ptr)(void*);
    char* name;
    size_t stack_depth;
    void* pvParams;
    uint8_t priority;
    void* handle;
} TaskType_t;

typedef struct
{
    uint32_t* blob;
    size_t blob_size;
    char* blob_name;
} BlobType_t;

static TaskType_t task_array[MAX_TASK_REGISTRATIONS] = {0};

static uint8_t* TOF_read_data = NULL;

static nvs_handle_t current_handle = 0;

static bool can_write = 0;

static BlobType_t blob_array[MAX_BLOBS] = {0};

//static function defs

static uint8_t getTaskFromName(const char* name);

// functions for testing purposes

bool spinQueueTaskOnce(const char* name)
{
    uint8_t task_array_iterator = getTaskFromName(name);
    if(task_array_iterator == MAX_TASK_REGISTRATIONS)
    {
        return false;
    }
    printf("spinning task %s", task_array[task_array_iterator].name);
    (*(task_array[task_array_iterator].func_ptr))(NULL);
    return true;
}

bool deleteTask(const char* name)
{
    uint8_t task_array_iterator = getTaskFromName(name);
    if(task_array_iterator == MAX_TASK_REGISTRATIONS)
    {
        return false;
    }
    printf("deleting task %s", task_array[task_array_iterator].name);
    task_array[task_array_iterator].func_ptr = NULL;
    free(task_array[task_array_iterator].name);
    task_array[task_array_iterator].name = NULL;
    task_array[task_array_iterator].stack_depth = 0;
    task_array[task_array_iterator].pvParams = NULL;
    task_array[task_array_iterator].priority = 0;
    task_array[task_array_iterator].handle = NULL;
    return true;
}

void deleteQueue(QueueHandle_t handle)
{
    //TODO: this should delete all pending queue messages
    printf("deleting queue at %p", handle);
    free(handle);
}

bool setTOFReadVal(const uint8_t* read_data, size_t size)
{
    if((size * sizeof(uint8_t)) > sizeof(read_data))
    {
        return false;
    }
    TOF_read_data = malloc(size * sizeof(uint8_t));
    memcpy(TOF_read_data, read_data, size * sizeof(uint8_t));
    return true;
}

static uint8_t getTaskFromName(const char* name)
{
    printf("searching for task %s", name);
    for(int i = 0; i < MAX_TASK_REGISTRATIONS; i++)
    {
        if(!strcmp(name, task_array[i].name))
        {
            printf("found task %s at iter %u", name, i);
            break;
        }
    }
    return MAX_TASK_REGISTRATIONS;
}

// mocked functions

QueueHandle_t xQueueCreate(uint8_t queue_length, size_t queue_type)
{
    QueueHandle_t newQueue = malloc(sizeof(QueueType_t));
    newQueue->message_queue_start = NULL;
    newQueue->data_type_length = queue_type;
    newQueue->queue_length = queue_length;
    printf("created queue at %p", newQueue);
    return newQueue;
}

void xTaskCreate(void (*func_ptr)(void*), const char* name, size_t stack_depth, void* pvParams, uint8_t priority, void* handle)
{
    for(int i = 0; i < MAX_TASK_REGISTRATIONS; i++)
    {
        if(task_array[i].func_ptr == NULL)
        {
            task_array[i].func_ptr = func_ptr;
            task_array[i].name = malloc(sizeof(name) + 1);
            strcpy(task_array[i].name, name);
            task_array[i].stack_depth = stack_depth;
            task_array[i].pvParams = pvParams;
            task_array[i].priority = priority;
            task_array[i].handle = handle;
            printf("created task %s", task_array[i].name);
            break;
        }
    }
}

bool xQueueSend(QueueHandle_t queue_ptr, void* message_info, TickType_t time_thing)
{
    if(queue_ptr == NULL)
    {
        return false;
    }
    message_node_t* lastNode;
    if(queue_ptr->message_queue_start != NULL)
    {
        uint8_t queue_length = 0;
        lastNode = queue_ptr->message_queue_start;
        while(lastNode->next_node != NULL)
        {
            if(queue_length >= queue_ptr->queue_length)
            {
                return false;
            }
            lastNode = lastNode->next_node;
            queue_length++;
        }
        lastNode->next_node = malloc(sizeof(message_node_t));
        lastNode = lastNode->next_node;
    }
    else
    {
        queue_ptr->message_queue_start = malloc(sizeof(message_node_t));
        lastNode = queue_ptr->message_queue_start;
    }
    lastNode->message_queue = malloc(sizeof(queue_ptr->data_type_length));
    memcpy(lastNode->message_queue, message_info, sizeof(queue_ptr->data_type_length));
    lastNode->next_node = NULL;
    printf("appended message at %p", lastNode);
    return true;
}

bool xQueueReceive(QueueHandle_t queue_ptr, void* message_info, TickType_t time_thing)
{
    if(queue_ptr == NULL)
    {
        return false;
    }
    if(queue_ptr->message_queue_start == NULL)
    {
        return false;
    }
    printf("received message at %p", queue_ptr->message_queue_start);
    message_node_t* nextNode = queue_ptr->message_queue_start->next_node;
    memcpy(message_info, queue_ptr->message_queue_start->message_queue, sizeof(queue_ptr->data_type_length));
    free(queue_ptr->message_queue_start->message_queue);
    free(queue_ptr->message_queue_start);
    queue_ptr->message_queue_start = nextNode;
    return true;
}

void vTaskDelay(TickType_t time_thing)
{
    printf("waited %u ms", time_thing);
}

esp_err_t mock_tof_read(uint8_t* TOF_OUT, uint8_t dat_size)
{
    memcpy(TOF_OUT, TOF_read_data, dat_size * sizeof(uint8_t));
    return ESP_OK;
}

esp_err_t mock_tof_read_write(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size)
{
    printf("the following bytes were written to TOF");
    for(uint8_t i = 0; i < in_dat_size; i++)
        {
            printf("%x", TOF_IN[i]);
        }
    memcpy(TOF_OUT, TOF_read_data, out_dat_size * sizeof(uint8_t));
    return ESP_OK;
}

esp_err_t mock_tof_write(uint8_t* TOF_IN, uint8_t dat_size)
{
    printf("the following bytes were written to TOF");
    for(uint8_t i = 0; i < dat_size; i++)
    {
        printf("%x", TOF_IN[i]);
    }
    return ESP_OK;
}

esp_err_t nvs_flash_init_partition(const char* partition_name)
{
    printf("Initialized %s", partition_name);
    return ESP_OK;
}

esp_err_t nvs_flash_erase_partition(const char* partition_name)
{
    printf("Erasing %s", partition_name);
    for(int i = 0; i < MAX_BLOBS; i++)
    {
        free(blob_array[i].blob_name);
        free(blob_array[i].blob);
        blob_array[i].blob_size = 0;
    }
    return ESP_OK;
}

esp_err_t nvs_get_stats(const char* partition_name, nvs_stats_t* stats_handle)
{
    printf("Printing stats for %s", partition_name);
    stats_handle = NULL;
    return ESP_OK;
}

esp_err_t nvs_open_from_partition(const char* partition_name, const char* namespace_var, bool canWrite, nvs_handle_t* handle_ptr)
{
    printf("Opening for %s with namespace %s", partition_name, namespace_var);
    current_handle++;
    if(current_handle > 99)
    {
        current_handle = 0;
    }
    can_write = canWrite;
    *handle_ptr = current_handle;
    return ESP_OK;
}

esp_err_t nvs_close(nvs_handle_t handle)
{
    if(handle != current_handle)
    {
        return ESP_ERROR_GENERIC;
    }
    return ESP_OK;
}

esp_err_t nvs_get_blob(nvs_handle_t handle, const char* blob_name, uint32_t* serial_data, size_t* serial_size)
{
    if(handle != current_handle)
    {
        return ESP_ERROR_GENERIC;
    }
    for(int i = 0; i < MAX_BLOBS; i++)
    {
        if(!strcmp(blob_name, blob_array[i].blob_name))
        {
            printf("found blob %s at iter %u", blob_array[i].blob_name, i);
            if(serial_data != NULL)
            {
                serial_data = blob_array[i].blob;
            }
            *serial_size = blob_array[i].blob_size;
            return ESP_OK;
        }
    }
    return ESP_ERR_NVS_NOT_FOUND;
}

esp_err_t nvs_set_blob(nvs_handle_t handle, const char* blob_name, uint32_t* serial_data, size_t serial_size)
{
    if(handle != current_handle)
    {
        return ESP_ERROR_GENERIC;
    }
    if(!can_write)
    {
        return ESP_ERR_NVS_READ_ONLY;
    }
    uint8_t lowest_empty_blob = 0;
    for(int i = 0; i < MAX_BLOBS; i++)
    {
        if(!strcmp(blob_name, blob_array[i].blob_name))
        {
            printf("found blob %s at iter %u", blob_array[i].blob_name, i);
            free(blob_array[i].blob);
            free(blob_array[i].blob_name);
            lowest_empty_blob = i;
            break;
        }
        else if(lowest_empty_blob == i)
        {
            if(blob_array[i].blob_size > 0)
            {
                lowest_empty_blob++;
            }
        }
    }
    if(lowest_empty_blob < MAX_BLOBS)
    {
        printf("writing blob %s at iter %u", blob_name, lowest_empty_blob);
        blob_array[lowest_empty_blob].blob = malloc(((serial_size / 4) + 1) * sizeof(uint32_t));
        memcpy(blob_array[lowest_empty_blob].blob, serial_data, serial_size);
        blob_array[lowest_empty_blob].blob_name = malloc(sizeof(blob_name) + 1);
        strcpy(blob_array[lowest_empty_blob].blob_name, blob_name);
        blob_array[lowest_empty_blob].blob_size = serial_size;
        return ESP_OK;
    }
    return ESP_ERROR_GENERIC;
}
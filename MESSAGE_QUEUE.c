#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef FUNCTIONAL_TESTS
#include "mocked_functions.h"
#else
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_system.h"
#endif

#include "MESSAGE_QUEUE.h"

#define MAX_COMPONENT_REGISTRATIONS 10

struct callback_node_t
{
    void (*callback_ptr)(component_handle_t, uint8_t, void*, size_t);
    struct callback_node_t* next_handler;
    callback_handle_t callback_handle;
};

typedef struct callback_node_t callback_handler_t;

typedef struct
{
    callback_handler_t* callback_list_start;
    bool is_component_registered;
} component_handler_t;

//static const char *TAG = "MSG_QUEUE"; //Should use in future

static void normal_queue_loop(void* args);
static void priority_queue_loop(void* args);
static bool is_handle_registered(component_handle_t handle);

static bool normal_queue_active = false;
static bool priority_queue_active = false;

static uint8_t queue_handle_cnt = 0;
static uint8_t lowest_unregistered_queue_handle = 0;

static QueueHandle_t normal_message_queue = NULL;
static QueueHandle_t priority_message_queue = NULL;

static component_handler_t normal_queue_handlers[MAX_COMPONENT_REGISTRATIONS] = {0};
static component_handler_t priority_queue_handlers[MAX_COMPONENT_REGISTRATIONS] = {0};

void MESSAGE_QUEUE_INIT(void)
{
    normal_message_queue = xQueueCreate(MESSAGE_QUEUE_LENGTH, sizeof(message_info_t));
    normal_queue_active = true;
    xTaskCreate(normal_queue_loop, "normal_queue", 2048, NULL, 5, NULL);
}

void PRIORITY_MESSAGE_QUEUE_INIT(void)
{
    priority_message_queue = xQueueCreate(MESSAGE_QUEUE_LENGTH, sizeof(message_info_t));
    priority_queue_active = true;
    xTaskCreate(priority_queue_loop, "priority_queue", 2048, NULL, 10, NULL);
}

void uninit_queue(uint8_t queuetype)
{
    switch (queuetype)
    {
    case 0:
        normal_queue_active = false;
#ifdef FUNCTIONAL_TESTS
        deleteTask("normal_queue");
        deleteQueue(normal_message_queue);
#endif
        break;
    case 1:
        priority_queue_active = false;
#ifdef FUNCTIONAL_TESTS
        deleteTask("priority_queue");
        deleteQueue(priority_message_queue);
#endif
        break;
    default:
        break;
    }
}

bool check_is_queue_active(uint8_t queuetype)
{
    switch (queuetype)
    {
    case 0:
        return normal_queue_active;
    case 1:
        return priority_queue_active;
    default:
        return false;
    }
}

uint8_t clear_all_handles(void)
{
    for(int i = 0; i < queue_handle_cnt; i++)
    {
        if(delete_handle_for_component((component_handle_t) i))
        {
            return 1;
        }
    }
    queue_handle_cnt = 0;
    lowest_unregistered_queue_handle = 0;
    return 0;
}

uint8_t create_handle_for_component(component_handle_t* handle)
{
    if(!normal_queue_active && !priority_queue_active)
    {
        return 1;
    }
    if(queue_handle_cnt >= MAX_COMPONENT_REGISTRATIONS)
    {
        return 2;
    }
    if(lowest_unregistered_queue_handle > queue_handle_cnt)
    {
        return 3;
    }
    *handle = lowest_unregistered_queue_handle;
    if(normal_queue_active)
    {
        normal_queue_handlers[lowest_unregistered_queue_handle].is_component_registered = true;
        normal_queue_handlers[lowest_unregistered_queue_handle].callback_list_start = NULL;
    }
    if(priority_queue_active)
    {
        priority_queue_handlers[lowest_unregistered_queue_handle].is_component_registered = true;
        priority_queue_handlers[lowest_unregistered_queue_handle].callback_list_start = NULL;
    }
    queue_handle_cnt++;
    for(component_handle_t i = 0; is_handle_registered(i) && i < queue_handle_cnt; i++)
    {
        lowest_unregistered_queue_handle = i + 1;
    }
    return 0;
}

uint8_t delete_handle_for_component(component_handle_t handle)
{
    callback_handler_t* handler_ptr = NULL;
    callback_handler_t* next_ptr = NULL;
    if(!normal_queue_active && !priority_queue_active)
    {
        return 1;
    }
    if(normal_queue_active)
    {
        handler_ptr = normal_queue_handlers[handle].callback_list_start;
        while(handler_ptr != NULL)
        {
            next_ptr = handler_ptr->next_handler;
            free(handler_ptr);
            handler_ptr = next_ptr;
        }
        normal_queue_handlers[handle].is_component_registered = false;
    }
    if(priority_queue_active)
    {
        handler_ptr = priority_queue_handlers[handle].callback_list_start;
        while(handler_ptr != NULL)
        {
            next_ptr = handler_ptr->next_handler;
            free(handler_ptr);
            handler_ptr = next_ptr;
        }
        priority_queue_handlers[handle].is_component_registered = false;
    }
    if(handle < lowest_unregistered_queue_handle)
    {
        lowest_unregistered_queue_handle = handle;
    }
    queue_handle_cnt--;
    return 0;
}

callback_handle_t register_component_handler_for_messages(void (*func_ptr)(component_handle_t, uint8_t, void*, size_t), component_handle_t handle)
{
    callback_handle_t return_handle = 0;
    if(normal_queue_handlers[handle].is_component_registered && normal_queue_active)
    {
        callback_handler_t* new_handler = malloc(sizeof(callback_handler_t));
        callback_handler_t* handler_iter = normal_queue_handlers[handle].callback_list_start;
        if(handler_iter == NULL)
        {
            normal_queue_handlers[handle].callback_list_start = new_handler;
            return_handle = 1;
        }
        else
        {
            callback_handler_t* prev_handler_iter = NULL;
            while((handler_iter) != NULL)
            {
                return_handle = handler_iter->callback_handle;
                prev_handler_iter = handler_iter;
                handler_iter = handler_iter->next_handler;
            }
            if(prev_handler_iter == NULL)
            {
                //should maybe panic here but I'm just going to return failure for now
                free(new_handler);
                return 0;
            }
            prev_handler_iter->next_handler = new_handler;
            return_handle++;
        }
        new_handler->callback_handle = return_handle;
        new_handler->callback_ptr = func_ptr;
        new_handler->next_handler = NULL;
    }
    return return_handle;
}

uint8_t unregister_component_handler_for_messages(component_handle_t handle, callback_handle_t function_handle)
{
    callback_handler_t* handler_iter = normal_queue_handlers[handle].callback_list_start;
    callback_handler_t* prev_handler_iter = NULL;
    if(handler_iter == NULL || !normal_queue_active) return 1;
    while(handler_iter != NULL)
    {
        if(handler_iter->callback_handle == function_handle)
        {
            if(prev_handler_iter != NULL)
            {
                prev_handler_iter->next_handler = handler_iter->next_handler;
            }
            else
            {
                normal_queue_handlers[handle].callback_list_start = handler_iter->next_handler;
            }
            free(handler_iter);
            return 0;
        }
        prev_handler_iter = handler_iter;
        handler_iter = handler_iter->next_handler;
    }
    return 1;
}

uint8_t send_message_to_normal_queue(message_info_t message_info)
{
    if(!normal_queue_active) return 1;
    xQueueSend(normal_message_queue, &message_info, ( TickType_t ) 0);
    return 0;
}

callback_handle_t register_priority_handler_for_messages(void (*func_ptr)(component_handle_t, uint8_t, void*, size_t), component_handle_t handle)
{
    callback_handle_t return_handle = 0;
    if(priority_queue_handlers[handle].is_component_registered && priority_queue_active)
    {
        callback_handler_t* new_handler = malloc(sizeof(callback_handler_t));
        callback_handler_t* handler_iter = priority_queue_handlers[handle].callback_list_start;
        if(handler_iter == NULL)
        {
            priority_queue_handlers[handle].callback_list_start = new_handler;
            return_handle = 1;
        }
        else
        {
            callback_handler_t* prev_handler_iter = NULL;
            while((handler_iter) != NULL)
            {
                return_handle = handler_iter->callback_handle;
                prev_handler_iter = handler_iter;
                handler_iter = handler_iter->next_handler;
            }
            if(prev_handler_iter == NULL)
            {
                //should maybe panic here but I'm just going to return failure for now
                free(new_handler);
                return 0;
            }
            prev_handler_iter->next_handler = new_handler;
            return_handle++;
        }
        new_handler->callback_handle = return_handle;
        new_handler->callback_ptr = func_ptr;
        new_handler->next_handler = NULL;
    }
    return return_handle;
}

uint8_t unregister_priority_handler_for_messages(component_handle_t handle, callback_handle_t function_handle)
{
    callback_handler_t* handler_iter = priority_queue_handlers[handle].callback_list_start;
    callback_handler_t* prev_handler_iter = NULL;
    if(handler_iter == NULL || !priority_queue_active) return 1;
    while(handler_iter != NULL)
    {
        if(handler_iter->callback_handle == function_handle)
        {
            if(prev_handler_iter != NULL)
            {
                prev_handler_iter->next_handler = handler_iter->next_handler;
            }
            else
            {
                priority_queue_handlers[handle].callback_list_start = handler_iter->next_handler;
            }
            free(handler_iter);
            return 0;
        }
        prev_handler_iter = handler_iter;
        handler_iter = handler_iter->next_handler;
    }
    return 1;
}

uint8_t send_message_to_priority_queue(message_info_t message_info)
{
    if(!priority_queue_active) return 1;
    xQueueSend(priority_message_queue, &message_info, ( TickType_t ) 0);
    return 0;
}

static void normal_queue_loop(void* args)
{
    message_info_t message_info;
    while(normal_queue_active)
    {
        if (xQueueReceive(normal_message_queue, &message_info, portMAX_DELAY))
        {
            component_handler_t component_handler = normal_queue_handlers[message_info.component_handle];
            callback_handler_t* callback_iter = NULL;
            if(component_handler.is_component_registered)
            {
                callback_iter = component_handler.callback_list_start;
            }
            while(callback_iter != NULL)
            {
                (*(callback_iter->callback_ptr))(message_info.component_handle, message_info.message_type, message_info.message_data, message_info.message_size);
                callback_iter = callback_iter->next_handler;
            }
            if(message_info.message_data != NULL)
            {
                free(message_info.message_data);
            }
        }
        else
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        if(args != NULL)
        {
            break;
        }
    }
}

static void priority_queue_loop(void* args)
{
    message_info_t message_info;
    while(priority_queue_active)
    {
        if (xQueueReceive(priority_message_queue, &message_info, portMAX_DELAY))
        {
            component_handler_t component_handler = priority_queue_handlers[message_info.component_handle];
            callback_handler_t* callback_iter = NULL;
            if(component_handler.is_component_registered)
            {
                callback_iter = component_handler.callback_list_start;
            }
            while(callback_iter != NULL)
            {
                (*(callback_iter->callback_ptr))(message_info.component_handle, message_info.message_type, message_info.message_data, message_info.message_size);
                callback_iter = callback_iter->next_handler;
            }
            if(message_info.message_data != NULL)
            {
                free(message_info.message_data);
            }
        }
        else
        {
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        if(args != NULL)
        {
            break;
        }
    }
}

static bool is_handle_registered(component_handle_t handle)
{
    return (normal_queue_handlers[handle].is_component_registered || priority_queue_handlers[handle].is_component_registered);
}

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"

#include "MESSAGE_QUEUE.h"

#define MAX_COMPONENT_REGISTRATIONS 10

typedef struct
{
    callback_handle_t callback_handle;
    void (*callback_ptr)(uint8_t);
    callback_handler_t* next_handler;
} callback_handler_t;

typedef struct
{
    bool is_component_registered;
    callback_handler_t* callback_list_start;
} component_handler_t;

static const char *TAG = "MSG_QUEUE";

static void normal_queue_loop();
static void priority_queue_loop();
static bool is_handle_registered(component_handle_t handle);

static bool normal_queue_active = false;
static bool priority_queue_active = true;

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
        delete_handle_for_component((component_handle_t) i);
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
    if(queue_handle_cnt < MAX_COMPONENT_REGISTRATIONS)
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
    for(component_handle_t i = 0; is_handle_registered(i) && i <= queue_handle_cnt; i++)
    {
        lowest_unregistered_queue_handle = i;
    }
    return 0;
}

uint8_t delete_handle_for_component(component_handle_t handle)
{
    callback_handler_t* handler_ptr = NULL;
    callback_handler_t* next_ptr = NULL;
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
    lowest_unregistered_queue_handle = handle;
    queue_handle_cnt--;
}

callback_handle_t register_component_handler_for_messages(void (*func_ptr)(uint8_t), component_handle_t handle)
{

}

uint8_t unregister_component_handler_for_messages(component_handle_t handle, callback_handle_t function_handle)
{

}

uint8_t send_message_to_normal_queue(message_info_t message_info)
{

}

callback_handle_t register_priority_handler_for_messages(void (*func_ptr)(uint8_t), component_handle_t handle)
{

}

uint8_t unregister_priority_handler_for_messages(component_handle_t handle, callback_handle_t function_handle)
{

}

uint8_t send_message_to_priority_queue(message_info_t message_info)
{

}

static void normal_queue_loop()
{

}

static void priority_queue_loop()
{

}

static bool is_handle_registered(component_handle_t handle)
{

}
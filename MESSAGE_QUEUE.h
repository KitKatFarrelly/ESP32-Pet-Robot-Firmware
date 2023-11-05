#ifndef H_MESSAGE_QUEUE
#define H_MESSAGE_QUEUE

#define MESSAGE_QUEUE_LENGTH 100

typedef struct
{
    component_handle_t component_handle;
    uint8_t message_type; //message_type should be casted from an enum
    void* message_data;
} message_info_t;

typedef uint8_t component_handle_t;

typedef uint8_t callback_handle_t;

void MESSAGE_QUEUE_INIT(void);

void PRIORITY_MESSAGE_QUEUE_INIT(void);

bool check_is_queue_active(uint8_t queuetype);

uint8_t create_handle_for_component(component_handle_t* handle);

uint8_t delete_handle_for_component(component_handle_t handle);

callback_handle_t register_component_handler_for_messages(void (*func_ptr)(uint8_t), component_handle_t handle);

uint8_t unregister_component_handler_for_messages(component_handle_t handle, callback_handle_t function_handle);

uint8_t send_message_to_normal_queue(message_info_t message_info);

callback_handle_t register_priority_handler_for_messages(void (*func_ptr)(uint8_t), component_handle_t handle);

uint8_t unregister_priority_handler_for_messages(component_handle_t handle, callback_handle_t function_handle);

uint8_t send_message_to_priority_queue(message_info_t message_info);

#endif

#ifndef H_MESSAGE_QUEUE
#define H_MESSAGE_QUEUE

void MESSAGE_QUEUE_INIT(void);

void PRIORITY_MESSAGE_QUEUE_INIT(void);

bool check_is_queue_active(uint8_t queuetype);

uint8_t register_component_handler_for_messages(void (*func_ptr)(uint8_t), uint8_t* handle);

uint8_t unregister_component_handler_for_messages(uint8_t handle);

uint8_t register_priority_handler_for_messages(void (*func_ptr)(uint8_t), uint8_t* handle);

uint8_t unregister_priority_handler_for_messages(uint8_t handle);

#endif
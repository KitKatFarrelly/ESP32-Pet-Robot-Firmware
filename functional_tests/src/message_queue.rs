fn testMessageHandlerOne()
{
    //this should be used to handle test functions
}

fn testMessageHandlerTwo()
{
    //this should be used to handle test functions
}

fn testMessageHandlerThree()
{
    //this should be used to handle test functions
}

pub fn initMessageQueue()
{
    unsafe
    {
        crate::MESSAGE_QUEUE_INIT();
    }
}

pub fn initPriorityMessageQueue()
{
    unsafe
    {
        crate::PRIORITY_MESSAGE_QUEUE_INIT();
    }
}

pub fn checkQueueActive(u8: queueType) -> bool
{
    let retVal = unsafe { crate::check_is_queue_active(queueType); };
    retVal
}

pub fn clearMessageQueueHandles(void) -> u8
{
    let retVal = unsafe { crate::clear_all_handles(); };
    retVal
}

pub fn generateTestComponentHandle() -> component_handle_t
{
    let retComp: component_handle_t = 0;
    unsafe { crate::create_handle_for_component(retComp.as_ptr() as *component_handle_t); };
    retComp
}

pub fn removeTestComponentHandle(compHandle: component_handle_t) -> u8
{
    let retVal unsafe { crate::delete_handle_for_component(compHandle); };
    retVal
}

//TODO

callback_handle_t register_component_handler_for_messages(void (*func_ptr)(component_handle_t, uint8_t, void*), component_handle_t handle);

uint8_t unregister_component_handler_for_messages(component_handle_t handle, callback_handle_t function_handle);

uint8_t send_message_to_normal_queue(message_info_t message_info);

callback_handle_t register_priority_handler_for_messages(void (*func_ptr)(component_handle_t, uint8_t, void*), component_handle_t handle);

uint8_t unregister_priority_handler_for_messages(component_handle_t handle, callback_handle_t function_handle);

uint8_t send_message_to_priority_queue(message_info_t message_info);
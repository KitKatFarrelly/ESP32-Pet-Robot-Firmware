unsafe extern "C" fn testMessageHandlerOne(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void)
{
    //this should be used to handle test functions
}

unsafe extern "C" fn testMessageHandlerTwo(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void)
{
    //this should be used to handle test functions
}

unsafe extern "C" fn testMessageHandlerThree(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void)
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

pub fn registerTestHandlerNormal(handler: u8, compHandle: component_handle_t) -> callback_handle_t
{
    match(handler)
    {
        1|_ => let funcPtr = testMessageHandlerOne;
        2 => let funcPtr = testMessageHandlerTwo;
        3 => let funcPtr = testMessageHandlerThree;
    }
    let retCall = unsafe { crate::register_component_handler_for_messages(funcPtr, compHandle); };
    retCall
}

pub fn unregisterTestHandlerNormal(compHandle: component_handle_t, callHandle: callback_handle_t) -> u8
{
    let retVal = unsafe { unregister_component_handler_for_messages(compHandle, callHandle); };
    retVal
}

pub fn createNewMessageNormal(type: u8, compHandle: component_handle_t, data: &str) -> u8
{
    let mut mutData = message_info_t
    {
        message_type = type;
        component_handle = compHandle;
        message_data = data.as_ptr() as *mut c_void;
    };

    let retVal = unsafe { send_message_to_normal_queue(mutData); };
    retVal
}

pub fn registerTestHandlerPriority(handler: u8, compHandle: component_handle_t) -> callback_handle_t
{
    match(handler)
    {
        1|_ => let funcPtr = testMessageHandlerOne;
        2 => let funcPtr = testMessageHandlerTwo;
        3 => let funcPtr = testMessageHandlerThree;
    }
    let retCall = unsafe { crate::register_priority_handler_for_messages(funcPtr, compHandle); };
    retCall
}

pub fn unregisterTestHandlerPriority(compHandle: component_handle_t, callHandle: callback_handle_t) -> u8
{
    let retVal = unsafe { unregister_priority_handler_for_messages(compHandle, callHandle); };
    retVal
}

pub fn createNewMessagePriority(type: u8, compHandle: component_handle_t, data: &str) -> u8
{
    let mut mutData = message_info_t
    {
        message_type = type;
        component_handle = compHandle;
        message_data = data.as_ptr() as *mut c_void;
    };

    let retVal = unsafe { send_message_to_priority_queue(mutData); };
    retVal
}

#[cfg(test)]
mod tests
{
    use super::*;
}
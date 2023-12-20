use crate::component_handle_t;
use crate::message_info_t;
use crate::callback_handle_t;
use std::slice;
use std::str;

static mut lastCompHandleOne: component_handle_t = 0;
static mut lastMsgTypeOne: u8 = 0;
static mut lastStrDatOne: &str = "";

unsafe extern "C" fn testMessageHandlerOne(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void, msg_size: usize)
{
    lastCompHandleOne = compHandle;
    lastMsgTypeOne = msg_type;
    lastStrDatOne = str::from_utf8(slice::from_raw_parts(msg_data as *const u8, msg_size)).unwrap();
}

static mut lastCompHandleTwo: component_handle_t = 0;
static mut lastMsgTypeTwo: u8 = 0;
static mut lastStrDatTwo: &str = "";

unsafe extern "C" fn testMessageHandlerTwo(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void, msg_size: usize)
{
    lastCompHandleTwo = compHandle;
    lastMsgTypeTwo = msg_type;
    lastStrDatTwo = str::from_utf8(slice::from_raw_parts(msg_data as *const u8, msg_size)).unwrap();
}

static mut lastCompHandleThree: component_handle_t = 0;
static mut lastMsgTypeThree: u8 = 0;
static mut lastStrDatThree: &str = "";

unsafe extern "C" fn testMessageHandlerThree(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void, msg_size: usize)
{
    lastCompHandleThree = compHandle;
    lastMsgTypeThree = msg_type;
    lastStrDatThree = str::from_utf8(slice::from_raw_parts(msg_data as *const u8, msg_size)).unwrap();
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

pub fn checkQueueActive(queueType: u8) -> bool
{
    let retVal = unsafe { crate::check_is_queue_active(queueType) };
    retVal
}

pub fn clearMessageQueueHandles() -> u8
{
    let retVal = unsafe { crate::clear_all_handles() };
    retVal
}

pub fn generateTestComponentHandle() -> component_handle_t
{
    let mut retComp: component_handle_t = 0;
    unsafe { crate::create_handle_for_component(&mut retComp as *mut component_handle_t); };
    retComp
}

pub fn removeTestComponentHandle(compHandle: component_handle_t) -> u8
{
    let retVal = unsafe { crate::delete_handle_for_component(compHandle) };
    retVal
}

pub fn registerTestHandlerNormal(handler: u8, compHandle: component_handle_t) -> callback_handle_t
{
    let mut funcPtr: Option<unsafe extern "C" fn(u8, u8, *mut ::std::os::raw::c_void, usize)> = None;
    
    match handler
    {
        1 => funcPtr = Some(testMessageHandlerOne),
        2 => funcPtr = Some(testMessageHandlerTwo),
        3 => funcPtr = Some(testMessageHandlerThree),
        _ => funcPtr = None,
    }
    let retCall = unsafe { crate::register_component_handler_for_messages(funcPtr, compHandle) };
    retCall
}

pub fn unregisterTestHandlerNormal(compHandle: component_handle_t, callHandle: callback_handle_t) -> u8
{
    let retVal = unsafe { crate::unregister_component_handler_for_messages(compHandle, callHandle) };
    retVal
}

pub fn createNewMessageNormal(msg_type: u8, compHandle: component_handle_t, data: &str) -> u8
{
    let mut mutData = message_info_t
    {
        message_type: msg_type,
        component_handle: compHandle,
        message_data: data.as_ptr() as *mut ::std::os::raw::c_void,
        message_size: data.len(),
    };

    let retVal = unsafe { crate::send_message_to_normal_queue(mutData) };
    retVal
}

pub fn registerTestHandlerPriority(handler: u8, compHandle: component_handle_t) -> callback_handle_t
{
    let mut funcPtr: Option<unsafe extern "C" fn(u8, u8, *mut ::std::os::raw::c_void, usize)> = None;
    
    match handler
    {
        1 => funcPtr = Some(testMessageHandlerOne),
        2 => funcPtr = Some(testMessageHandlerTwo),
        3 => funcPtr = Some(testMessageHandlerThree),
        _ => funcPtr = None,
    }
    let retCall = unsafe { crate::register_priority_handler_for_messages(funcPtr, compHandle) };
    retCall
}

pub fn unregisterTestHandlerPriority(compHandle: component_handle_t, callHandle: callback_handle_t) -> u8
{
    let retVal = unsafe { crate::unregister_priority_handler_for_messages(compHandle, callHandle) };
    retVal
}

pub fn createNewMessagePriority(msg_type: u8, compHandle: component_handle_t, data: &str) -> u8
{
    let mut mutData = message_info_t
    {
        message_type: msg_type,
        component_handle: compHandle,
        message_data: data.as_ptr() as *mut ::std::os::raw::c_void,
        message_size: data.len(),
    };

    let retVal = unsafe { crate::send_message_to_priority_queue(mutData) };
    retVal
}

#[cfg(test)]
mod tests
{
    use super::*;

    #[test]
    fn test_normal_queue()
    {

    }

    #[test]
    fn test_priority_queue()
    {
        
    }

    #[test]
    fn test_both_queues()
    {
        
    }
}
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

pub fn generateTestComponentHandle() -> (component_handle_t, u8)
{
    let mut retComp: component_handle_t = 0;
    let error = unsafe { crate::create_handle_for_component(&mut retComp as *mut component_handle_t) };
    (retComp, error)
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

pub fn createNewMessageNormal(msg_type: u8, compHandle: component_handle_t, data: *mut ::std::os::raw::c_void, len: usize) -> u8
{
    let mutData = message_info_t
    {
        message_type: msg_type,
        component_handle: compHandle,
        message_data: data,
        message_size: len,
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

pub fn createNewMessagePriority(msg_type: u8, compHandle: component_handle_t, data: *mut ::std::os::raw::c_void, len: usize) -> u8
{
    let mutData = message_info_t
    {
        message_type: msg_type,
        component_handle: compHandle,
        message_data: data,
        message_size: len,
    };

    let retVal = unsafe { crate::send_message_to_priority_queue(mutData) };
    retVal
}

#[cfg(test)]
mod tests
{
    use super::*;

    fn spin_normal_queue_once() -> bool
    {
        let queue_type = "normal_queue\0".as_ptr() as *const i8;
        let retVal = unsafe { crate::spinQueueTaskOnce(queue_type) };
        retVal
    }

    fn spin_priority_queue_once() -> bool
    {
        let queue_type = "priority_queue\0".as_ptr() as *const i8;
        let retVal = unsafe { crate::spinQueueTaskOnce(queue_type) };
        retVal
    }

    #[test]
    fn test_normal_queue()
    {
        let testMsg: &str = "test Message\0";
        let strPtr = testMsg.as_ptr() as *const i8;
        let testPtr = unsafe{ crate::createVoidPtr(strPtr, testMsg.len()) };
        let msg_type: u8 = 3;
        initMessageQueue();
        assert_eq!(checkQueueActive(0), true);
        assert_eq!(checkQueueActive(1), false);
        let (testComponent, error) = generateTestComponentHandle();
        assert_eq!(error, 0);
        let testCallbackOne = registerTestHandlerNormal(1, testComponent);
        let testCallbackTwo = registerTestHandlerNormal(2, testComponent);
        let testCallbackThree = registerTestHandlerNormal(3, testComponent);
        createNewMessageNormal(msg_type, testComponent, testPtr, testMsg.len());
        assert_eq!(spin_normal_queue_once(), true);
        unsafe
        {
            assert_eq!(testMsg, lastStrDatOne);
            assert_eq!(msg_type, lastMsgTypeOne);
            assert_eq!(testComponent, lastCompHandleOne);
        }
        unsafe
        {
            assert_eq!(testMsg, lastStrDatTwo);
            assert_eq!(msg_type, lastMsgTypeTwo);
            assert_eq!(testComponent, lastCompHandleTwo);
        }
        unsafe
        {
            assert_eq!(testMsg, lastStrDatThree);
            assert_eq!(msg_type, lastMsgTypeThree);
            assert_eq!(testComponent, lastCompHandleThree);
        }
        unregisterTestHandlerNormal(testComponent, testCallbackOne);
        unregisterTestHandlerNormal(testComponent, testCallbackTwo);
        unregisterTestHandlerNormal(testComponent, testCallbackThree);
        removeTestComponentHandle(testComponent);
        unsafe{ crate::uninit_queue(0) };
    }

    #[test]
    fn test_priority_queue()
    {
        let testMsg: &str = "test Message\0";
        let strPtr = testMsg.as_ptr() as *const i8;
        let testPtr = unsafe{ crate::createVoidPtr(strPtr, testMsg.len()) };
        let msg_type: u8 = 3;
        initPriorityMessageQueue();
        assert_eq!(checkQueueActive(1), true);
        assert_eq!(checkQueueActive(0), false);
        let (testComponent, error) = generateTestComponentHandle();
        assert_eq!(error, 0);
        let testCallbackOne = registerTestHandlerPriority(1, testComponent);
        let testCallbackTwo = registerTestHandlerPriority(2, testComponent);
        let testCallbackThree = registerTestHandlerPriority(3, testComponent);
        createNewMessagePriority(msg_type, testComponent, testPtr, testMsg.len());
        assert_eq!(spin_priority_queue_once(), true);
        unsafe
        {
            assert_eq!(testMsg, lastStrDatOne);
            assert_eq!(msg_type, lastMsgTypeOne);
            assert_eq!(testComponent, lastCompHandleOne);
        }
        unsafe
        {
            assert_eq!(testMsg, lastStrDatTwo);
            assert_eq!(msg_type, lastMsgTypeTwo);
            assert_eq!(testComponent, lastCompHandleTwo);
        }
        unsafe
        {
            assert_eq!(testMsg, lastStrDatThree);
            assert_eq!(msg_type, lastMsgTypeThree);
            assert_eq!(testComponent, lastCompHandleThree);
        }
        unregisterTestHandlerPriority(testComponent, testCallbackOne);
        unregisterTestHandlerPriority(testComponent, testCallbackTwo);
        unregisterTestHandlerPriority(testComponent, testCallbackThree);
        removeTestComponentHandle(testComponent);
        unsafe{ crate::uninit_queue(1) };
    }

    #[test]
    fn test_both_queues()
    {
        let testMsgN: &str = "test Message N\0";
        let strPtrN = testMsgN.as_ptr() as *const i8;
        let testPtrN = unsafe{ crate::createVoidPtr(strPtrN, testMsgN.len()) };
        let testMsgP: &str = "test Message P\0";
        let strPtrP = testMsgP.as_ptr() as *const i8;
        let testPtrP = unsafe{ crate::createVoidPtr(strPtrP, testMsgP.len()) };
        let msg_type: u8 = 3;
        initMessageQueue();
        initPriorityMessageQueue();
        assert_eq!(checkQueueActive(0), true);
        assert_eq!(checkQueueActive(1), true);
        let (testComponent, error) = generateTestComponentHandle();
        assert_eq!(error, 0);
        let testCallbackOne = registerTestHandlerPriority(1, testComponent);
        let testCallbackTwo = registerTestHandlerPriority(2, testComponent);
        let testCallbackThree = registerTestHandlerNormal(3, testComponent);
        createNewMessageNormal(msg_type, testComponent, testPtrN, testMsgN.len());
        createNewMessagePriority(msg_type, testComponent, testPtrP, testMsgP.len());
        assert_eq!(spin_normal_queue_once(), true);
        assert_eq!(spin_priority_queue_once(), true);
        unsafe
        {
            assert_eq!(testMsgP, lastStrDatOne);
            assert_eq!(msg_type, lastMsgTypeOne);
            assert_eq!(testComponent, lastCompHandleOne);
        }
        unsafe
        {
            assert_eq!(testMsgP, lastStrDatTwo);
            assert_eq!(msg_type, lastMsgTypeTwo);
            assert_eq!(testComponent, lastCompHandleTwo);
        }
        unsafe
        {
            assert_eq!(testMsgN, lastStrDatThree);
            assert_eq!(msg_type, lastMsgTypeThree);
            assert_eq!(testComponent, lastCompHandleThree);
        }
        unregisterTestHandlerPriority(testComponent, testCallbackOne);
        unregisterTestHandlerPriority(testComponent, testCallbackTwo);
        unregisterTestHandlerNormal(testComponent, testCallbackThree);
        removeTestComponentHandle(testComponent);
        unsafe{ crate::uninit_queue(0) };
        unsafe{ crate::uninit_queue(1) };
    }
}
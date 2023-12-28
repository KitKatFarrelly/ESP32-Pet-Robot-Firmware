use crate::component_handle_t;
use crate::message_info_t;
use crate::callback_handle_t;
use crate::TOF_DATA_t;
use std::mem;
use std::slice;
use crate::spi_flash;
use crate::message_queue;

static mut ToFCompHandle: component_handle_t = 0;
static mut ToFMsgType: u8 = 0;
static mut TofArrayData: &[TOF_DATA_t] = &[];

unsafe extern "C" fn ToFMessageHandler(compHandle: component_handle_t, msg_type: u8, msg_data: *mut ::std::os::raw::c_void, msg_size: usize)
{
    ToFCompHandle = compHandle;
    ToFMsgType = msg_type;
    TofArrayData = slice::from_raw_parts(msg_data as *const TOF_DATA_t, mem::size_of::<TOF_DATA_t>());
}

pub fn appendNewTOFSensorReturn(dat: &[u8])
{
    let test_ptr = dat.as_ptr() as *const u8; // and a pointer, created from the reference
    let ret_bool;
    for x in dat
    {
        print!("{} ", x);
    }
    println!("");
    unsafe
    {
        ret_bool = crate::setTOFReadVal(test_ptr, dat.len());
    }
    println!("Append Data returned {}.", ret_bool);
}

pub fn tofInitialize()
{
    unsafe{ crate::TOF_INIT() };
}

pub fn tofLoadConfig(config: u8) -> u8
{
    let retVal = unsafe{ crate::TOF_LOAD_CONFIG(config) };
    retVal
}

pub fn tofResetSensor() -> u8
{
    let retVal = unsafe{ crate::TOF_RESET() };
    retVal
}

pub fn tofFactoryCalibration() -> u8
{
    //Need to append all the test factory calibration data first
    let retVal = unsafe{ crate::TOF_STORE_FACTORY_CALIBRATION() };
    retVal
}

pub fn tofStoreFactoryCalibration() -> u8
{
    let retVal = unsafe{ crate::TOF_STORE_FACTORY_CALIBRATION() };
    retVal
}

pub fn tofLoadFactoryCalibration() -> u8
{
    let retVal = unsafe{ crate::TOF_LOAD_FACTORY_CALIBRATION() };
    retVal
}

pub fn tofReturnCalibrationStatus() -> u8
{
    let retVal = unsafe{ crate::TOF_RETURN_CALIBRATION_STATUS() };
    retVal
}

pub fn tofStartMeasurements() -> u8
{
    let retVal = unsafe{ crate::TOF_START_MEASUREMENTS() };
    retVal
}

pub fn tofStopMeasurements() -> u8
{
    let retVal = unsafe{ crate::TOF_STOP_MEASUREMENTS() };
    retVal
}

pub fn tofSpinISROnce(gpio_num: u8) -> bool
{
    let retVal = unsafe{ crate::spinISROnce(gpio_num) };
    retVal
}

/* I don't really see the need to test these but they exist I guess
esp_err_t TOF_READ(uint8_t* TOF_OUT, uint8_t dat_size);

esp_err_t TOF_READ_WRITE(uint8_t* TOF_OUT, uint8_t out_dat_size, uint8_t* TOF_IN, uint8_t in_dat_size);

esp_err_t TOF_WRITE(uint8_t* TOF_IN, uint8_t dat_size);
*/
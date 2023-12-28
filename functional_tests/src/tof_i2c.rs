extern crate rand;

use crate::component_handle_t;
use crate::message_info_t;
use crate::callback_handle_t;
use crate::TOF_DATA_t;
use std::mem;
use std::slice;
use rand::Rng;
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
        print!("{:#04x} ", x);
    }
    println!("length = {:#04x}", dat.len());
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
    let retVal = unsafe{ crate::TOF_FACTORY_CALIBRATION() };
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

pub fn tofSwitchTofMode(tmf_8828_mode: bool) -> bool
{
    let retVal = unsafe{ crate::TOF_SET_TMF8828_MODE(tmf_8828_mode) };
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

#[cfg(test)]
mod tests
{
    use super::*;

    static tof_partition: &str = "factory\0";
    static tof_namespace: &str = "tof\0";
    static tof_fac_cal: &str = "tmf8821_fac\0";

    fn createRandomFactoryCalData() -> Vec<u8>
    {
        let mut rng = rand::thread_rng();

        let mut header: Vec<u8> = vec![0x19, 0, 0xC0, 0];

        let mut vals: Vec<u8> = (0..0xBC).map(|_| rng.gen()).collect();

        header.append(&mut vals);

        assert_eq!(header.len(), 0xC0);

        header
    }

    fn createRandomMeasurementDataFrame() -> Vec<u8>
    {
        let mut rng = rand::thread_rng();

        let vals: Vec<u8> = (0..16).map(|_| rng.gen()).collect();

        vals
    }

    #[test]
    fn test_factory_calibration_tmf8821()
    {
        //Initialize
        let mut test_data: [u8; 3] = [0; 3];
        test_data[0] = 0x41;
        appendNewTOFSensorReturn(&test_data[..1]);
        test_data[0] = 0x03;
        appendNewTOFSensorReturn(&test_data[..3]);
        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        test_data[0] = 0x08;
        appendNewTOFSensorReturn(&test_data[..1]);
        tofInitialize();
        //Switch Mode
        test_data[0] = 0x08;
        appendNewTOFSensorReturn(&test_data[..1]);
        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        assert_eq!(tofSwitchTofMode(false), false);

        //Start Factory Calibration
        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        tofFactoryCalibration();

        //Append Factory Calibration data here
        let fac_cal = createRandomFactoryCalData();

        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        appendNewTOFSensorReturn(&fac_cal[..0x40]);
        appendNewTOFSensorReturn(&fac_cal[0x40..0x80]);
        appendNewTOFSensorReturn(&fac_cal[0x80..]);

        //Store Factory Calibration
        tofStoreFactoryCalibration();

        //Check Stored Data is equal to fac_cal
        let output_vec = spi_flash::readBlobFromKey(tof_partition, tof_namespace, tof_fac_cal, fac_cal.len());
        assert_eq!(output_vec, fac_cal);

        //Load Factory Calibration
        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        tofLoadFactoryCalibration();

        //Check Calibration Status
        test_data[0] = 0x00;
        appendNewTOFSensorReturn(&test_data[..1]);
        assert_eq!(tofReturnCalibrationStatus(), 0);
    }

    fn test_factory_calibration_tmf8828()
    {
        //tofInitialize();
        //tofSwitchTofMode(true);
        let fac_cal_1 = createRandomFactoryCalData();
        let fac_cal_2 = createRandomFactoryCalData();
        let fac_cal_3 = createRandomFactoryCalData();
        let fac_cal_4 = createRandomFactoryCalData();
    }

    #[test]
    fn test_collect_measurements_tmf8821()
    {
        //tofInitialize();
        //tofSwitchTofMode(false);
    }

    #[test]
    fn test_collect_measurements_tmf8828()
    {
        //tofInitialize();
        //tofSwitchTofMode(true);
    }
}

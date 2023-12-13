#![allow(non_upper_case_globals)]
#![allow(non_camel_case_types)]
#![allow(non_snake_case)]

mod spi_flash;
mod message_queue;
mod tof_i2c;

include!("bindings.rs");

fn appendNewTOFReadVal(dat: &[u8], size: usize)
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
        ret_bool = setTOFReadVal(test_ptr, size);
    }
    println!("Append Data returned {}.", ret_bool);
}

fn main()
{
    println!("Beginning running tests.");
    let mut test_data: [u8; 3] = [0; 3];
    test_data[0] = 0x41;
    appendNewTOFReadVal(&test_data[..1], 1); // a shared reference to this data...
    test_data[0] = 0x03;
    appendNewTOFReadVal(&test_data[..3], 3);
    test_data[0] = 0x00;
    appendNewTOFReadVal(&test_data[..1], 1);
    test_data[0] = 0x08;
    appendNewTOFReadVal(&test_data[..1], 1);
    unsafe
    {
        app_main();
    }
}

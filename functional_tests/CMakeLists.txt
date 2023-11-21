cmake_minimum_required(VERSION 3.16)

project(unit_tests C)

add_definitions(-DFUNCTIONAL_TESTS)

add_library(unit_test_lib STATIC
../mocked_functions.h
../mocked_functions.c
../tof_bin_image.h
../tof_bin_image.c
../LED_DRVR.h
../LED_DRVR.c
../IMU_SPI.h
../IMU_SPI.c
../ToF_I2C.h
../ToF_I2C.c
../MTR_DRVR.h
../MTR_DRVR.c
../MESSAGE_QUEUE.h
../MESSAGE_QUEUE.c
../FLASH_SPI.h
../FLASH_SPI.c
../UART_CMDS.h
../UART_CMDS.c
../ROBOT_APP.c
../bindgen_wrapper.h
)

install(TARGETS unit_test_lib DESTINATION .)

cmake_minimum_required(VERSION 3.16)

project(unit_tests C)

add_library(unit_test_lib STATIC
..\bindgen_wrapper.h
)

install(TARGETS unit_test_lib DESTINATION .)

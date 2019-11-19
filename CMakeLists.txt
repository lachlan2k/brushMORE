cmake_minimum_required(VERSION 3.7)

set(AVR_MCU atmega8)
set(AVR_H_FUSE 0xca)
set(AVR_L_FUSE 0x3f)
set(MCU_SPEED "16000000UL")

include(cmake/toolchain.cmake)

project(brushMORE)

include_directories(include)

file(GLOB_RECURSE CXX_SOURCES "src/*.cpp")
file(GLOB_RECURSE C_SOURCES "src/*.c")

add_definitions("-DF_CPU=${MCU_SPEED}")

# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/firmware)

add_avr_executable(brushMORE ${CXX_SOURCES} ${C_SOURCES})
# avr_target_link_libraries(brushMORE)

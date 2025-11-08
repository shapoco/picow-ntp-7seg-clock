cmake_minimum_required(VERSION 3.12)

set(APP_NAME display_test)
set(APP_INC_DIR ${CMAKE_CURRENT_LIST_DIR}/include)
set(APP_SRC_DIR ${CMAKE_CURRENT_LIST_DIR}/src)
set(SUBMODULE_DIR ${CMAKE_CURRENT_LIST_DIR}/../submodules)

set(VLCFG_DIR ${SUBMODULE_DIR}/vlconfig/cpp/lib)

# Pull in PICO SDK (must be before project)
set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
include(${PICO_SDK_PATH}/pico_sdk_init.cmake)

project(${APP_NAME} C CXX ASM)

# Initialize the SDK
pico_sdk_init()

add_executable(${APP_NAME})
add_subdirectory(${VLCFG_DIR} vlcfg)

target_compile_options(${APP_NAME} PUBLIC -O2 -Wall)

pico_generate_pio_header(${APP_NAME} ${APP_SRC_DIR}/seg7_pwm_tx.pio)

file(GLOB CPP_FILES
    ${APP_SRC_DIR}/*.cpp
    ${APP_SRC_DIR}/*.c
)

target_sources(${APP_NAME} PRIVATE 
    ${CPP_FILES}
)

target_include_directories(${APP_NAME} PRIVATE
    ${APP_INC_DIR}
    ${VLCFG_DIR}/include
)

target_link_libraries(${APP_NAME} PRIVATE
    pico_stdlib
    pico_multicore
    pico_cyw43_arch_lwip_poll
    #pico_cyw43_arch_lwip_threadsafe_background
    hardware_adc
    hardware_dma
    vlcfg
)

pico_enable_stdio_usb(${APP_NAME} 1)
pico_enable_stdio_uart(${APP_NAME} 0)

pico_add_extra_outputs(${APP_NAME})

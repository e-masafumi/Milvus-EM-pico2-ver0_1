cmake_minimum_required(VERSION 3.13)
set(PICO_BOARD pico2)

include(pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()

add_executable(main
	main.cpp
	func-pwm.cpp
	func-i2c.cpp
	func-uart.cpp
	func-usbuart.cpp
	func-MS5837-02BA.cpp
	func-BNO055.cpp
	func-INA228.cpp
	hw_config.c
	pio_uart.cpp
)

pico_generate_pio_header(main ${CMAKE_CURRENT_LIST_DIR}/uart_rx.pio)

add_subdirectory(lib/no-OS-FatFs/src)

pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 0)

target_link_libraries(main 
	pico_stdlib 
	pico_multicore 
	hardware_pwm 
	hardware_i2c 
	no-OS-FatFS-SD-SDIO-SPI-RPi-Pico
	hardware_uart 
	hardware_pio
	hardware_dma
	hardware_irq
)  

pico_add_extra_outputs(main)


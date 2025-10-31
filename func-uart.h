#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#ifndef FUNC_PICO_UART_H
#define FUNC_PICO_UART_H


class pico_uart{
	public:
//	void on_uart_rx();
	int setup(uart_inst_t *uartPort, uint uartBaudrate, uint dataBit, uint stopBit);
//		int writeDirect(i2c_inst_t *i2cPort, uint8_t address, uint8_t data, size_t len);
//		int writeOneByte(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t data);
//		int writeMultiByte(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t data[], size_t len);
//		int read(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t *data, size_t len);
};

#endif

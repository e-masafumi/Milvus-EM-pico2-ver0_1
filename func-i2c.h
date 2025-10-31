#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"


#ifndef FUNC_PICO_I2C_H
#define FUNC_PICO_I2C_H

class pico_i2c{
	public:
		int setup(i2c_inst_t *i2cPort, uint i2cBaudrate);
		int writeDirect(i2c_inst_t *i2cPort, uint8_t address, uint8_t data, size_t len);
		int writeOneByte(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t data);
		int writeMultiByte(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t data[], size_t len);
		int read(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t *data, size_t len);
};

#endif

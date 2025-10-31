#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "func-i2c.h"

#ifndef FUNC_INA228_H
#define FUNC_INA228_H

class INA228: public pico_i2c{
	public:
		int setup(i2c_inst_t *i2cPort);
		int readCurVolPow(i2c_inst_t *i2cPort, double *current, double *voltage, double *power);
//		int readTempPress(i2c_inst_t *i2CPort, double *temp, double *press);
};

#endif

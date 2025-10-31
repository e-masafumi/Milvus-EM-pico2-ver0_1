#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "func-i2c.h"

#ifndef FUNC_MS5837_02BA_H
#define FUNC_MS5837_02BA_H

class MS5837_02BA : public pico_i2c{
	public:
		int setup(i2c_inst_t *i2cPort);
		int readTempPress(i2c_inst_t *i2CPort, double *temp, double *press);
};

#endif

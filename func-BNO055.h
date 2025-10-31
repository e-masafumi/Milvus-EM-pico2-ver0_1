#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "func-i2c.h"

#ifndef FUNC_BNO055_H
#define FUNC_BNO055_H

class BNO055 : public pico_i2c{
	public:
		int setup(i2c_inst_t *i2cPort);
		int setMode(i2c_inst_t *i2cPort);
		int readAccel(i2c_inst_t *i2cPort, double *x, double *y, double *z);
		int readMag(i2c_inst_t *i2cPort, double *x, double *y, double *z);
		int readGyro(i2c_inst_t *i2cPort, double *x, double *y, double *z);
		int readQuaternion(i2c_inst_t *i2cPort, double *qw, double *qx, double *qy, double *qz);
};

#endif

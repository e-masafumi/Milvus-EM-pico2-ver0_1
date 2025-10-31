#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "func-i2c.h"


int pico_i2c::setup(i2c_inst_t *i2cPort, uint i2cBaudrate){
	int check=0;

 	check = i2c_init(i2cPort, i2cBaudrate);
	if(i2cBaudrate == check){
		return 0;
	}
	else{
		return -1;
	}
}

int pico_i2c::writeDirect(i2c_inst_t *i2cPort, uint8_t address, uint8_t data, size_t len){
	i2c_write_blocking(i2cPort, address, &data, len, false);
	return 0;
}

int pico_i2c::writeOneByte(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t data){
	uint8_t buff[2]={regAddress, data};
	i2c_write_blocking(i2cPort, devAddress, buff, 2, false);

// following method doesn't work. Why?
//	i2c_write_blocking(i2cPort, devAddress, &regAddress, 1, true);
//	i2c_write_blocking(i2cPort, devAddress, &data, len, false);
	return 0;
}	


int pico_i2c::writeMultiByte(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t data[], size_t len){
	uint8_t buff[len+1];
	buff[0] = regAddress;
	for (int i=1; i<=len; i++){
		buff[i] = data[i-1];
	}
	i2c_write_blocking(i2cPort, devAddress, buff, len+1, false);
//	i2c_write_blocking(i2cPort, devAddress, &regAddress, len, false);
//	for(uint8_t i=1; i<=len; i++){
//			uint8_t buf=data[i];
//	i2c_write_blocking(i2cPort, devAddress, data, len, false);
//	}
	return 0;
}

int pico_i2c::read(i2c_inst_t *i2cPort, uint8_t devAddress, uint8_t regAddress, uint8_t *data, size_t len){
	i2c_write_blocking(i2cPort, devAddress, &regAddress, 1, true);
	i2c_read_blocking(i2cPort, devAddress, data, len, false);
	return 0;
}

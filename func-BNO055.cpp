#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "Adafruit_BNO055_addressMap.h"
#include "func-i2c.h"
#include "func-BNO055.h"

pico_i2c i2c_BNO055;

int BNO055::setup(i2c_inst_t *i2cPort){
	uint8_t readBuff=0;
	uint8_t idCheckCnt=0;
//	i2c_BNO055.write(i2cPort, BNO055_ADDRESS_A)

//ID_CHECK
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_CHIP_ID_ADDR, &readBuff, 1);
	printf("BNO055_CHIP_ID = 0x%x\n", readBuff);
	if(readBuff == 0xa0){
		idCheckCnt++;
	}
	else{
		idCheckCnt = idCheckCnt+10;
	}
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_REV_ID_ADDR, &readBuff, 1);
	printf("BNO055_ACCEL_REV_ID_ADDR = 0x%x\n", readBuff);
	if(readBuff == 0xfb){
		idCheckCnt++;
	}
	else{
		idCheckCnt = idCheckCnt+10;
	}

	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_MAG_REV_ID_ADDR, &readBuff, 1);
	printf("BNO055_MAG_REV_ID_ADDR = 0x%x\n", readBuff);
	if(readBuff == 0x32){
		idCheckCnt++;
	}
	else{
		idCheckCnt = idCheckCnt+10;
	}
	
	
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_GYRO_REV_ID_ADDR, &readBuff, 1);
	printf("BNO055_GYRO_REV_ID_ADDR = 0x%x\n", readBuff);
	if(readBuff == 0x0f){
		idCheckCnt++;
	}
	else{
		idCheckCnt = idCheckCnt+10;
	}
	
	if(idCheckCnt==4){
		printf("BNO ID Check OK\n");
	}
	else if(idCheckCnt >= 10){
		printf("ID Check NG\n");
		sleep_ms(10000);
	}
	else{
		printf("ID Check ERROR\n");
		sleep_ms(10000);
	}
	printf("\n");

//setting
	printf("\n+++++++++++++++++++++++++++++++++++++++++++++++++++++\n");
	i2c_BNO055.writeOneByte(i2cPort, BNO055_ADDRESS_A, BNO055_PWR_MODE_ADDR, POWER_MODE_NORMAL);
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_PWR_MODE_ADDR, &readBuff, 1);
	printf("BNO055_PWR_MODE_ADDR = 0x%b\n", readBuff);
	
	i2c_BNO055.writeOneByte(i2cPort, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, OPERATION_MODE_CONFIG);
	sleep_ms(20); //min. 19ms
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, &readBuff, 1);
	printf("BNO055_OPR_MODE_ADDR = 0x%b BEFORE INIT\n", readBuff);

	i2c_BNO055.writeOneByte(i2cPort, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, OPERATION_MODE_NDOF);
	sleep_ms(10); //min. 7ms
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_OPR_MODE_ADDR, &readBuff, 1);
	printf("BNO055_OPR_MODE_ADDR = 0x%b AFTER INIT\n", readBuff);


	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_UNIT_SEL_ADDR, &readBuff, 1);
	printf("BNO055_UNIT_SEL_ADDR = 0x%b BEFORE INIT\n", readBuff);
	i2c_BNO055.writeOneByte(i2cPort, BNO055_ADDRESS_A, BNO055_UNIT_SEL_ADDR, 0b10000000);	//[m/s^2], [degree/s], degrees,Celsius, Android Format
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_UNIT_SEL_ADDR, &readBuff, 1);
	printf("BNO055_UNIT_SEL_ADDR INIT_FIN = 0x%b AFTER INIT\n", readBuff);

	return 0;
}


int BNO055::readAccel(i2c_inst_t *i2cPort, double *x, double *y, double *z){	
	uint8_t buff[6]={0,0,0,0,0,0};
	int16_t xBin, yBin, zBin;

/*	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_X_LSB_ADDR, &buff[0], 1);
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_X_MSB_ADDR, &buff[1], 1);
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_Y_LSB_ADDR, &buff[2], 1);
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_Y_MSB_ADDR, &buff[3], 1);
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_Z_LSB_ADDR, &buff[4], 1);
	i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_Z_MSB_ADDR, &buff[5], 1);*/
	for(int i=0;i<=5;i++){
		i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_ACCEL_DATA_X_LSB_ADDR+(0x01*i), &buff[i], 1);
	}
	xBin = buff[0] | buff[1] << 8;
	yBin = buff[2] | buff[3] << 8;
	zBin = buff[4] | buff[5] << 8;

	*x = (double)xBin/100;
	*y = (double)yBin/100;
	*z = (double)zBin/100;


	return 0;
}


int BNO055::readMag(i2c_inst_t *i2cPort, double *x, double *y, double *z){	
	uint8_t buff[6]={0,0,0,0,0,0};
	int16_t xBin, yBin, zBin;

	for(int i=0;i<=5;i++){
		i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_MAG_DATA_X_LSB_ADDR+(0x01*i), &buff[i], 1);
	}
	xBin = buff[0] | buff[1] << 8;
	yBin = buff[2] | buff[3] << 8;
	zBin = buff[4] | buff[5] << 8;

	*x = (double)xBin/16;
	*y = (double)yBin/16;
	*z = (double)zBin/16;

	return 0;
}

int BNO055::readGyro(i2c_inst_t *i2cPort, double *x, double *y, double *z){	
	uint8_t buff[6]={0,0,0,0,0,0};
	int16_t xBin, yBin, zBin;

	for(int i=0;i<=5;i++){
		i2c_BNO055.read(i2cPort, BNO055_ADDRESS_A, BNO055_GYRO_DATA_X_LSB_ADDR+(0x01*i), &buff[i], 1);
	}
	xBin = buff[0] | buff[1] << 8;
	yBin = buff[2] | buff[3] << 8;
	zBin = buff[4] | buff[5] << 8;

	*x = (double)xBin/16;
	*y = (double)yBin/16;
	*z = (double)zBin/16;

	return 0;
}

int BNO055::readQuaternion(i2c_inst_t *i2cPort, double *qw, double *qx, double *qy, double *qz){
	

	return 0;
}


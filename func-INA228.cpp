#include <stdio.h>
#include <math.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "func-i2c.h"
#include "func-INA228.h"

const uint8_t INA228_ADDR = 0b01000000;
const uint8_t INA228_CONFIG = 0x00;
const uint8_t INA228_ADC_CONFIG = 0x01;
const uint8_t INA228_MAN_ID = 0x3e;
const uint8_t INA228_DEV_ID = 0x3f;
const uint8_t INA228_CUR = 0x07;
const uint8_t INA228_VOL = 0x05;
const uint8_t INA228_POW = 0x08;
const uint8_t INA228_DIAG_ALRT = 0x0B;
const double LSBVOL = 195.3125;
const double LSBCUR_HIGH = 156.25;

pico_i2c i2c_INA228;

int INA228::setup(i2c_inst_t *i2cPort){
	uint8_t cBuff[2]={1,1};
	uint8_t buff[2]={0,0};
	uint16_t readBuff;
//	i2c_INA238.writeDirect(i2cPort, MS5837_ADDR, MS5837_RESET, 1); //RESET
	sleep_ms(100);
	printf("OK!\n");
//	i2c_INA238.writeDirect(i2cPort, MS5837_ADDR, 0b00011110, 1); //PROM-READ-START
//	for(uint8_t i=0; i<7; i++){
//		i2c_INA238.read(i2cPort, MS5837_ADDR, MS5837_PROM_READ+i*2, cBuff, 2);
//		c[i] = ((cBuff[0]<<8) | (cBuff[1]));
//		printf("0x%x, 0x%x, %d\n", MS5837_PROM_READ+i*2, c[i], c[i]);
//	}
	buff[0] = 0x00;
	buff[1] = 0x00;
	i2c_INA228.writeMultiByte(i2cPort, INA228_ADDR, INA228_CONFIG, buff, 2);
	buff[0] = 0b11110000;
	buff[1] = 0b00000111;
	i2c_INA228.writeMultiByte(i2cPort, INA228_ADDR, INA228_ADC_CONFIG, buff, 2);
	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_CONFIG, cBuff, 2);
	readBuff = ((cBuff[0] << 8) | (cBuff[1]));
	printf("0x%x, 0x%x, INA228 CONFIG= 0x%x\n", cBuff[0], cBuff[1], readBuff);
	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_ADC_CONFIG, cBuff, 2);
	readBuff = ((cBuff[0] << 8) | (cBuff[1]));
	printf("0x%x, 0x%x, INA228 ADC_CONFIG= 0x%x\n", cBuff[0], cBuff[1], readBuff);
	sleep_ms(100);
	
	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_MAN_ID, cBuff, 2);
	readBuff = ((cBuff[0] << 8) | (cBuff[1]));
	printf("0x%x, 0x%x, INA228 MAN ID= 0x%x\n", cBuff[0], cBuff[1], readBuff);
	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_DEV_ID, cBuff, 2);
	readBuff = ((cBuff[0] << 8) | (cBuff[1]));
	printf("0x%x, 0x%x, INA228 DEV ID= 0x%x\n", cBuff[0], cBuff[1], readBuff);
//	i2c_INA238.writeDirect(i2cPort, MS5837_ADDR, MS5837_CONVERT_D2_8192, 1); 
	return 0;
}

int INA228::readCurVolPow(i2c_inst_t *i2cPort, double *current, double *voltage, double *power){
	uint8_t buff[3]={0x00,0x00,0x00};
	uint8_t buff2[2]={0x00,0x00};
	int32_t buffSigned=0;

	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_VOL, buff, 3);
	*voltage = (((buff[0]<<16) | (buff[1]<<8) | buff[2])>>4) * LSBVOL;
//	printf("0b%b, 0b%b, 0b%b \n", buff[0], buff[1], buff[2]);

	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_CUR, buff, 3);
//	*current = (double)(((buff[0]<<16) | (buff[1]<<8) | buff[2])>>4) * LSBCUR_HIGH;
	buffSigned = buff[0]<<16 | buff[1]<<8 | buff[2];
	buffSigned = buffSigned >> 4;
	*current =  (double)buffSigned * LSBCUR_HIGH;
//	printf("0b%b, 0b%b, 0b%b \n", buff[0], buff[1], buff[2]);

	i2c_INA228.read(i2cPort, INA228_ADDR, INA228_DIAG_ALRT, buff2, 2);
	return 0;
}

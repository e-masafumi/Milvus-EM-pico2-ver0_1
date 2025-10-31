#include <iostream>
#include <bits/stdc++.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/pwm.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "uart_rx.pio.h"
#include "pico/binary_info.h"
#include "extern.h"
#include "struct.h"
#include "math.h"

#include "func-pwm.h"
#include "func-i2c.h"
#include "func-uart.h"
#include "func-usbuart.h"
#include "func-MS5837-02BA.h"
#include "func-BNO055.h"
#include "func-INA228.h"
#include "ff.h"
#include "sd_card.h"
#include "pio_uart.hpp"

#define CORE1_HELLO   (99999)
#define LOG_WRITE_COM   (12345)
#define EXIT_MSG    (99998)
#define MAX_LOOP    (10)
#define DO_COMMAND  (1)
#define EXIT_LOOP   (2)

#define DEFAULT_TARGET_DEPTH_M (0.00)
#define EXE_FREC (10.00)	//Hz

#define LEFT_VERTICAL 1
#define LEFT_HORIZONTAL 0
#define RIGHT_VERTICAL 3
#define RIGHT_HORIZONTAL 2

#define CELL_NUMBER 3
#define VOLTAGE_LOWER_LIMIT_PER_CELL 3.5
#define VOLTAGE_UPPER_LIMIT_PER_CELL 4.5


#define SERIAL_BAUD_PIO 115200
#define PIO_RX_PIN 3

using namespace std;

const uint I2C0_SDA_PIN = 8;
const uint I2C0_SCL_PIN = 9;
const uint I2C1_SDA_PIN = 14;
const uint I2C1_SCL_PIN = 15;

const uint LED_PIN = 25;
volatile bool exeFlag = false;
struct repeating_timer st_timer;
pico_pwm pwm;
pico_i2c i2c;
pico_uart uart;
pico_usbuart usbuart;
MS5837_02BA MS5837;
BNO055 BNO055;
INA228 INA228;
static semaphore_t sem;
bool nmeaDecodedFlag = false;
bool logWriteFlag = false;
bool onBoardLED = true;
bool inputAccept = true;
bool ManualModeFlag = false;	//-1111,ON, -8888, OFF
double currentDepth = 0;

double kp=0.0001;
const double ki=0.0;
const double kd=0.0;
double depthError=0.0;
int startCnt = 0;

FRESULT fr;
FATFS fs;
FIL fil;
int ret;
char buf[100];
char filename[] = "log.txt";
double thrusterOutput[4] = {0, 0, 0, 0};
double targetDepth = DEFAULT_TARGET_DEPTH_M;
double uartReceiveData;

struct str_sensorsData logData;
struct str_NMEA decodedNMEA;
struct str_ULSA decodedULSA;


constexpr int LINE_MAX = 256;
constexpr int QSIZE    = 16;
static char q[QSIZE][LINE_MAX];
static volatile uint8_t q_w=0, q_r=0;

bool reserved_addr(uint8_t addr){
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}
int64_t alarm_callback(alarm_id_t id, void *user_data) {
	exeFlag = true;
	return 0;
}

bool repeating_timer_callback(struct repeating_timer *t) {
	exeFlag = true;
	return true;
}

bool fifo_push(const char* s){
    uint8_t n=(q_w+1)&(QSIZE-1);
    if(n==q_r) return false;           // full
    strncpy(q[q_w], s, LINE_MAX-1);
    q[q_w][LINE_MAX-1]='\0';
    q_w=n; return true;
}

void parseCsvULSA(const char* line) {
  // もし '#' で始まっていたらスキップするだけ
	if (line[0] == '#') line++;

  // 一時バッファを作る（strtokは元の文字列を書き換えるのでコピーする）
  char buf[256];
  strncpy(buf, line, sizeof(buf));
  buf[sizeof(buf) - 1] = '\0';

  char* token = strtok(buf, ",");
  int   index = 0;

  while (token != nullptr) {
		if (index == 0) {
			decodedULSA.id = atoi(token);
      printf("id = %d\n", decodedULSA.id);
    } 
		else if (index == 1) {
			decodedULSA.active = atoi(token);
      printf("active = %d\n", decodedULSA.active);
    } 
		else if (index == 2) {
      decodedULSA.direction = atoi(token);
      printf("direction = %d\n", decodedULSA.direction);
    } 
		else if (index == 3) {
      decodedULSA.absoluteSpeed = atof(token);
      printf("absoluteSpeed = %f\n", decodedULSA.absoluteSpeed);
		}
		else if (index == 4) {
      decodedULSA.noseSpeed = atof(token);
      printf("noseSpeed = %f\n", decodedULSA.noseSpeed);
		}
		else if (index == 5) {
      decodedULSA.soundSpeed = atof(token);
      printf("soundSpeed = %f\n", decodedULSA.soundSpeed);
		}
		else if (index == 6) {
      decodedULSA.virtualTemp = atof(token);
      printf("virtualTemp = %f\n", decodedULSA.virtualTemp);
		}

    index++;
    token = strtok(nullptr, ",");  // 次のカンマ区切りへ
  }
}



void core1_main(void){


  if (!sd_init_driver()) {
		printf("ERROR: Could not initialize SD card\r\n");
		while (true);
	}

  fr = f_mount(&fs, "0:", 1);
  if (fr != FR_OK) {
		printf("ERROR: Could not mount filesystem (%d)\r\n", fr);
		while (true);
  }

	fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK) {
		printf("ERROR: Could not open file (%d)\r\n", fr);
		while (true);
  }

  ret = f_printf(&fil, "Test\r\n");
	if (ret < 0) {
		printf("ERROR: Could not write to file (%d)\r\n", ret);
		f_close(&fil);
		while (true);
	}
    // Close file
  fr = f_close(&fil);
  if (fr != FR_OK) {
		printf("ERROR: Could not close file (%d)\r\n", fr);
    while (true);
  }

  // Open file for reading
  fr = f_open(&fil, filename, FA_READ);
  if (fr != FR_OK) {
		printf("ERROR: Could not open file (%d)\r\n", fr);
    while (true);
  }

    // Print every line in file over serial
	printf("Reading from file '%s':\r\n", filename);
  printf("---\r\n");
  while (f_gets(buf, sizeof(buf), &fil)) {
		printf(buf);
  }
  printf("\r\n---\r\n");

  // Close file
  fr = f_close(&fil);
  if (fr != FR_OK) {
		printf("ERROR: Could not close file (%d)\r\n", fr);
		while (true);
  }

    // Unmount drive
//    f_unmount("0:");
    // Open file for writing ()
	fr = f_open(&fil, filename, FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK) {
		printf("ERROR: Could not open file (%d)\r\n", fr);
    while (true);
  }


	ret = f_printf(&fil, "Internal Time, Voltage, Current, Outer Temperature,Outer Pressure,Accel X,Accel Y,Accel Z,Mag X,Mag Y,Mag Z,GPS Time,Time_seconds,NorS,Latitude,EorW,Longitude,Qual,Sats,Hdop,Altitude ASL,Altitude Geoid, ULSA id, ULSA active, ULSA direction, ULSA absoluteSpeed, ULSA noseSpeed, ULSA soundSpeed, ULSA virtualTemp\r\n");

	// Close file
	fr = f_close(&fil);
	if (fr != FR_OK) {
		printf("ERROR: Could not close file (%d)\r\n", fr);
		while (true);
	}



	multicore_fifo_push_blocking(CORE1_HELLO);

	printf("SPI Baudrate = %d\n\n", spi_get_baudrate(spi0));
	
	while(1){
		
		static uint32_t core1preTime;
		static uint32_t core1postTime;

		uint32_t logWriteCom = multicore_fifo_pop_blocking();
		if(!(logWriteCom == LOG_WRITE_COM)){
			continue;
		}
		else{
			logWriteFlag = true;
		}
		if(logWriteFlag){
			core1preTime = time_us_32();
			// Open file for writing ()
			fr = f_open(&fil, filename, FA_WRITE | FA_OPEN_ALWAYS);
			if (fr != FR_OK) {
				printf("ERROR: Could not open file (%d)\r\n", fr);
				while (true);
			}
				//Move to end
			ret = f_lseek(&fil, f_size(&fil));
			ret = f_printf(&fil, "%lu,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%c,%lf,%c,%lf,%d,%d,%lf,%lf,%lf, %d,%d,%d,%lf,%lf,%lf,%lf\r\n", 
				logData.timeBuff_64, 
				logData.mainVol, 
				logData.mainCur,
				logData.outTemp, 
				logData.outPress,
				logData.xAccel,
				logData.yAccel,
				logData.zAccel,
				logData.xMag,
				logData.yMag,
				logData.zMag, 
				decodedNMEA.time, 
				decodedNMEA.seconds, 
				decodedNMEA.nOrS, 
				decodedNMEA.latitude, 
				decodedNMEA.eOrW, 
				decodedNMEA.longitude, 
				decodedNMEA.qual, 
				decodedNMEA.sats, 
				decodedNMEA.hdop, 
				decodedNMEA.altitudeASL, 
				decodedNMEA.altitudeGeoid,
				decodedULSA.id,
				decodedULSA.active,
				decodedULSA.direction,
				decodedULSA.absoluteSpeed,
				decodedULSA.noseSpeed,
				decodedULSA.soundSpeed,
				decodedULSA.virtualTemp
			);
			if (ret < 0) {
				printf("ERROR: Could not write to file (%d)\r\n", ret);
	     f_close(&fil);
	     while (true);
			}
			// Close file
			fr = f_close(&fil);
			if (fr != FR_OK) {
				printf("ERROR: Could not close file (%d)\r\n", fr);
				while (true);
			}
			logWriteFlag = false;
			core1postTime = time_us_32();
//			printf("Pre(core1)[ms]: %lf\n", (double)core1preTime/1000);
//			printf("Post(core1)[ms]: %lf\n", (double)core1postTime/1000);
//			printf("dt(core1)[ms]: %lf\n\n", ((double)core1postTime-(double)core1preTime)/1000);
			
		}
	
	}
}

int main(){

	uint8_t data=0;
	uint64_t timeBuff_64=0;

	double pSurface=0.0;
	double tempSurface=0.0;
	double turgetDepth=0.0;
	int i=0;

	PIO pio;
	uint sm;
	uint offset;

	sem_init(&sem, 1, 1);
	printf("start");
	stdio_init_all();
	pwm.setup();

	i2c.setup(i2c0, 400*1000);
	i2c.setup(i2c1, 400*1000);
	gpio_set_function(I2C0_SDA_PIN, GPIO_FUNC_I2C);	//raspi mother ver1.0
	gpio_set_function(I2C0_SCL_PIN, GPIO_FUNC_I2C);	//raspi mother ver1.0
	gpio_set_function(I2C1_SDA_PIN, GPIO_FUNC_I2C);	//raspi mother ver1.0
	gpio_set_function(I2C1_SCL_PIN, GPIO_FUNC_I2C);	//raspi mother ver1.0
	gpio_pull_up(I2C0_SDA_PIN);
	gpio_pull_up(I2C0_SCL_PIN);
	gpio_pull_up(I2C1_SDA_PIN);
	gpio_pull_up(I2C1_SCL_PIN);
	bi_decl(bi_2pins_with_func(I2C0_SDA_PIN, I2C0_SCL_PIN, GPIO_FUNC_I2C));
	bi_decl(bi_2pins_with_func(I2C1_SDA_PIN, I2C1_SCL_PIN, GPIO_FUNC_I2C));

	sleep_ms(500);
  printf("Done.\n");
	
	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	printf("HelloLED!\n");
	gpio_put(LED_PIN, onBoardLED);
	sleep_ms(500);


  bool success = pio_claim_free_sm_and_add_program_for_gpio_range(&uart_rx_program, &pio, &sm, &offset, PIO_RX_PIN, 1, true);
  hard_assert(success);
	uart_rx_program_init(pio, sm, offset, PIO_RX_PIN, SERIAL_BAUD_PIO);


	BNO055.setup(i2c1);
	printf("HelloBNO055!\n");
	sleep_ms(100);
//	MS5837.setup(i2c1);
//	printf("HelloMS5837!\n");
//	sleep_ms(100);
	printf("OK!\n");
	INA228.setup(i2c0);
	printf("HelloINA238!\n");
	sleep_ms(100);
	

	printf("TestDone\n");
	sleep_ms(100);

	printf("uint32_t: %lu\n", sizeof(logData.timeBuff_64));
	printf("double: %lu\n", sizeof(logData.outTemp));
	printf("logData: %lu\n", sizeof(logData));
	
	double pSurfaceSum = 0;
	double tempSurfaceSum = 0;
//	int MS5837initCnt = 0;
//	for(int i=0;i<100;i++){
//		MS5837.readTempPress(i2c1, &tempSurface, &pSurface);
//		pSurfaceSum += pSurface;
//		tempSurfaceSum += tempSurface;
//		MS5837initCnt++;
//	}
//	pSurface = pSurfaceSum/MS5837initCnt;
//	tempSurface = tempSurfaceSum/MS5837initCnt;
	
	printf("SurfaceTemp = %f [C]\n", tempSurface);
	printf("SurfacePress = %f [mbar]\n", pSurface);


//GPS
	int actualBaudrate[2]={12000,12000};
	int messageBlockCnt = 0;
	char nmeaReadBuff=0;
	double nmeaBuff=0.0;
	actualBaudrate[0] = uart.setup(uart0, 9600, 8, 1);
	sleep_ms(3000);
	printf("UART actual baudrate,core1 0: %d, 1: %d\n", actualBaudrate[0], actualBaudrate[1]);


	add_repeating_timer_us(-1000/EXE_FREC*1000, repeating_timer_callback, NULL, &st_timer);


	uint32_t f_pll_sys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_PLL_SYS_CLKSRC_PRIMARY);
	printf("\n-----\n");
  printf("pll_sys = %dkHz\n", f_pll_sys);
	printf("\n-----\n");
	
	multicore_launch_core1(core1_main);
	uint32_t core1HelloMsg = multicore_fifo_pop_blocking();
	while(!(core1HelloMsg == CORE1_HELLO)){
	}
	printf("hello: %d\n", core1HelloMsg);
	

	uint32_t volErrorCnt;

/*	while(1){		//Battery check loop
		INA228.readCurVolPow(i2c0, &logData.mainCur, &logData.mainVol, &logData.mainPow);
		if(logData.mainVol*1e-6 > CELL_NUMBER * VOLTAGE_UPPER_LIMIT_PER_CELL){
			printf("Voltage NG (TOO HIGH): %lf [V],  %d\n", logData.mainVol*1e-6, volErrorCnt);
			volErrorCnt++;
			sleep_ms(3000);
		}
		else if(logData.mainVol*1e-6 < CELL_NUMBER * VOLTAGE_LOWER_LIMIT_PER_CELL){
			printf("Voltage NG (TOO LOW): %lf [V],  %d\n", logData.mainVol*1e-6, volErrorCnt);
			volErrorCnt++;
			sleep_ms(3000);
		}
		else{
			printf("Voltage OK: %lf [V]\n", logData.mainVol*1e-6);
			break;
		}
	}*/
/*
	for(int i=0; i<4; i++){
		for(int j=0; j<20; j++){
			pwm.duty(i, pwm.dutyFitPct(j, 0.55, 0.95));
			printf("Motor: %d,  ", i);
			printf("Duty[%%]: %d\n", j);
			sleep_ms(50);
		}
		for(int j=20; j>-1; j--){
			pwm.duty(i, pwm.dutyFitPct(j, 0.55, 0.95));
			printf("Motor: %d,  ", i);
			printf("Duty[%%]: %d\n", j);
			sleep_ms(50);
		}
		sleep_ms(500);
	}

	
	printf("Thruster Check Done\n");
*/

/*	printf("Waiting");
	while(1){
		if(inputAccept){
			uartReceiveData = usbuart.receive_usbuart_data();
		}
		if(uartReceiveData == 12345){
			printf("start with %lf\n", uartReceiveData);
			startCnt++;
			printf("If you want to start: type 0\n");
			while(1){
				uartReceiveData = usbuart.receive_usbuart_data();
				if(uartReceiveData == 0){
					startCnt++;
					break;
				}
				sleep_ms(100);
			}
		}
		else if(startCnt >= 2){
			break;
		}
		else{
			printf(".");
			startCnt = 0;
			sleep_ms(300);
		}
	}
*/
	double targetPress = (DEFAULT_TARGET_DEPTH_M*1013*10)+pSurface;
	static uint32_t preTime = time_us_32();
	static uint32_t nowTime = time_us_32();

	PioUartRx rx(pio0, /*sm*/0, /*RX GPIO*/3, /*baud*/115200);
	char line[256];

	while(1){	//main loop
		if (rx.dataArrived()) {
			rx.clearDataArrived();
			while (rx.popLine(line, sizeof(line))) {
        fifo_push(line);  // ここでは単にコピーして保存するだけ
				parseCsvULSA(line);
				puts(line);
			}
		}	   
		
		if(messageFinishFlag == true){
			messageFinishFlag = false;
			onBoardLED = !onBoardLED;
			gpio_put(LED_PIN, onBoardLED);
			if((readNMEA[0][2] == 'G') && (readNMEA[0][3] == 'G') && (readNMEA[0][4] == 'A')){
				decodedNMEA.time = atof(readNMEA[1]);
				
				decodedNMEA.latitude = atof(readNMEA[2]);
				nmeaBuff = decodedNMEA.latitude/100;
				decodedNMEA.latitude = (int)nmeaBuff + (nmeaBuff-(int)nmeaBuff)/60;
				decodedNMEA.nOrS = readNMEA[3][0];
				
				decodedNMEA.longitude = atof(readNMEA[4]);
				nmeaBuff = decodedNMEA.longitude/100;
				decodedNMEA.latitude = (int)nmeaBuff + (nmeaBuff-(int)nmeaBuff)/60;
				decodedNMEA.eOrW = readNMEA[5][0];
				decodedNMEA.qual = atoi(readNMEA[6]);
				decodedNMEA.sats = atoi(readNMEA[7]);
				decodedNMEA.hdop = atof(readNMEA[8]);
				decodedNMEA.altitudeASL = atof(readNMEA[9]);
				decodedNMEA.altitudeGeoid = atof(readNMEA[11]);
			}
		}

		if(decodedNMEA.qual <= 1){
		}

		if(exeFlag == false){
			continue;
		}
		
		nowTime = time_us_32();
		preTime = time_us_32();

		logData.timeBuff_64 = time_us_64();
		BNO055.readAccel(i2c1, &logData.xAccel, &logData.yAccel, &logData.zAccel);
		BNO055.readMag(i2c1, &logData.xMag, &logData.yMag, &logData.zMag);
		INA228.readCurVolPow(i2c0, &logData.mainCur, &logData.mainVol, &logData.mainPow);

		multicore_fifo_push_blocking(LOG_WRITE_COM);
		exeFlag = false;
	}
}

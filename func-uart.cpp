#include <stdio.h>
#include <string>
#include <vector>
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/irq.h"
#include "func-uart.h"

const uint UART0_TX_PIN = 0;
const uint UART0_RX_PIN = 1;
const uint UART1_TX_PIN = 4;
const uint UART1_RX_PIN = 5;
const std::string targetMessage = "GPGGA";

static int chars_rxed = 0;
bool messageStartFlag = false;
bool messageTypeDetectFlag = false;
bool nmeaUpdateFlag = false;
bool uart0DataInFlag = false;
//std::vector<std::string> splitNMEA(32);
char uart0ReadBuff;
bool messageFinishFlag = false;
bool nmeaLaunchFlag = false;

char nmeaBlockCnt=0;
char nmeaCharCnt=0;
char readNMEA[30][15];

void on_uart0_rx(){
	while (uart_is_readable(uart0)) {
		uart0ReadBuff = uart_getc(uart0);
		if(nmeaLaunchFlag){
			switch(uart0ReadBuff){
				case 0x24:		//$	
					messageFinishFlag = false;
					nmeaBlockCnt = 0;
					nmeaCharCnt = 0;
					break;
				case 0x2c:			//,
					nmeaBlockCnt++;
					nmeaCharCnt = 0;
					break;
				case 0x0A:		//LF
					messageFinishFlag = true;
					break;
				default:
					readNMEA[nmeaBlockCnt][nmeaCharCnt] = uart0ReadBuff;
					nmeaCharCnt++;
			}
		}
		else{
			if(uart0ReadBuff == 0x24){
				nmeaLaunchFlag = true;
			}
		}
		
//		sem_acquire_blocking(&sem);
/*			uart0DataInFlag = true;
//		sem_release(&sem);
//		printf("hogehoge");
		if(uart0ReadBuff == 0x0A){
			printf("<LF>");
		}
		else if(uart0ReadBuff == 0x0D){
			printf("<CR>");
		}
		else{
			printf("%c", uart0ReadBuff);
		}*/
/*
		if(uart0Buff == 0x24 && !messageStartFlag){							//$
			messageFinishFlag = false;
			messageStartFlag = true;
			printf("MESSAGE START");
//			printf("%c\n", uart0Buff);
			messageBlockCnt = 0;
//			continue;
		}
*/
/*
		if(uart0Buff == 0x0a){							//<LF>
			messageFinishFlag = true;
			printf("MESSAGE FIN. %s\n", messageFinishFlag ? "true" : "false");
		}
*/
/*		if(messageStartFlag){
			if(uart0Buff != 0x2c){			//,
				splitNMEA[messageBlockCnt] += uart0Buff;
			}
			else if(uart0Buff == 0x2c){
				messageBlockCnt++;
			}
			else{
				printf("ERROR!!!!!!!");
				sleep_ms(100000);
			}
			if(messageBlockCnt >= 1){
				if(splitNMEA[1].compare(0, 5, targetMessage) < 5){
					continue;
				}
			}
		}
*/
		

        // Can we send it back?
//        if (uart_is_writable(uart0)) {
            // Change it slightly first!
//            ch++;
 //           uart_putc(uart0, ch);
    }
 //       chars_rxed++;
}
/*void on_uart1_rx() {
    while (uart_is_readable(uart1)) {
        uint8_t ch = uart_getc(uart1);
        // Can we send it back?
        if (uart_is_writable(uart1)) {
            // Change it slightly first!
            ch++;
            uart_putc(uart1, ch);
        }
        chars_rxed++;
    }
}
*/


int pico_uart::setup(uart_inst_t *uartPort, uint uartBaudrate, uint dataBit, uint stopBit){
	int uartTxPin = 1000;
	int uartRxPin = 1000;

	if(uartPort == uart0){
		uartTxPin = UART0_TX_PIN;
		uartRxPin = UART0_RX_PIN;
	}
	else if(uartPort == uart1){
		uartTxPin = UART1_TX_PIN;
		uartRxPin = UART1_RX_PIN;
	}
	else{
		printf("CHECK THE UART ID");
		return 1;
	}

	uart_init(uartPort, uartBaudrate);
	gpio_set_function(uartTxPin, GPIO_FUNC_UART);	//raspi mother ver1.0
	gpio_set_function(uartRxPin, GPIO_FUNC_UART);	//raspi mother ver1.0

	int actual = uart_set_baudrate(uartPort, uartBaudrate);
	uart_set_hw_flow(uartPort, false, false);
	uart_set_format(uartPort, dataBit, stopBit, UART_PARITY_NONE);

	uart_set_fifo_enabled(uartPort, false);

	int UART_IRQ = uartPort == uart0 ? UART0_IRQ : UART1_IRQ;
	if(uartPort == uart0){
		irq_set_exclusive_handler(UART_IRQ, on_uart0_rx);
	}
	else if(uartPort == uart1){
//		irq_set_exclusive_handler(UART_IRQ, on_uart1_rx);
	}
	else{
		printf("CHECK THE UART ID");
		return 1;
	}
	irq_set_enabled(UART_IRQ, true);
	uart_set_irq_enables(uartPort, true, false);


	for(int i=0; i<30; i++){
		for(int j=0; j<15; j++){
	//		readNMEA[i][j] = 0x23;
//			readNMEA[i][j] = 0;
		}
	}
	char ultimateGPSset[] = "PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0";
	//PMTK314, GLL, RMC, VTG, GGA, GSA, GSV, ~reserved~, NMEA PMTKCHN

	char NMEAchecksum = ultimateGPSset[0];
	for(int i=1; i<(sizeof(ultimateGPSset)/sizeof(ultimateGPSset[0]))-1; i++){
		NMEAchecksum ^= ultimateGPSset[i];
	}
	char NMEAchecksumAscii[2] = {0, 0};
	char hexTable[] = "0123456789ABCDEF";
	NMEAchecksumAscii[0] = hexTable[(char)NMEAchecksum/16];
	NMEAchecksumAscii[1] = hexTable[(char)NMEAchecksum%16];
	
	uart_putc(uartPort, '$');
	printf("$");
	for(int i=0; i<(sizeof(ultimateGPSset)/sizeof(ultimateGPSset[0]))-1; i++){
		uart_putc_raw(uartPort, ultimateGPSset[i]);
		printf("%c", ultimateGPSset[i]);
	}
	uart_putc_raw(uartPort, '*');
	printf("*");
	uart_putc_raw(uartPort, NMEAchecksumAscii[0]);
	printf("%c", NMEAchecksumAscii[0]);
	uart_putc_raw(uartPort, NMEAchecksumAscii[1]);
	printf("%c", NMEAchecksumAscii[1]);
	uart_puts(uartPort, "\r\n");
	printf("\r\n");
	sleep_ms(500);	


	char ultimateGPSset1[] = "PMTK220,200";
	//PMTK220, milliseconds
	NMEAchecksum = ultimateGPSset1[0];
	for(int i=1; i<(sizeof(ultimateGPSset1)/sizeof(ultimateGPSset1[0]))-1; i++){
		NMEAchecksum ^= ultimateGPSset1[i];
	}
	NMEAchecksumAscii[0] = hexTable[(char)NMEAchecksum/16];
	NMEAchecksumAscii[1] = hexTable[(char)NMEAchecksum%16];
	
	uart_putc(uartPort, '$');
	printf("$");
	for(int i=0; i<(sizeof(ultimateGPSset1)/sizeof(ultimateGPSset1[0]))-1; i++){
		uart_putc_raw(uartPort, ultimateGPSset1[i]);
		printf("%c", ultimateGPSset1[i]);
	}
	uart_putc_raw(uartPort, '*');
	printf("*");
	uart_putc_raw(uartPort, NMEAchecksumAscii[0]);
	printf("%c", NMEAchecksumAscii[0]);
	uart_putc_raw(uartPort, NMEAchecksumAscii[1]);
	printf("%c", NMEAchecksumAscii[1]);
	uart_puts(uartPort, "\r\n");
	printf("\r\n");
	sleep_ms(500);	


	char ultimateGPSset2[] = "PMTK251,38400";
	//PMTK251, baudrate
	NMEAchecksum = ultimateGPSset2[0];
	for(int i=1; i<(sizeof(ultimateGPSset2)/sizeof(ultimateGPSset2[0]))-1; i++){
		NMEAchecksum ^= ultimateGPSset2[i];
	}
	NMEAchecksumAscii[0] = hexTable[(char)NMEAchecksum/16];
	NMEAchecksumAscii[1] = hexTable[(char)NMEAchecksum%16];
	
	uart_putc(uartPort, '$');
	printf("$");
	for(int i=0; i<(sizeof(ultimateGPSset2)/sizeof(ultimateGPSset2[0]))-1; i++){
		uart_putc_raw(uartPort, ultimateGPSset2[i]);
		printf("%c", ultimateGPSset2[i]);
	}
	uart_putc_raw(uartPort, '*');
	printf("*");
	uart_putc_raw(uartPort, NMEAchecksumAscii[0]);
	printf("%c", NMEAchecksumAscii[0]);
	uart_putc_raw(uartPort, NMEAchecksumAscii[1]);
	printf("%c", NMEAchecksumAscii[1]);
	uart_puts(uartPort, "\r\n");
	printf("\r\n");
	sleep_ms(500);	

	actual = uart_set_baudrate(uartPort, 38400);

	return actual;
}

#include <stdio.h>
#include <stdlib.h>
//#include <ctype.h>
#include "pico/stdlib.h"
#include "class/cdc/cdc_device.h"
#include "func-usbuart.h"

#define BUF_SIZE 64
char rxInput;
char rxBuff[BUF_SIZE];
unsigned short rxIndex = 0;
double receivedValue = 0.0;

double pico_usbuart::receive_usbuart_data(){
	while(tud_cdc_available() > 0){
		rxInput = tud_cdc_read_char();
		if(rxInput == '\n'){
			rxBuff[rxIndex] = '\0';
			receivedValue = strtod(rxBuff, 0);	
		}
		else{
			rxBuff[rxIndex] = rxInput;
			rxIndex++;
		}
	}
	rxIndex = 0;
	return receivedValue;
}

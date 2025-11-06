#pragma once	//二重インクルード防止
#include "pico/stdlib.h"	//picoの標準関数
#include "hardware/uart.h"	//ハードウェアUART使用
#include "hardware/dma.h"		//ハードウェアDMA使用
#include "hardware/pio.h"

struct UartRxDma {		//Uart受信用の構造体
    uart_inst_t* uart = nullptr;	
    int ch = -1;                  // DMA channel
    uint8_t* ring = nullptr;      // ring buffer
    size_t ring_size = 0;         // power of 2
    uint32_t ring_mask = 0;       // ring_size - 1
    volatile uint32_t last_w = 0; // last write index (mod ring_size)
		volatile void* src_reg = nullptr;
		uint dreq = 0;
};


bool dma_rx_ring_init(UartRxDma& u, volatile void* src_reg,  uint dreq, uint8_t* ring_mem, size_t ring_sz);
bool uart_rx_dma_init_hw(UartRxDma& u, uart_inst_t* UARTx, uint8_t* ring_mem, size_t ring_sz);
bool uart_rx_dma_init_pio(UartRxDma& u, PIO pio, uint sm, uint8_t* ring_mem, size_t ring_sz);
bool uart_rx_dma_init(UartRxDma& u, uart_inst_t* uart, uint8_t* ring_mem, size_t ring_size, uint baud);
size_t uart_rx_dma_read(UartRxDma& u, uint8_t* out, size_t maxlen); // pull new bytes


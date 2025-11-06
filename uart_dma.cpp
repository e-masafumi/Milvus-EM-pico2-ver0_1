#include "uart_dma.hpp"		//構造体の定義と関数プロトタイプ宣言
#include "hardware/regs/dreq.h"		//DMA request番号の一覧
#include <cstring>	//memcpy使いたい
#include <algorithm>	//min使いたい


// UART受信をDMAでリングバッファに流し続けるように設定する初期化関数(hard UARTとPIO両対応)
bool dma_rx_ring_init(UartRxDma& u, volatile void* src_reg,  uint dreq, uint8_t* ring_mem, size_t ring_sz){	//初期化用関数
	u.src_reg = src_reg;	//受信レジスタ設定
	u.dreq = dreq;				//割り込みレジスタ設定
	u.ring = ring_mem;	//リングバッファ先頭アドレス
	u.ring_size = ring_sz;	//リングバッファサイズ
	u.ring_mask = (uint32_t)(ring_sz - 1);	//リングバッファ先頭に戻るためのマスク
  u.last_w = 0;	//最終書き込み位置を初期化


  // DMAチャネル確保
  u.ch = dma_claim_unused_channel(true);	//DMAで使用されていないチャンネルを要求
  dma_channel_config c = dma_channel_get_default_config(u.ch);	//上記でgetしたチャンネルを使用するよう設定
  channel_config_set_transfer_data_size(&c, DMA_SIZE_8);	//一度に扱うデータサイズを設定
  channel_config_set_read_increment(&c, false);  // src = UART DR 固定
  channel_config_set_write_increment(&c, true);  // dst = メモリ前進
  channel_config_set_dreq(&c, dreq);	//使用するUART番号に応じてRequestの番号を選択

  // 書き込みアドレスをリング化（ring_sz は 2^n）
  uint ring_bits = 0;		//リングバッファを移動するカウンタを初期化
	while ((1u << ring_bits) < ring_sz){	//リングサイズring_sz を満たす 2^ring_bits を計算
		++ring_bits;
	}
  channel_config_set_ring(&c, true, ring_bits);	//書き込みアドレスを 2^ring_bits バイト境界で循環させる

  dma_channel_configure(	//DMAチャンネルの設定
		u.ch, &c,							//DMAの使用チャネルを設定
    u.ring,								//転送先 (リング先頭)
    src_reg,							//転送元(UART DR or PIO RXF)
    0xFFFFFFFF,           //大きなカウントで回しっぱなし
    true                  //設定完了後すぐ開始
   );
    return true;
}


bool uart_rx_dma_init_hw(UartRxDma& u, uart_inst_t* UARTx, uint8_t* ring_mem, size_t ring_sz) {
    volatile void* src = (volatile void*)&uart_get_hw(UARTx)->dr;	//読み出し元をHardware UARTのDataResisterに設定
    uint dreq = (UARTx == uart0) ? DREQ_UART0_RX : DREQ_UART1_RX;	//UART受信用のトリガを設定によって振り分け
    return dma_rx_ring_init(u, src, dreq, ring_mem, ring_sz);	//初期化を呼ぶ
}

bool uart_rx_dma_init_pio(UartRxDma& u, PIO pio, uint sm, uint8_t* ring_mem, size_t ring_sz) {
    volatile void* src = (volatile void*)&pio->rxf[sm];	//読み出し元をPIOのStateMachineのRX FIFOに設定
    uint dreq = (pio == pio0) ? (DREQ_PIO0_RX0 + sm) : (DREQ_PIO1_RX0 + sm);	//PIO受信用DREQを選択
    return dma_rx_ring_init(u, src, dreq, ring_mem, ring_sz);
}


// UART受信をDMAでリングバッファに流し続けるように設定する初期化関数(hard UARTのみ対応)
bool uart_rx_dma_init(UartRxDma& u, uart_inst_t* UARTx, uint8_t* ring_mem, size_t ring_sz, uint baud){	//初期化用関数
    u.uart = UARTx;		//Hardware UARTの番号定義
		u.ring = ring_mem;	//リングバッファ先頭アドレス
		u.ring_size = ring_sz;	//リングバッファサイズ
		u.ring_mask = (uint32_t)(ring_sz - 1);	//リングバッファ先頭に戻るためのマスク
    u.last_w = 0;	//最終書き込み位置を初期化

    // UART初期化（既存設定があるならそれを流用）
//    uart_init(UARTx, baud);		//UARTのbaudrate設定
//    uart_set_format(UARTx, 8, 1, UART_PARITY_NONE);	//UARTの通信フォーマット設定
    uart_set_fifo_enabled(UARTx, true);	//FIFOバッファを有効化

    // DMAチャネル確保
    u.ch = dma_claim_unused_channel(true);	//DMAで使用されていないチャンネルを要求
    dma_channel_config c = dma_channel_get_default_config(u.ch);	//上記でgetしたチャンネルを使用するよう設定
    channel_config_set_transfer_data_size(&c, DMA_SIZE_8);	//一度に扱うデータサイズを設定
    channel_config_set_read_increment(&c, false);  // src = UART DR 固定
    channel_config_set_write_increment(&c, true);  // dst = メモリ前進
    channel_config_set_dreq(&c, (UARTx == uart0) ? DREQ_UART0_RX : DREQ_UART1_RX);	//使用するUART番号に応じてRequestの番号を選択

    // 書き込みアドレスをリング化（ring_sz は 2^n）
    uint ring_bits = 0;		//リングバッファを移動するカウンタを初期化
		while ((1u << ring_bits) < ring_sz){	//リングサイズring_sz を満たす 2^ring_bits を計算
			++ring_bits;
		}
    channel_config_set_ring(&c, true, ring_bits);	//書き込みアドレスを 2^ring_bits バイト境界で循環させる

    dma_channel_configure(	//DMAチャンネルの設定
        u.ch, &c,													//DMAの使用チャネルを設定
        u.ring,                           //転送先 (リング先頭)
        &uart_get_hw(UARTx)->dr,          //転送元(UART RX FIFO)
        0xFFFFFFFF,                       //大きなカウントで回しっぱなし
        true                              //設定完了後すぐ開始
    );
    return true;
}

// 新着分だけ吸い出す（pull型）
size_t uart_rx_dma_read(UartRxDma& u, uint8_t* out, size_t maxlen){	
    // 現在のDMA writeアドレスからリング内インデックスを得る
    uint32_t waddr = dma_hw->ch[u.ch].write_addr;
    // 書込み先頭からのオフセット（リングbitでマスク）
    uint32_t w = (waddr - (uint32_t)u.ring) & u.ring_mask;
		
    uint32_t avail = (w >= u.last_w) ? (w - u.last_w) : (u.ring_size - (u.last_w - w));	//w(DMAの現在書き込み位置)，u.last_w(ソフトウェア側で前回読み終わった位置)を使って，wがu.last_wより後ろなら進んでいたら最初に戻っていないので差分が今回読み込める(ソフトウェアがまだ読み込んでいない)量になる．wがu.last_wより前なら最初に戻っているので，リングサイズ末尾まで+先頭からwまでが今回読み込める量になる．
    if (avail == 0) return 0;	//w = u.last_wならソフトウェアが知らないデータはないので何もせず終わる．

    size_t n = (avail > maxlen) ? maxlen : avail;		//ソフトウェアが知らないデータの長さがmaxlenより長ければmaxlen，そうでなければavailの長さを読むようサイズ決定．
    size_t first = std::min(n, (size_t)(u.ring_size - u.last_w));	//上記で設定したサイズか，リング末尾までの残量の小さい方を読むよう設定
    memcpy(out, u.ring + u.last_w, first);	//outに，リングバッファ先頭アドレスからu.last_wだけ進んだ位置を先頭とするデータをfirstバイトだけコピー
    if (n > first) memcpy(out + first, u.ring, n - first);	//もしn>firstなら，上記の続きにリングバッファの最初からn-firstバイトだけコピー．最初に戻っていた場合の残り分．

    u.last_w = (u.last_w + n) & u.ring_mask;	//ソフトウェアが知っている(読み出し済み)の場所を更新．もしリングサイズを超えるなら最初に戻る．
    return n;
}


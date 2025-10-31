#pragma once
#include <cstddef>
#include <cstdint>
#include <atomic>

// Pico SDK
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include "hardware/irq.h"

// ▼ 公式サンプルの uart_rx.pio から生成されるヘッダ（リポにあるやつを使います）
#include "uart_rx.pio.h"

// 受信: PIO + DMAリング + 「データ到着」IRQ（行終端はCPU側で '\n' 検出）
// 行のフォーマット例: "#foo,bar,baz\n" （先頭 '#'、末尾 LF。CR は自動除去）
class PioUartRx {
public:
  // pio: pio0 or pio1
  // sm : 0..3
  // rx_gpio: 外部機器TXを繋ぐGPIO
  // baud: 通信速度
  // ring_size_pow2: リングサイズ 2^N（例: 11 => 2048バイト）
  PioUartRx(PIO pio, uint sm, uint rx_gpio, uint baud,
            uint8_t ring_size_pow2 = 11);
  ~PioUartRx();

  // 新しいデータが到着した合図（IRQが立てる）
  bool dataArrived() const { return arrived_.load(std::memory_order_acquire); }
  void clearDataArrived()  { arrived_.store(false, std::memory_order_release); }

  // 1行取り出し（'\n' まで）。戻り値=書き込んだ長さ(0=未完)
  // 先頭が '#' でない行は捨てます。終端の '\r' は削除。出力は '\0' 終端。
  size_t popLine(char* out, size_t cap);

  // （任意）生のリング使用率を知りたい時
  size_t ringUsedBytes() const;

  // IRQハンドラから呼ぶ “フラグだけ立てる” 短い関数
  void onRxFifoNotEmptyIrq();

private:
  // DMA の write_addr 下位ビット＝現在位置（リング）
  inline uint32_t dmaWritePosMasked_() const {
    return dma_hw->ch[dma_chan_].write_addr & ring_mask_;
  }

  // '\n' を探し、見つかったら 1 行を out にコピー（内部用）
  size_t popLineInternal_(char* out, size_t cap);

private:
  PIO   pio_;
  uint  sm_;
  uint  rx_pin_;
  uint  baud_;
  int   dma_chan_;
  uint  ring_mask_;      // (2^N - 1)
  uint8_t* ring_;        // 動的確保（2^N バイト）
  std::atomic<uint32_t> rp_{0}; // 読み位置
  std::atomic<bool>     arrived_{false};
  uint  prog_offset_{0};
  bool  installed_{false};

  // インスタンス→IRQブリッジ（PIO0/PIO1 それぞれの IRQ0 用）
  static PioUartRx* g_pio0_irq0_inst_;
  static PioUartRx* g_pio1_irq0_inst_;

  // どちらの PIO IRQ0 を使うか
  uint pioIrqNum_() const { return (pio_ == pio0) ? PIO0_IRQ_0 : PIO1_IRQ_0; }
  // このSMに対応する “RX FIFO not empty” ソース
  uint pioRxNemptySrc_() const { return (uint)(pis_sm0_rx_fifo_not_empty + sm_); }

  // IRQハンドラ本体（Cリンケージのスタブから呼ぶ）
  static void irq0HandlerPio0_();
  static void irq0HandlerPio1_();
};


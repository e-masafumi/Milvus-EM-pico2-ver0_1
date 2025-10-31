#include "pio_uart.hpp"
#include <cstring>
#include <cstdlib>

// 静的メンバ定義(IRQからは静的関数しか呼べないため,他アドレスに干渉しないようにヌルポインタ)
PioUartRx* PioUartRx::g_pio0_irq0_inst_ = nullptr;
PioUartRx* PioUartRx::g_pio1_irq0_inst_ = nullptr;

PioUartRx::PioUartRx(PIO pio, uint sm, uint rx_gpio, uint baud, uint8_t ring_pow2)
: pio_(pio), sm_(sm), rx_pin_(rx_gpio), baud_(baud) {
  // ---- リング確保 ----
  if (ring_pow2 < 8) ring_pow2 = 8;           // 最小 256B
  if (ring_pow2 > 15) ring_pow2 = 15;         // 最大 32KB（お好みで）
  size_t ring_size = 1u << ring_pow2;
  ring_mask_ = (uint)ring_size - 1u;
  ring_ = (uint8_t*)calloc(ring_size, 1);
  hard_assert(ring_ != nullptr);

  // ---- PIO プログラムをロード（公式サンプルの uart_rx_program を使用）----
  // これは framing error / stop bit 検査つき。同梱の init をそのまま使う。
  prog_offset_ = pio_add_program(pio_, &uart_rx_program);
  installed_ = true;

  // 初期化＆有効化（内側で pullup / in_pin / jmp_pin / fifo などを設定）
  uart_rx_program_init(pio_, sm_, prog_offset_, rx_pin_, baud_);

  // ---- DMA: PIO RX FIFO -> リング（8bit） ----
  dma_chan_ = dma_claim_unused_channel(true);
  dma_channel_config d = dma_channel_get_default_config(dma_chan_);
  channel_config_set_transfer_data_size(&d, DMA_SIZE_8);      // 8bit write
  channel_config_set_read_increment(&d, false);                // read固定
  channel_config_set_write_increment(&d, true);                // write進める
  channel_config_set_dreq(&d, pio_get_dreq(pio_, sm_, false)); // PIO RX DREQ
  // リング（2^N）を設定
  channel_config_set_ring(&d, true, ring_pow2);

  // ★ uart_rx_program はデータが FIFO の **上位バイト** に入る（左詰め）
  volatile void* src8 = (void*)(((uint8_t*)&pio_->rxf[sm_]) + 3);

  dma_channel_configure(
      dma_chan_, &d,
      ring_,           // write addr
      src8,            // read addr (PIO RX FIFO upper byte)
      0xFFFFFFFF,      // ほぼ無限
      true             // start
  );

  // ---- PIO “RX FIFO not empty” を IRQ0 で通知 ----
  pio_set_irq0_source_enabled(pio_, static_cast<pio_interrupt_source_t>(pis_sm0_rx_fifo_not_empty + sm_), true);

  // インスタンスをブリッジ登録して ISR ハンドラ差し込み
  if (pio_ == pio0) {
    g_pio0_irq0_inst_ = this;
    irq_set_exclusive_handler(PIO0_IRQ_0, &PioUartRx::irq0HandlerPio0_);
  } else {
    g_pio1_irq0_inst_ = this;
    irq_set_exclusive_handler(PIO1_IRQ_0, &PioUartRx::irq0HandlerPio1_);
  }
  irq_set_enabled(pioIrqNum_(), true);
}

PioUartRx::~PioUartRx() {
  irq_set_enabled(pioIrqNum_(), false);
  dma_channel_abort(dma_chan_);
  dma_channel_unclaim(dma_chan_);
  if (installed_) {
    pio_remove_program(pio_, &uart_rx_program, prog_offset_);
  }
  // SM を無効化（他で使っていなければ）
  pio_sm_set_enabled(pio_, sm_, false);
  free(ring_);
}

void PioUartRx::onRxFifoNotEmptyIrq() {
  // ※ ここでは何もしない（フラグだけ）。実処理はメイン側で。
  arrived_.store(true, std::memory_order_release);
}

// 生リング使用量
size_t PioUartRx::ringUsedBytes() const {
  uint32_t wp = dmaWritePosMasked_();
  uint32_t p  = rp_.load(std::memory_order_acquire);
  return (wp - p) & ring_mask_;
}

// 内部：'\n' まで探して 1 行コピー（先頭'#'以外は捨てる）
size_t PioUartRx::popLineInternal_(char* out, size_t cap) {
  if (cap == 0) return 0;

  uint32_t wp = dmaWritePosMasked_();
  uint32_t p  = rp_.load(std::memory_order_relaxed);
  if (p == wp) return 0;

  while (p != wp) {
    uint8_t c = ring_[p];
    p = (p + 1) & ring_mask_;
    if (c == '\n') {
      // 行長（LF除外）
      size_t len = (p - rp_.load(std::memory_order_relaxed) - 1) & ring_mask_;
      if (len == 0) { rp_.store(p, std::memory_order_release); return 0; }

      // 先頭チェック：'#' で始まる行だけ採用（'#' は残す）
			uint32_t q = rp_.load(std::memory_order_relaxed);	
			if (ring_[q] != '#') { rp_.store(p, std::memory_order_release); return 0; }	
			// '#' を含めて丸ごとコピー
			size_t copy_len = len;		
      size_t i = 0;
      while (i < copy_len && i + 1 < cap) {
        out[i++] = (char)ring_[q];
        q = (q + 1) & ring_mask_;
      }
      // 末尾が '\r' なら削除
      if (i > 0 && out[i - 1] == '\r') --i;
      out[i] = '\0';

      rp_.store(p, std::memory_order_release);
      return i;
    }
  }
  return 0; // まだ '\n' に到達していない
}

size_t PioUartRx::popLine(char* out, size_t cap) {
  // 新着データが少量だと 1 回で行に届かないことがあるので、複数回試みる
  size_t n = popLineInternal_(out, cap);
  return n;
}

void PioUartRx::irq0HandlerPio0_() {
  if (g_pio0_irq0_inst_) g_pio0_irq0_inst_->onRxFifoNotEmptyIrq();
  // SDKが源クリアを面倒見ますが、必要なら明示クリアも可能
}

void PioUartRx::irq0HandlerPio1_() {
  if (g_pio1_irq0_inst_) g_pio1_irq0_inst_->onRxFifoNotEmptyIrq();
}


#pragma once

#include <cstddef>
#include <cstdint>

#include <libhal/functional.hpp>

namespace hal::lpc40 {

constexpr std::uintptr_t dma_reg_address = 0x2008'0000;
constexpr std::size_t dma_max_transfer_size = (1 << 12) - 1;
constexpr std::size_t dma_channel_count = 8;

enum class dma_transfer_type : std::uint8_t
{
  /// Flow Control: DMA controller
  memory_to_memory = 0b000,
  /// Flow Control: DMA controller
  memory_to_peripheral = 0b001,
  /// Flow Control: DMA controller
  peripheral_to_memory = 0b010,
  /// Flow Control: DMA controller
  peripheral_to_peripheral = 0b011,
  /// Flow Control: Destination Peripheral
  peripheral_to_peripheral_dp = 0b100,
  /// Flow Control: Destination Peripheral
  memory_to_peripheral_dp = 0b101,
  /// Flow Control: Source Peripheral
  peripheral_to_memory_sp = 0b110,
  /// Flow Control: Source Peripheral
  peripheral_to_peripheral_sp = 0b111
};

enum class dma_channel_select : std::uint8_t
{
  channel0 = 0,
  channel1,
  channel2,
  channel3,
  channel4,
  channel5,
  channel6,
  channel7,
};

enum class dma_transfer_width : std::uint8_t
{
  bit_8 = 0b000,
  bit_16 = 0b001,
  bit_32 = 0b010,
};

enum class dma_burst_size : std::uint8_t
{
  bytes_1 = 0b000,    // - 1 bytes
  bytes_4 = 0b001,    // - 4 bytes
  bytes_8 = 0b010,    // - 8 bytes
  bytes_16 = 0b011,   // - 16 bytes
  bytes_32 = 0b100,   // - 32 bytes
  bytes_64 = 0b101,   // - 64 bytes
  bytes_128 = 0b110,  // - 128 bytes
  bytes_256 = 0b111,  // - 256 bytes
};

enum class dma_peripheral : std::uint8_t
{
  memory_or_timer0_match0 = 0,
  sd_card_and_timer0_match1 = 1,
  spi0_tx_and_timer1_match0 = 2,
  spi0_rx_and_timer1_match1 = 3,
  spi1_tx_and_timer2_match0 = 4,
  spi1_rx_and_timer2_match1 = 5,
  spi2_tx_and_i2s_channel_0 = 6,
  spi2_rx_and_i2s_channel_1 = 7,
  adc = 8,
  dac = 9,
  uart0_tx_and_uart3_tx = 10,
  uart0_rx_and_uart3_rx = 11,
  uart1_tx_and_uart4_tx = 12,
  uart1_rx_and_uart4_rx = 13,
  uart2_tx_and_timer3_match0 = 14,
  uart2_rx_and_timer3_match1 = 15,
};

struct dma
{
  void const volatile* source;
  void volatile* destination;
  std::size_t length;
  bool source_increment;
  bool destination_increment;
  dma_transfer_type transfer_type;
  dma_transfer_width source_transfer_width;
  dma_transfer_width destination_transfer_width;
  dma_peripheral source_peripheral = dma_peripheral::memory_or_timer0_match0;
  dma_peripheral destination_peripheral =
    dma_peripheral::memory_or_timer0_match0;
  dma_burst_size source_burst_size = dma_burst_size::bytes_1;
  dma_burst_size destination_burst_size = dma_burst_size::bytes_1;
};

void setup_dma_transfer(dma const& p_configuration,
                        hal::callback<void(void)> p_interrupt_callback);
}  // namespace hal::lpc40
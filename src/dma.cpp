#include <bit>
#include <cstddef>
#include <cstdint>

#include <array>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/dma.hpp>
#include <libhal-lpc40/power.hpp>
#include <libhal-util/bit.hpp>
#include <libhal-util/enum.hpp>
#include <libhal/functional.hpp>
#include <libhal/units.hpp>

namespace hal::lpc40 {

struct dma_channel
{
  volatile std::uint32_t source_address;
  volatile std::uint32_t destination_address;
  volatile std::uint32_t linked_list_item;
  volatile std::uint32_t control;
  volatile std::uint32_t config;
};

struct gpdma
{
  volatile std::uint32_t interrupt_status;
  volatile std::uint32_t interrupt_terminal_count_status;
  volatile std::uint32_t interrupt_terminal_count_clear;
  volatile std::uint32_t interrupt_error_status;
  volatile std::uint32_t interrupt_error_clear;
  volatile std::uint32_t raw_interrupt_terminal_count_status;
  volatile std::uint32_t raw_interrupt_error_status;
  volatile std::uint32_t enabled_channels;
  volatile std::uint32_t SOFTBREQ;
  volatile std::uint32_t SOFTSREQ;
  volatile std::uint32_t SOFTLBREQ;
  volatile std::uint32_t SOFTLSREQ;
  volatile std::uint32_t config;
  volatile std::uint32_t sync;
};

namespace dma_control {
constexpr auto transfer_size = hal::bit_mask::from<0, 11>();
constexpr auto source_burst_size = hal::bit_mask::from<12, 14>();
constexpr auto destination_burst_size = hal::bit_mask::from<15, 17>();
constexpr auto source_transfer_width = hal::bit_mask::from<18, 20>();
constexpr auto destination_transfer_width = hal::bit_mask::from<21, 23>();
constexpr auto source_increment = hal::bit_mask::from<26>();
constexpr auto destination_increment = hal::bit_mask::from<27>();
constexpr auto enable_terminal_count_interrupt = hal::bit_mask::from<31>();
}  // namespace dma_control

namespace dma_config {
// config masks
constexpr auto enable = hal::bit_mask::from<0>();
constexpr auto source_peripheral = hal::bit_mask::from<5, 1>();
constexpr auto destination_peripheral = hal::bit_mask::from<10, 6>();
constexpr auto transfer_type = hal::bit_mask::from<13, 11>();
constexpr auto terminal_count_interrupt_mask = hal::bit_mask::from<15>();
}  // namespace dma_config

auto* dma_reg = reinterpret_cast<gpdma*>(dma_reg_address);

constexpr inline std::uintptr_t dma_channel_offset(unsigned p_channel)
{
  return 0x100 + (p_channel * 0x20);
}

std::array<dma_channel*, dma_channel_count> dma_channel_reg{
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(0)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(1)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(2)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(3)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(4)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(5)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(6)),
  reinterpret_cast<dma_channel*>(dma_reg_address + dma_channel_offset(7)),
};

std::array<hal::callback<void(void)>, dma_channel_count> dma_callbacks{
  hal::cortex_m::interrupt::nop, hal::cortex_m::interrupt::nop,
  hal::cortex_m::interrupt::nop, hal::cortex_m::interrupt::nop,
  hal::cortex_m::interrupt::nop, hal::cortex_m::interrupt::nop,
  hal::cortex_m::interrupt::nop, hal::cortex_m::interrupt::nop,
};

void handle_dma_interrupt() noexcept
{
  // The zero count from the LSB tells you where the least significant 1 is
  // located. This allows the handled DMA interrupt callback to start at 0 and
  // end at the last bit.
  const auto status = std::countr_zero(dma_reg->interrupt_status);
  const auto clear_mask = 1 << status;

  dma_reg->interrupt_terminal_count_clear = clear_mask;
  dma_reg->interrupt_error_clear = clear_mask;

  // Call this channel's callback
  dma_callbacks[status]();
}

void initialize_dma()
{
  hal::lpc40::power_on(hal::lpc40::peripheral::gpdma);
  hal::cortex_m::interrupt::initialize<hal::value(hal::lpc40::irq::max)>();
  hal::cortex_m::interrupt(hal::value(hal::lpc40::irq::dma))
    .enable(handle_dma_interrupt);
  // Enable DMA & use default AHB endianness
  dma_reg->config = 1;
}

void setup_dma_transfer(const dma_configuration_t& p_configuration,
                        hal::callback<void(void)> p_interrupt_callback)
{
  const auto config_value =
    hal::bit_value()
      .insert<dma_config::source_peripheral>(
        hal::value(p_configuration.source_peripheral))
      .insert<dma_config::destination_peripheral>(
        hal::value(p_configuration.destination_peripheral))
      .set<dma_config::enable>()
      .insert<dma_config::transfer_type>(
        hal::value(p_configuration.transfer_type))
      .set<dma_config::terminal_count_interrupt_mask>()
      .to<std::uint32_t>();

  const auto control_value =
    hal::bit_value()
      .insert<dma_control::transfer_size>(p_configuration.length)
      .insert<dma_control::source_burst_size>(
        hal::value(p_configuration.source_burst_size))
      .insert<dma_control::destination_burst_size>(
        hal::value(p_configuration.destination_transfer_width))
      .insert<dma_control::source_transfer_width>(
        hal::value(p_configuration.source_transfer_width))
      .insert<dma_control::destination_transfer_width>(
        hal::value(p_configuration.destination_transfer_width))
      .insert<dma_control::source_increment>(p_configuration.source_increment)
      .insert<dma_control::destination_increment>(
        p_configuration.destination_increment)
      .set<dma_control::enable_terminal_count_interrupt>()
      .to<std::uint32_t>();

  // Busy wait until a channel is available
  while (true) {
    // Count the number of 1s until you reach a zero. That zero will be the
    // available channel. If that zero is 8, then all 8 channels are currently
    // in use and cannot service this request.
    const auto available_channel = std::countr_one(dma_reg->enabled_channels);

    if (available_channel < 8) {
      // Copy callback to the callbacks
      dma_callbacks[available_channel] = p_interrupt_callback;

      // Clear previous interrupts on this channel, if there were any
      dma_reg->interrupt_terminal_count_clear = 1 << available_channel;
      dma_reg->interrupt_error_clear = 1 << available_channel;

      auto* dma_channel = dma_channel_reg[available_channel];

      dma_channel->source_address =
        reinterpret_cast<std::uintptr_t>(p_configuration.source);
      dma_channel->destination_address =
        reinterpret_cast<std::uintptr_t>(p_configuration.destination);
      dma_channel->control = control_value;
      // This will start the dma transfer
      dma_channel->config = config_value;
      break;
    }
  }
}
}  // namespace hal::lpc40

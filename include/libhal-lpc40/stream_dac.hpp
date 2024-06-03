#pragma once

#include <libhal/io_waiter.hpp>
#include <libhal/stream_dac.hpp>

namespace hal::lpc40 {
class stream_dac_u16 : public hal::stream_dac_u16
{
public:
  stream_dac_u16(hal::io_waiter& p_waiter);

private:
  void driver_write(const hal::stream_dac_u16::samples& p_samples) override;

  hal::io_waiter* m_waiter;
};

class stream_dac_u8 : public hal::stream_dac_u8
{
public:
  stream_dac_u8(hal::io_waiter& p_waiter);

private:
  void driver_write(const hal::stream_dac_u8::samples& p_samples) override;

  hal::io_waiter* m_waiter;
};
}  // namespace hal::lpc40
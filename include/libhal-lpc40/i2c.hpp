// Copyright 2024 Khalil Estell
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//      http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <atomic>
#include <cstdint>
#include <span>
#include <system_error>

#include <libhal/i2c.hpp>

#include "constants.hpp"
#include "pin.hpp"

namespace hal::lpc40 {
class i2c : public hal::i2c
{
public:
  using write_iterator = std::span<const hal::byte>::iterator;
  using read_iterator = std::span<hal::byte>::iterator;

  /// port holds all of the information for an i2c bus on the LPC40xx
  /// platform.
  struct bus_info
  {
    /// peripheral id used to power on the i2c peripheral at creation
    peripheral peripheral_id;
    /// IRQ number for this i2c port.
    irq irq_number;
    /// i2c data pin
    pin sda;
    /// sda pin function code
    std::uint8_t sda_function;
    /// i2c clock pin
    pin scl;
    /// scl pin function code
    std::uint8_t scl_function;
    /// Clock rate duty cycle
    float duty_cycle = 0.5f;
  };

  /**
   * @brief Construct a new i2c object
   *
   * @param p_bus - i2c bus number, can be 0, 1, or 2.
   * @param p_settings - i2c configuration settings
   * @throws hal::operation_not_supported - if the settings or if the bus number
   * is not 0, 1, or 2.
   */
  i2c(std::uint8_t p_bus, const i2c::settings& p_settings = {});

  /**
   * @brief Construct a new i2c object using a bus info object
   *
   * @param p_bus_info - device specific bus information
   * @param p_settings - i2c configuration settings
   * @throws hal::operation_not_supported - if the settings or bus info
   * designation could not be achieved.
   */
  i2c(const bus_info& p_bus_info, const i2c::settings& p_settings = {});

  i2c(i2c& p_other) = delete;
  i2c& operator=(i2c& p_other) = delete;
  i2c(i2c&& p_other) noexcept = delete;
  i2c& operator=(i2c&& p_other) noexcept = delete;
  ~i2c();

private:
  void driver_configure(const settings& p_settings) override;
  void driver_transaction(
    hal::byte p_address,
    std::span<const hal::byte> p_data_out,
    std::span<hal::byte> p_data_in,
    hal::function_ref<hal::timeout_function> p_timeout) override;
  void setup_interrupt();
  void interrupt();

  enum class error_state
  {
    no_error = 0,
    no_such_device,
    io_error,
    arbitration_lost,
  };

  bus_info m_bus;
  write_iterator m_write_iterator;
  write_iterator m_write_end;
  read_iterator m_read_iterator;
  read_iterator m_read_end;
  error_state m_status{};
  hal::byte m_address = hal::byte{ 0x00 };
  bool m_busy = false;
};
}  // namespace hal::lpc40

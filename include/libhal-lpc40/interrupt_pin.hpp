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

#include <array>
#include <bit>
#include <cstdint>

#include <libhal/interrupt_pin.hpp>

namespace hal::lpc40 {
/**
 * @brief Interrupt pin implementation for the lpc40xx
 *
 */
class interrupt_pin : public hal::interrupt_pin
{
public:
  /**
   * @brief Get the interrupt pin object
   *
   * @param port - selects pin port to use
   * @param pin - selects pin within the port to use
   * @param p_settings - initial pin settings
   * @return result<interrupt_pin> - interrupt pin driver object
   */
  static result<interrupt_pin> get(std::uint8_t port,
                                   std::uint8_t pin,
                                   settings p_settings = {});

  interrupt_pin(interrupt_pin& p_other) = delete;
  interrupt_pin& operator=(interrupt_pin& p_other) = delete;
  interrupt_pin(interrupt_pin&& p_other) noexcept;
  interrupt_pin& operator=(interrupt_pin&& p_other) noexcept;
  ~interrupt_pin();

private:
  interrupt_pin(std::uint8_t p_port, std::uint8_t p_pin);

  status driver_configure(const settings& p_settings) override;
  void driver_on_trigger(hal::callback<handler> p_callback) override;

  uint8_t m_port;
  uint8_t m_pin;
  bool m_moved = false;
};

}  // namespace hal::lpc40
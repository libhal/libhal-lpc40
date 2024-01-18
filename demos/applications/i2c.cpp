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

#include <array>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/i2c.hpp>
#include <libhal-lpc40/uart.hpp>
#include <libhal-util/i2c.hpp>
#include <libhal-util/serial.hpp>
#include <libhal-util/steady_clock.hpp>

hal::status application()
{
  using namespace hal::literals;

  auto& clock = hal::lpc40::clock::get();
  const auto cpu_frequency = clock.get_frequency(hal::lpc40::peripheral::cpu);
  hal::cortex_m::dwt_counter steady_clock(cpu_frequency);

  std::array<hal::byte, 32> uart_buffer{};
  auto uart0 = HAL_CHECK(hal::lpc40::uart::get(0,
                                               uart_buffer,
                                               hal::serial::settings{
                                                 .baud_rate = 38400.0f,
                                               }));

  auto i2c2 = HAL_CHECK(hal::lpc40::i2c::get(2));

  while (true) {
    using namespace std::literals;

    constexpr hal::byte first_i2c_address = 0x08;
    constexpr hal::byte last_i2c_address = 0x78;

    hal::print(uart0, "Devices Found: ");

    for (hal::byte address = first_i2c_address; address < last_i2c_address;
         address++) {
      // This can only fail if the device is not present
      if (hal::probe(i2c2, address)) {
        hal::print<12>(uart0, "0x%02X ", address);
      }
    }

    print(uart0, "\n");
    hal::delay(steady_clock, 1s);
  }

  return hal::success();
}

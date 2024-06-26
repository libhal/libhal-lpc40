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

#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/interrupt_pin.hpp>
#include <libhal-lpc40/output_pin.hpp>

void application()
{
  hal::lpc40::interrupt_pin button(0, 29);
  hal::lpc40::output_pin led(1, 18);

  led.level(false);
  button.configure({});

  auto handler = [&led]([[maybe_unused]] bool p_level) {
    auto previous_state = led.level();
    (void)led.level(!previous_state);
  };

  button.on_trigger(handler);

  while (true) {
    continue;
  }
}

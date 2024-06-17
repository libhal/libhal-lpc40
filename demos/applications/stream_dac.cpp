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

#include <cmath>

#include <array>

#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/stream_dac.hpp>
#include <libhal/pwm.hpp>
#include <libhal/timer.hpp>

#include "resources/uniq-BOMBORA.u8.pcm.h"

std::array<std::uint8_t, 128> samples{};

void application()
{
  hal::lpc40::stream_dac_u8 dac;

  while (true) {
    // Change to 8'000.0f for LOFI
    dac.write({
      .sample_rate = 16'000.0f,
      .data = uniq_BOMBORA_u8_pcm,
    });
  }
}

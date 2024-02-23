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
#include <span>

#include <libhal-armcortex/dwt_counter.hpp>
#include <libhal-lpc40/clock.hpp>
#include <libhal-lpc40/constants.hpp>
#include <libhal-lpc40/output_pin.hpp>
#include <libhal-util/steady_clock.hpp>

// Application function must be implemented by one of the compilation units
// (.cpp) files.
extern void application();

int main()
{
  using namespace hal::literals;
  // Change the input frequency to match the frequency of the crystal attached
  // to the external OSC pins.
  hal::lpc40::maximum(10.0_MHz);

  // Run the application
  application();

  return 0;
}

[[noreturn]] void hal_terminate() noexcept
{
  hal::cortex_m::dwt_counter steady_clock(
    hal::lpc40::get_frequency(hal::lpc40::peripheral::cpu));

  hal::lpc40::output_pin led(1, 10);

  while (true) {
    using namespace std::chrono_literals;
    led.level(false);
    hal::delay(steady_clock, 100ms);
    led.level(true);
    hal::delay(steady_clock, 100ms);
    led.level(false);
    hal::delay(steady_clock, 100ms);
    led.level(true);
    hal::delay(steady_clock, 1000ms);
  }
}

namespace __cxxabiv1 {                                       // NOLINT
std::terminate_handler __terminate_handler = hal_terminate;  // NOLINT
}

extern "C"
{
  void _exit([[maybe_unused]] int rc)
  {
    std::terminate();
  }

  int kill(int, int)
  {
    return -1;
  }

  struct _reent* _impure_ptr = nullptr;  // NOLINT

  int getpid()
  {
    return 1;
  }

  std::array<std::uint8_t, 256> exception_storage{};

  void* __wrap___cxa_allocate_exception(unsigned int p_size)  // NOLINT
  {
    // Size of the GCC exception object header
    constexpr size_t header_size = 128;

    if (exception_storage.size() < header_size + p_size) {
      std::terminate();
    }

    // Required for GCC's impl to work correctly as it assumes that all bytes
    // default to 0.
    exception_storage.fill(0);
    return exception_storage.data() + header_size;
  }

  void __wrap___cxa_call_unexpected(void*)  // NOLINT
  {
    std::terminate();
  }

  void __wrap___cxa_free_exception(void*)  // NOLINT
  {
    // Clear the contents of the storage buffer as the exception runtime expects
    // the contents to already be cleared.
    exception_storage.fill(0);
  }
  void __assert_func([[maybe_unused]] const char* p_file,
                     [[maybe_unused]] int p_line,
                     [[maybe_unused]] const char* p_function,
                     [[maybe_unused]] const char* p_failed_expr)
  {
    abort();
  }
}  // extern "C"

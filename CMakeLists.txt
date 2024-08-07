# Copyright 2024 Khalil Estell
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.15)

project(libhal-lpc40 LANGUAGES CXX)

libhal_test_and_make_library(
  LIBRARY_NAME libhal-lpc40

  SOURCES
  src/power.cpp
  src/clock.cpp
  src/can.cpp
  src/pin.cpp
  src/pwm.cpp
  src/input_pin.cpp
  src/output_pin.cpp
  src/i2c.cpp
  src/uart.cpp
  src/interrupt_pin.cpp
  src/interrupt.cpp
  src/adc.cpp
  src/spi.cpp
  src/dma.cpp
  src/stream_dac.cpp

  TEST_SOURCES
  tests/adc.test.cpp
  tests/input_pin.test.cpp
  tests/output_pin.test.cpp
  tests/interrupt_pin.test.cpp
  tests/can.test.cpp
  tests/uart.test.cpp
  tests/i2c.test.cpp
  tests/pwm.test.cpp
  tests/spi.test.cpp
  tests/stream_dac.test.cpp
  tests/main.test.cpp

  PACKAGES
  libhal
  libhal-util
  libhal-armcortex
  libhal-soft
  ring-span-lite

  LINK_LIBRARIES
  libhal::libhal
  libhal::util
  libhal::armcortex
  libhal::soft
  nonstd::ring-span-lite
)

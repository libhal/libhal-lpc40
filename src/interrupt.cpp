#include <libhal-armcortex/interrupt.hpp>
#include <libhal-lpc40/constants.hpp>

namespace hal::lpc40 {
void initialize_interrupts()
{
  hal::cortex_m::initialize_interrupts<irq::max>();
}
}  // namespace hal::lpc40

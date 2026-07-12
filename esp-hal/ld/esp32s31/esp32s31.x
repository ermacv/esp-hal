/* ESP32-S31 RAM-loader linker integration. */
PROVIDE(interrupt0 = DefaultHandler);

SECTIONS {
  INCLUDE "rwtext.x"
  INCLUDE "rwdata.x"
}

INCLUDE "rodata.x"
INCLUDE "text.x"
INCLUDE "rtc_fast.x"
INCLUDE "stack.x"
INCLUDE "dram2.x"
INCLUDE "metadata.x"
INCLUDE "eh_frame.x"

_dram_data_start = ORIGIN(RAM);

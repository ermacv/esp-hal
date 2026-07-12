INCLUDE "memory.x"

REGION_ALIAS("ROTEXT", IRAM);
REGION_ALIAS("RODATA", RAM);
REGION_ALIAS("RWTEXT", IRAM);
REGION_ALIAS("RWDATA", RAM);
/* Retention placement is not supported yet; keep such data in normal RAM. */
REGION_ALIAS("RTC_FAST_RWTEXT", RAM);
REGION_ALIAS("RTC_FAST_RWDATA", RAM);

INCLUDE "esp32s31.x"
INCLUDE "hal-defaults.x"

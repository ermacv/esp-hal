# ESP32-S31 port notes

This document tracks the evidence used while adding ESP32-S31 support. It is a
working document for the `esp32s31-port` branch, not a statement of supported
functionality.

## Primary sources

- ESP-IDF ESP32-S31 SoC definitions at commit
  [`e88643bc`](https://github.com/espressif/esp-idf/tree/e88643bc619dff3c21b51705113bb488c9bc0990/components/soc/esp32s31)
- ESP32-S31 low-level SYSTIMER implementation at the same commit:
  [`components/esp_hal_systimer/esp32s31`](https://github.com/espressif/esp-idf/tree/e88643bc619dff3c21b51705113bb488c9bc0990/components/esp_hal_systimer/esp32s31)

Register addresses, interrupt numbers, GPIO signals, clock/reset controls and
capabilities must be derived from the ESP32-S31 definitions. Similarity to an
existing chip is useful for selecting reusable driver code, but is not evidence
that register layouts or capabilities match.

## Initial comparison

An exact file comparison against the ESP-IDF definitions for related RISC-V
chips found that ESP32-C5 has the largest common surface: 147 ESP32-S31 paths
also exist for C5. Only seven of those files are currently byte-identical:

- `huk_reg.h`
- `lp_i2c_ana_mst_reg.h`
- `mem_monitor_reg.h`
- `pau_reg.h`
- `pau_struct.h`
- `uart_reg.h`
- `uhci_reg.h`

The byte-identical UART register definition makes ESP32-C5 a strong initial
donor for the UART PAC and driver integration. CLIC and SYSTIMER are
structurally close but not identical, so their ESP32-S31 register definitions
and low-level implementations remain authoritative.

ESP32-P4 remains relevant for dual-core CLIC runtime and high-performance SoC
structure, but its ESP-IDF SoC directory has a substantially smaller direct
file overlap with ESP32-S31. GPIO, interrupt source enumeration, clock/reset,
cache/MMU and flash configuration must be treated as ESP32-S31-specific until
individual fields have been verified.

## Proven in the bring-up repository

The companion [`ermacv/esp32s31_rust`](https://github.com/ermacv/esp32s31_rust)
repository currently proves the following on ESP32-S31 revision 0.0:

- RAM loading with `espflash`;
- `esp-riscv-rt` with a 48-entry CLIC model;
- interrupt-matrix routing of `SYSTIMER_TARGET0` to CLIC input 17;
- a 1 MHz Embassy time driver backed by the 16 MHz, 52-bit SYSTIMER counter;
- an Embassy RISC-V thread executor sleeping with `wfi` between deadlines.

These implementations are bring-up code. They should be moved into the normal
`esp-hal` metadata, PAC, runtime and time-driver layers instead of being copied
as a permanent parallel HAL.

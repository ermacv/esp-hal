# Upstream tracking

This fork (`oer/main` at `github.com/ermacv/esp-hal`) follows
`github.com/esp-rs/esp-hal` `main` by periodic merges into `oer/main`. It is
never rebased: every pinned revision stays reachable, and each merge records
what it took. `main` in this repository mirrors upstream unchanged. Every
revision open-esp-radio-rs ever pinned carries a `pin/<sha>` tag, and every
deleted branch tip an `archive/<branch>` tag.

## Last merge

- Upstream: `c6fa25fcf060379de349f51c78dc6c69fe0948b5` (2026-09-28,
  "Fix ADC + radio on certain chips (#6412)").
- Previous merge base: `0eb3e53b4a2e555d2136ce9dc83e18c6692b9673`.

To prepare the next merge, `git fetch upstream` and review
`git log <last merged upstream>..upstream/main`. After resolving,
regenerate `esp-metadata-generated` with `cargo update-metadata` instead of
merging generated files by hand.

## Why the fork differs

open-esp-radio-rs runs its own ESP32-S31 radio drivers on esp-hal and places
code, data and task stacks in PSRAM after a two-stage boot. The fork keeps
only the ESP32-S31 support that needs:

- **Staged PSRAM boot.** `Psram::from_existing_mapping` adopts the mapping
  stage two already runs from and holds the PSRAM function clock for the
  rest of the program, so no other MPLL user can power it down under the
  executing code. `psram::prepare_code` and `soc::esp32s31::cache` make a
  copied image executable. Core 1 programs its per-core PMA for external
  memory before it loads a stack that may live there.
- **Shared-word ownership.** `MODEM_LPCON.CLK_CONF` holds the analog-I2C
  master, coexistence, low-power timer and Wi-Fi power gates. One lock
  guards every read-modify-write of that word, and the first three gates
  are reference-counted, exported through `clock::ll` as
  `acquire_*`/`release_*` so radio drivers request them instead of writing
  the word.
- **Flash timing.** `Flash::tune_120mhz` switches the bootloader-configured
  flash to 120 MHz STR only after checking every timing candidate against
  direct and uncached XIP reads. It is an ESP32-S31 extension of the
  upstream `Flash` driver.
- **Interrupted context.** `interrupt::interrupted_context()` returns the
  return address and stack pointer the running handler preempted, for a
  hang watchdog without a debugger.
- **Changed in fork: `BLE_LP_CLK` on ESP32-S31.** The ESP32-S31 presets set
  `ble_lp_clk: None`, so `esp_hal::init` writes neither `LP_TIMER_CONF`,
  `TEST_CONF` nor `RST_CONF`: the radio driver owns the Bluetooth low-power
  timer selection, and a second writer of those fields would race it. The
  node's gate goes through the counted `CLK_CONF` lock
  (`acquire_modem_low_power_timer_clock`) instead of writing the word. An
  upstream change to the S31 presets conflicts here at the next merge.
- **Brownout detector.** `rtc_cntl::brownout::configure` replaces the
  ESP-IDF bootloader's armed mode-1 brownout reset with the mode-0 detector,
  in the order of ESP-IDF's `esp_brownout_init`/`brownout_hal_config`
  (hardware system reset after the reset wait, no interrupt handler;
  `LP_ANA.INT_ENA` is left as found). Upstream
  configures no brownout detector. `LP_ANA` is declared in metadata for it.
- **ESP32-C5 PMP granularity.** The C5's PMP matches TOR boundaries at 128
  bytes, so `.rwtext` and the read-write data after it start 128-byte aligned
  (`ld/sections/rwtext.x`); an unaligned start put the tail of `.trap`, the
  vector table `bind_handler` writes, into the read-execute region and
  faulted the first handler binding with `enable-pmp` on. Other chips' link
  output is unchanged.
- **Internal-SRAM placement.** USB Serial/JTAG interrupt state and code stay
  in internal SRAM when ordinary mutable state lives in PSRAM.
- **Smaller fixes.** TIMG watchdog configuration is latched with
  `wdt_conf_update_en` before its clock is removed; the software-interrupt
  publication does not wait for a pending state the other core may already
  have consumed; the core-1 entry reaches its Rust body with `tail`, which
  links at any distance; Ethernet is opt-in through `__ethernet`; the TRNG,
  `I2C_ANA_MST`, `LP_TSENS` and `WIFI` peripherals are declared in metadata.
- **Panic-free reset path.** `pre_system_reset` on the ESP32-H4 and
  ESP32-S31 takes the other core by hart parity instead of `Cpu::current()`,
  whose unknown-hart branch panics, so a panic handler that resets cannot
  re-enter itself. Proposed upstream as esp-rs/esp-hal#6458.

## Upstream changes taken in place of fork code

- `0a60c6d4a` "ESP32-S31: configure TIMG clocks (#6365)": replaces the
  fork's timer-group and watchdog clock programming, which set the same
  fields.
- `ece91a496` "Add core flash driver (#6223)": replaces the fork's
  `Flash::from_bootloader` and `FlashInfo`; use `Flash::new` and
  `Flash::chip_info`. The timing sweep reports `TuningError`.
- `f769fcd27` "Add IPC, use CLINT software interrupt where possible
  (#6221)": the common RISC-V `start_core1_init` in `system::multi_core`
  replaces the fork's ESP32-S31 copy; the fork adds the PMA setup and
  `tail` there.

## Upstream changes not taken

None.

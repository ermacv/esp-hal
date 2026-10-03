//! ESP32-S31 independent LP true-random source control.

use crate::peripherals::{LP_PERI, RNG};

// See <https://github.com/espressif/esp-idf/blob/4d59230/components/hal/esp32s31/include/hal/rng_ll.h>
// (`rng_ll_enable`).
pub(crate) fn rng_ll_enable() {
    let lp_peri = LP_PERI::regs();
    let rng = RNG::regs();
    lp_peri
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_clk_en().set_bit());
    rng.date().modify(|_, w| w.clk_en().set_bit());
    lp_peri
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().set_bit());
    lp_peri
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().clear_bit());
    // One-hot noise source and sampling-enable selections, then the
    // repetition-count and adaptive-proportion health-test cutoffs.
    rng.conf().modify(|_, w| unsafe {
        w.noise_source_sel().bits(1 << 4);
        w.noise_pos_sel().bits(1 << 4);
        w.repetition_value_c().bits(0x1f);
        w.adpative_value_c().bits(0x12)
    });
    rng.debug_conf().modify(|_, w| unsafe {
        w.startup_test_limit().bits(1024);
        w.health_test_bypass().clear_bit()
    });
    rng.conf().modify(|_, w| {
        w.random_output_mode().set_bit();
        w.noise_crc_en().set_bit();
        w.sample_enable().set_bit()
    });
    rng.debug_conf()
        .modify(|_, w| w.startup_test_start().set_bit());
}

/// Enables the independent LP TRNG entropy source with its startup and
/// continuous health tests, as ESP-IDF's `rng_ll_enable` does.
pub(crate) fn ensure_randomness() {
    rng_ll_enable();
}

/// Leaves the LP TRNG running.
///
/// The TRNG is also the source `Rng` reads after a `TrngSource` is dropped;
/// ESP-IDF's `rng_ll_disable` would stop its clock and leave `Rng` reading a
/// halted generator.
pub(crate) fn revert_trng() {}

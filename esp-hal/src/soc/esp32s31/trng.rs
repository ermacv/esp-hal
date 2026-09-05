//! ESP32-S31 independent LP true-random source control.

use crate::peripherals::{LP_PERI, RNG};

/// Enables the independent LP TRNG entropy source.
pub(crate) fn ensure_randomness() {
    let clock = LP_PERI::regs();
    let trng = RNG::regs();

    clock.rng_ctrl().modify(|_, w| w.lp_rng_clk_en().set_bit());
    clock.rng_ctrl().modify(|_, w| w.lp_rng_rst_en().set_bit());
    clock
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_rst_en().clear_bit());
    trng.date().modify(|_, w| w.clk_en().set_bit());
    trng.conf()
        .modify(|_, w| w.sample_enable().set_bit().noise_crc_en().set_bit());
}

/// Disables the independent LP TRNG entropy source.
pub(crate) fn revert_trng() {
    let clock = LP_PERI::regs();
    let trng = RNG::regs();

    trng.conf()
        .modify(|_, w| w.sample_enable().clear_bit().noise_crc_en().clear_bit());
    trng.date().modify(|_, w| w.clk_en().clear_bit());
    clock
        .rng_ctrl()
        .modify(|_, w| w.lp_rng_clk_en().clear_bit());
}

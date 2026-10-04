//! # Brownout detector (ESP32-S31)
//!
//! The ESP-IDF bootloader leaves the analog mode-1 brownout reset armed, and
//! ESP-IDF's application startup (`esp_brownout_init`) replaces it with the
//! software-controlled mode-0 detector. [`configure`] performs that
//! replacement, in the register order of ESP-IDF 4d59230
//! `brownout_hal_config` (`components/esp_hal_pmu/brownout_hal.c`,
//! `esp32s31/include/hal/brownout_ll.h`).
//!
//! Mode 0 resets the digital system through the hardware reset after its
//! reset wait; this is ESP-IDF's variant without the brownout interrupt: no
//! handler is installed, and `LP_ANA.INT_ENA` is left as found.

use crate::{
    peripherals::{I2C_ANA_MST, LP_ANA},
    soc::regi2c,
};

/// The analog mode-1 control bit in `FIB_ENABLE`; clearing it gives mode 1 to
/// software (`BROWNOUT_DETECTOR_LL_FIB_ENABLE`).
const FIB_ENABLE_BOD_MODE1: u32 = 1 << 1;

/// Brownout reset wait, in detector cycles (ESP-IDF's fixed `0x3ff`).
const RESET_WAIT: u16 = 0x3ff;

/// Detector cycles before the brownout event (ESP-IDF's fixed `2`).
const INTR_WAIT: u16 = 2;

/// Mode-0 brownout detector configuration.
#[instability::unstable]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub struct BrownoutConfig {
    /// Threshold level 0..=7 (`I2C_BOD_THRESHOLD`); ESP-IDF's default level 7
    /// is about 2.4 V.
    pub threshold: u8,
    /// Suspend the flash when a brownout is detected.
    pub flash_power_down: bool,
    /// Power down the RF circuits when a brownout is detected.
    pub rf_power_down: bool,
}

impl Default for BrownoutConfig {
    /// ESP-IDF's defaults: level 7, flash suspend and RF power-down on.
    fn default() -> Self {
        Self {
            threshold: 7,
            flash_power_down: true,
            rf_power_down: true,
        }
    }
}

/// Replace the bootloader's mode-1 brownout reset with the mode-0 detector.
///
/// The threshold is an analog-I2C register, so the call borrows the analog
/// I2C master: whoever owns `I2C_ANA_MST` (a radio driver, once it starts)
/// is the only writer of the analog bus.
///
/// # Panics
///
/// Panics if `config.threshold` is above 7.
#[instability::unstable]
pub fn configure(_analog_bus: &mut I2C_ANA_MST<'_>, config: BrownoutConfig) {
    assert!(config.threshold <= 7, "brownout threshold is 0..=7");
    let lp_ana = LP_ANA::regs();

    // brownout_ll_ana_reset_enable(false): take mode 1 into software control,
    // then disarm its reset; it has the highest priority otherwise.
    lp_ana.fib_enable().modify(|r, w| unsafe {
        w.ana_fib_ena()
            .bits(r.ana_fib_ena().bits() & !FIB_ENABLE_BOD_MODE1)
    });
    lp_ana
        .bod_mode1_cntl()
        .modify(|_, w| w.bod_mode1_reset_ena().clear_bit());

    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| unsafe { w.bod_mode0_intr_wait().bits(INTR_WAIT) });
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_close_flash_ena().bit(config.flash_power_down));
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_pd_rf_ena().bit(config.rf_power_down));

    // brownout_ll_clear_count
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_cnt_clr().set_bit());
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_cnt_clr().clear_bit());

    // brownout_ll_reset_config(true, 0x3ff, BROWNOUT_RESET_LEVEL_SYSTEM)
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| unsafe { w.bod_mode0_reset_wait().bits(RESET_WAIT) });
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_reset_ena().set_bit());
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_reset_sel().set_bit());

    regi2c::I2C_BOD_THRESHOLD.write_field(config.threshold);

    // brownout_ll_bod_enable(true)
    lp_ana
        .bod_mode0_cntl()
        .modify(|_, w| w.bod_mode0_intr_ena().set_bit());
}

//! Bootstrap clock model for ESP32-S31.
#![allow(missing_docs)]

use esp_rom_sys::rom::ets_update_cpu_frequency_rom;

use crate::peripherals::{HP_SYS_CLKRST, LP_AON_CLKRST, PMU};

define_clock_tree_types!();

/// CPU clock frequency exposed by the initial port.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum CpuClock {
    /// 320 MHz, configured by ROM during the current bring-up path.
    #[default]
    _320MHz = 320,
}

impl From<CpuClock> for ClockConfig {
    fn from(_value: CpuClock) -> Self {
        Self {
            xtal_clk: Some(XtalClkConfig::_40),
            cpu_clk: Some(CpuClkConfig::_320),
            apb_clk: Some(ApbClkConfig::_80),
        }
    }
}

impl Default for ClockConfig {
    fn default() -> Self {
        CpuClock::default().into()
    }
}

impl ClockConfig {
    pub(crate) fn try_get_preset(self) -> Option<CpuClock> {
        Some(CpuClock::_320MHz)
    }

    pub(crate) fn configure(self, clocks: &mut ClockTree) {
        configure_cpu_320mhz();
        self.apply(clocks);
    }
}

/// Select the maximum clock plan documented by ESP-IDF for ESP32-S31:
/// CPLL 320 MHz -> CPU /1, MEM /2, SYS /3, APB /2.
///
/// The second-stage bootloader already enables and calibrates the 320 MHz
/// CPLL, but normally leaves the CPU divider at /4 (80 MHz). ESP-IDF changes
/// these dividers later in `esp_clk_init`; bare-metal applications must do the
/// equivalent during HAL initialization.
#[crate::ram]
fn configure_cpu_320mhz() {
    let regs = HP_SYS_CLKRST::regs();

    // The ROM RAM-download path may leave the CPU on XTAL after a true cold
    // boot. Power and calibrate CPLL explicitly, as rtc_clk_cpll_enable() and
    // rtc_clk_cpll_configure() do in ESP-IDF.
    unsafe {
        // The PAC follows the write-only SVD access, while ESP-IDF's
        // SET_PERI_REG_MASK reads this hardware register before writing it.
        let immediate_power = (0x200b_0000 + 0xf4) as *mut u32;
        immediate_power.write_volatile(
            immediate_power.read_volatile() | (1 << 19) | (1 << 23) | (1 << 27),
        );
        // HP_ALIVE_SYS is intentionally hidden from the public peripheral
        // list, but this clock gate is part of the documented CPLL sequence.
        let hp_clock_control = 0x2058_9000 as *mut u32;
        hp_clock_control.write_volatile(hp_clock_control.read_volatile() | (1 << 29));
    }
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_cpll_div()
        .modify(|_, w| unsafe {
            w.lp_aonclkrst_cpll_ref_div().bits(1);
            w.lp_aonclkrst_cpll_fb_div().bits(8)
        });
    regs.ana_pll_ctrl0()
        .modify(|_, w| w.reg_cpu_pll_cal_stop().clear_bit());
    while !regs
        .ana_pll_ctrl0()
        .read()
        .reg_cpu_pll_cal_end()
        .bit()
    {}
    crate::rom::ets_delay_us(10);
    regs.ana_pll_ctrl0()
        .modify(|_, w| w.reg_cpu_pll_cal_stop().set_bit());

    // A value of N-1 encodes the integer divider N. Fractional fields remain
    // zero for all clocks in this plan.
    regs.cpu_freq_ctrl0().write(|w| unsafe { w.bits(0) }); // /1
    regs.mem_freq_ctrl0().write(|w| unsafe { w.bits(1) }); // /2
    regs.sys_freq_ctrl0().write(|w| unsafe { w.bits(2) }); // /3
    regs.apb_freq_ctrl0().write(|w| unsafe { w.bits(1) }); // /2

    // Select CPLL and apply all staged clock changes atomically.
    regs.soc_clk_sel().modify(|_, w| unsafe {
        w.reg_soc_clk_sel().bits(1)
    });
    regs.root_clk_ctrl0()
        .write(|w| w.reg_soc_clk_update().set_bit());
    while regs
        .root_clk_ctrl0()
        .read()
        .reg_soc_clk_update()
        .bit_is_set()
    {}

    ets_update_cpu_frequency_rom(320);
}

fn configure_xtal_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<XtalClkConfig>,
    _new: XtalClkConfig,
) {
}

fn configure_cpu_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<CpuClkConfig>,
    _new: CpuClkConfig,
) {
}

fn configure_apb_clk_impl(
    _clocks: &mut ClockTree,
    _old: Option<ApbClkConfig>,
    _new: ApbClkConfig,
) {
}

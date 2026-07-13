//! Bootstrap clock model for ESP32-S31.
#![allow(missing_docs)]

use esp_rom_sys::rom::ets_update_cpu_frequency_rom;

use crate::peripherals::HP_SYS_CLKRST;

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
            xtal_clk: None,
            cpu_clk: None,
            apb_clk: None,
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

    // A value of N-1 encodes the integer divider N. Fractional fields remain
    // zero for all clocks in this plan.
    regs.cpu_freq_ctrl0().write(|w| unsafe { w.bits(0) }); // /1
    regs.mem_freq_ctrl0().write(|w| unsafe { w.bits(1) }); // /2
    regs.sys_freq_ctrl0().write(|w| unsafe { w.bits(2) }); // /3
    regs.apb_freq_ctrl0().write(|w| unsafe { w.bits(1) }); // /2

    // Apply all staged dividers atomically. The bootloader-selected source is
    // CPLL (SOC_CLK_SEL=1), so no PLL or MSPI source transition is required.
    debug_assert_eq!(regs.soc_clk_sel().read().bits() & 0x3, 1);
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

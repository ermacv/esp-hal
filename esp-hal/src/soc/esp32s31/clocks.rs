//! Bootstrap clock model for ESP32-S31.
#![allow(missing_docs)]

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
        self.apply(clocks);
    }
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

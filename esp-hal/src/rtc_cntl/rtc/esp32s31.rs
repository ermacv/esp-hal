//! ESP32-S31 reset reasons.

use strum::FromRepr;

use crate::soc::clocks::ClockConfig;

pub(crate) fn init(_config: &ClockConfig) {}

/// SoC reset reason values defined by ESP-IDF `reset_reasons.h`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, FromRepr)]
#[repr(usize)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum SocResetReason {
    ChipPowerOn   = 0x01,
    CoreSw        = 0x03,
    CoreDeepSleep = 0x05,
    CpuPmuPowerDown = 0x06,
    CoreMwdt0     = 0x07,
    CoreMwdt1     = 0x08,
    CoreRwdt      = 0x09,
    CpuMwdt       = 0x0b,
    CpuSw         = 0x0c,
    CpuRwdt       = 0x0d,
    SysBrownOut   = 0x0f,
    SysRwdt       = 0x10,
    SysSuperWdt   = 0x12,
    CorePowerGlitch = 0x13,
    CoreEfuseCrc  = 0x14,
    CoreUsbJtag   = 0x16,
    CoreUsbUart   = 0x17,
    CpuJtag       = 0x18,
    CpuLockup     = 0x1a,
}

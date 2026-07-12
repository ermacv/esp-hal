//! ESP32-S31 SoC support.

pub(crate) use esp32s31 as pac;

pub mod clocks;
pub(crate) mod regi2c;

#[inline(always)]
#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {}

pub(crate) fn enable_branch_predictor() {
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

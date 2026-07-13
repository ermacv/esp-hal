//! ESP32-S31 SoC support.

pub(crate) use esp32s31 as pac;

pub mod clocks;
pub(crate) mod regi2c;

#[inline(always)]
#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    // Match ESP-IDF's ESP32-S31 bring-up workaround (IDF-14620). On reset only
    // the HP CPU is in TEE mode, so the default control filters deny access to
    // every other bus master. Full TEE/APM setup is not supported yet.
    let lp_apm = unsafe { &*pac::LP_APM::ptr() };
    lp_apm.func_ctrl().write(|w| {
        w.m0_func_en()
            .clear_bit()
            .m1_func_en()
            .clear_bit()
            .m2_func_en()
            .clear_bit()
            .m3_func_en()
            .clear_bit()
    });
    let hp_apm = unsafe { &*pac::HP_APM::ptr() };
    hp_apm.func_ctrl().write(|w| {
        w.m0_func_en()
            .clear_bit()
            .m1_func_en()
            .clear_bit()
            .m2_func_en()
            .clear_bit()
            .m3_func_en()
            .clear_bit()
            .m4_func_en()
            .clear_bit()
            .m5_func_en()
            .clear_bit()
            .m6_func_en()
            .clear_bit()
    });
    let hp_mem_apm = unsafe { &*pac::HP_MEM_APM::ptr() };
    hp_mem_apm.func_ctrl().write(|w| {
        w.m0_func_en()
            .clear_bit()
            .m1_func_en()
            .clear_bit()
            .m2_func_en()
            .clear_bit()
            .m3_func_en()
            .clear_bit()
            .m4_func_en()
            .clear_bit()
            .m5_func_en()
            .clear_bit()
    });
}

/// Permit cached accesses to the external-memory virtual address aperture.
///
/// ESP32-S31 uses one vendor-specific PMA CSR per entry. ESP-IDF reserves
/// entry 7 for the 64 MiB EXTRAM range and notes that PSRAM is unreachable
/// without it.
pub(crate) fn enable_external_memory_pma() {
    const ADDRESS: u32 = (0x5000_0000 | ((0x0400_0000 / 2) - 1)) >> 2;
    const CONFIG: u32 = 0xc000_0000 // NAPOT
        | 0x2000_0000 // locked
        | 1 // enabled
        | (1 << 4) // read
        | (1 << 3) // write
        | (1 << 2); // execute

    unsafe {
        core::arch::asm!("csrw 0xbc7, zero", "csrw 0xbd7, zero");
        core::arch::asm!("csrw 0xbd7, {address}", address = in(reg) ADDRESS);
        core::arch::asm!("csrw 0xbc7, {config}", config = in(reg) CONFIG);
        core::arch::asm!("fence rw, rw", "fence.i");
    }
}

pub(crate) fn enable_branch_predictor() {
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

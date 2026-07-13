//! ESP32-S31 SoC support.

pub(crate) use esp32s31 as pac;

pub mod clocks;
pub(crate) mod cpu_control;
pub(crate) mod regi2c;

#[inline(always)]
#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    #[cfg(multi_core)]
    unsafe {
        // PMU stall state and HP clock/reset state can survive a software
        // reset. Keep Core 1 quiescent until CpuControl starts it explicitly.
        cpu_control::internal_park_core(crate::system::Cpu::AppCpu, true);
        cpu_control::disable_core1();
    }

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

    // ESP-IDF uses the same temporary bring-up policy for peripheral PMS:
    // grant read/write access from TEE and all three REE modes. These control
    // registers are contiguous u32 words from offset zero in each block.
    unsafe fn open_peripheral_pms(base: *mut u32, register_count: usize) {
        for offset in 0..register_count {
            unsafe { base.add(offset).write_volatile(0xff) };
        }
    }

    unsafe {
        // LP_SYSREG_CTRL .. LP_DAC_CTRL (offsets 0x00..=0x70).
        open_peripheral_pms(pac::LP_PERI_PMS::ptr().cast_mut().cast(), 29);
        // TRACE0_CTRL .. AXI_PERF_MON_CTRL (offsets 0x00..=0x78).
        open_peripheral_pms(pac::HP_PERI0_PMS::ptr().cast_mut().cast(), 31);
        // HP_USBOTG_PHY_CTRL .. HP_PERI1_PMS_CTRL (offsets 0x00..=0x98).
        open_peripheral_pms(pac::HP_PERI1_PMS::ptr().cast_mut().cast(), 39);
    }
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

/// Writes cached CPU data back so a non-coherent DMA master can observe it.
pub(crate) unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    const CACHE_LINE_SIZE: u32 = 64;
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    const CACHE_BASE: u32 = 0x2c00_0000;
    const SYNC_CTRL: *mut u32 = (CACHE_BASE + 0x9c) as *mut u32;
    const SYNC_MAP: *mut u32 = (CACHE_BASE + 0xa0) as *mut u32;
    const SYNC_ADDR: *mut u32 = (CACHE_BASE + 0xa4) as *mut u32;
    const SYNC_SIZE: *mut u32 = (CACHE_BASE + 0xa8) as *mut u32;
    const WRITEBACK_ENABLE: u32 = 1 << 2;
    const SYNC_DONE: u32 = 1 << 4;

    // ESP-IDF replaces the S31 ROM routine with this aligned, double-sync
    // sequence. The second operation is required by the cache hardware patch.
    let offset = addr & (CACHE_LINE_SIZE - 1);
    let aligned_addr = addr - offset;
    let aligned_size = (size + offset + CACHE_LINE_SIZE - 1) & !(CACHE_LINE_SIZE - 1);
    unsafe {
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
        SYNC_MAP.write_volatile(CACHE_MAP_L1_DCACHE);
        SYNC_ADDR.write_volatile(aligned_addr);
        SYNC_SIZE.write_volatile(aligned_size);
        for _ in 0..2 {
            SYNC_CTRL.write_volatile(WRITEBACK_ENABLE);
            while SYNC_CTRL.read_volatile() & SYNC_DONE == 0 {
                core::hint::spin_loop();
            }
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::SeqCst);
    }
}

/// Invalidates cached CPU data before reading memory written by DMA.
pub(crate) unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(cache_map: u32, addr: u32, size: u32);
    }
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    unsafe { Cache_Invalidate_Addr(CACHE_MAP_L1_DCACHE, addr, size) };
}

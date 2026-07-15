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

// The sync engine is shared by both cores and by every cache operation.  ROM
// cache functions don't serialize callers, so protect direct writeback and ROM
// invalidation with the same cross-core, interrupt-safe lock.
static CACHE_SYNC_LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

fn cache_aligned_range(addr: u32, size: u32) -> Option<(u32, u32)> {
    const CACHE_LINE_SIZE: u32 = 64;

    if size == 0 {
        return None;
    }

    let start = addr & !(CACHE_LINE_SIZE - 1);
    let end = addr
        .saturating_add(size)
        .saturating_add(CACHE_LINE_SIZE - 1)
        & !(CACHE_LINE_SIZE - 1);
    Some((start, end.saturating_sub(start)))
}

// ESP-IDF replaces the ESP32-S31 ROM writeback routine with a register-level
// implementation. The ROM routine can reject otherwise valid unaligned
// ranges, and the cache sync operation must be issued twice on this chip.
// Keep this helper inside CACHE_SYNC_LOCK: the CACHE sync registers are shared
// by both CPU cores and all cache-maintenance operations.
unsafe fn cache_writeback_addr_locked(addr: u32, size: u32) {
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    let Some((start, size)) = cache_aligned_range(addr, size) else {
        return;
    };
    let cache = unsafe { &*pac::CACHE::ptr() };

    cache
        .sync_map()
        .write(|w| unsafe { w.sync_map().bits(CACHE_MAP_L1_DCACHE as u8) });
    cache
        .sync_addr()
        .write(|w| unsafe { w.sync_addr().bits(start) });
    cache
        .sync_size()
        .write(|w| unsafe { w.sync_size().bits(size) });

    for _ in 0..2 {
        cache.sync_ctrl().write(|w| w.writeback_ena().set_bit());
        while !cache.sync_ctrl().read().sync_done().bit_is_set() {}
    }
}

unsafe fn cache_invalidate_addr_locked(cache_map: u32, addr: u32, size: u32) {
    let Some((start, size)) = cache_aligned_range(addr, size) else {
        return;
    };
    let cache = unsafe { &*pac::CACHE::ptr() };

    cache
        .sync_map()
        .write(|w| unsafe { w.sync_map().bits(cache_map as u8) });
    cache
        .sync_addr()
        .write(|w| unsafe { w.sync_addr().bits(start) });
    cache
        .sync_size()
        .write(|w| unsafe { w.sync_size().bits(size) });
    cache.sync_ctrl().write(|w| w.invalidate_ena().set_bit());
    while !cache.sync_ctrl().read().sync_done().bit_is_set() {}
}

/// Writes cached CPU data back so a non-coherent DMA master can observe it.
pub(crate) unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    CACHE_SYNC_LOCK.lock(|| unsafe { cache_writeback_addr_locked(addr, size) });
}

/// Invalidates cached CPU data before reading memory written by DMA.
pub(crate) unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    const CACHE_MAP_L1_DCACHE: u32 = 1 << 4;
    CACHE_SYNC_LOCK
        .lock(|| unsafe { cache_invalidate_addr_locked(CACHE_MAP_L1_DCACHE, addr, size) });
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum CachePrepareCodeError {
    WritebackFailed,
    InvalidateFailed,
}

/// Writes data bytes through to PSRAM and invalidates both instruction caches
/// before code at the same external-memory address is executed.
pub(crate) unsafe fn cache_prepare_code_addr(
    addr: u32,
    size: u32,
) -> Result<(), CachePrepareCodeError> {
    const CACHE_MAP_L1_ICACHE_0: u32 = 1 << 0;
    const CACHE_MAP_L1_ICACHE_1: u32 = 1 << 1;
    CACHE_SYNC_LOCK.lock(|| {
        unsafe { cache_writeback_addr_locked(addr, size) };
        unsafe {
            cache_invalidate_addr_locked(CACHE_MAP_L1_ICACHE_0 | CACHE_MAP_L1_ICACHE_1, addr, size)
        }
        Ok(())
    })
}

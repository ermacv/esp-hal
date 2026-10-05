//! # SOC (System-on-Chip) module (ESP32-C5)
//!
//! ## Overview
//!
//! The `SOC` module provides access, functions and structures that are useful
//! for interacting with various system-related peripherals on `ESP32-C5` chip.

crate::unstable_module! {
    pub mod clocks;
}
pub(crate) mod regi2c;

pub(crate) use esp32c5 as pac;

#[cfg(feature = "rt")]
pub(crate) fn riscv_preinit() {}

pub(crate) fn pre_init() {
    // Reset TEE security modes. This allows unrestricted access to TEE masters, including DMA.
    // FIXME: this is a temporary workaround until we have a proper solution for TEE security modes.
    for m in crate::peripherals::TEE::regs().m_mode_ctrl_iter() {
        m.reset();
    }
    // this is hacky, but for some reason we must reset the output enable register manually
    crate::peripherals::GPIO::regs().enable().reset();

    // Clear bit reset_event_bypass to ensure that the system bus is also reset during a core reset
    // (WDT), preventing bus freezing caused by an incorrect MSPI core reset in ROM. Mirrors
    // ESP-IDF's bootloader_hardware_init.
    crate::peripherals::PCR::regs()
        .reset_event_bypass()
        .modify(|_, w| w.reset_event_bypass().clear_bit());
}

pub(crate) fn enable_branch_predictor() {
    // Enable branch predictor
    // Note that the branch predictor will start cache requests and needs to be disabled when
    // the cache is disabled.
    // MHCR: CSR 0x7c1
    const MHCR_RS: u32 = 1 << 4; // R/W, address return stack set bit
    const MHCR_BFE: u32 = 1 << 5; // R/W, allow predictive jump set bit
    const MHCR_BTB: u32 = 1 << 12; // R/W, branch target prediction enable bit
    unsafe {
        core::arch::asm!("csrrs x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

#[cfg(feature = "unstable")]
pub(crate) fn disable_branch_predictor() {
    // Clears the bits set by `enable_branch_predictor`.
    const MHCR_RS: u32 = 1 << 4;
    const MHCR_BFE: u32 = 1 << 5;
    const MHCR_BTB: u32 = 1 << 12;
    unsafe {
        core::arch::asm!("csrrc x0, 0x7c1, {0}", in(reg) MHCR_RS | MHCR_BFE | MHCR_BTB);
    }
}

/// Writes back a specific range of data in the cache.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_WriteBack_Addr(addr: u32, size: u32);
    }

    unsafe {
        Cache_WriteBack_Addr(addr, size);
    }
}

/// Invalidate a specific range of addresses in the cache.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    unsafe extern "C" {
        fn Cache_Invalidate_Addr(addr: u32, size: u32);
    }
    unsafe {
        Cache_Invalidate_Addr(addr, size);
    }
}

/// Writes the shared instruction/data cache back over `addr..addr + size`, so
/// code the CPU copied there through the data path is in external memory
/// before it is fetched as instructions.
///
/// SOURCE: ESP-IDF 4d59230d
/// `components/esp_rom/patches/esp_rom_cache_writeback_esp32c5_esp32c61_esp32h4.c`
/// (`Cache_WriteBack_Addr`: line-aligned range, flash-cache map, the write-back
/// sync issued twice; `MIN_CACHE_LINE_SIZE` and `CACHE_MAP_FLASH_CACHE` from
/// `components/esp_rom/esp32c5/include/esp32c5/rom/cache.h:28,53`), selected for this chip by
/// `ESP_ROM_CACHE_WRITEBACK_NEEDS_SYNC_TWICE_NO_MAP`; the chip shares one cache
/// for instructions and data (`soc_caps.h` `SOC_SHARED_IDCACHE_SUPPORTED`).
#[crate::ram]
pub(crate) unsafe fn cache_prepare_code_addr(addr: u32, size: u32) {
    const CACHE_LINE_BYTES: u32 = 32;
    const CACHE_MAP_FLASH_CACHE: u8 = 1 << 4;
    if size == 0 {
        return;
    }
    let start = addr & !(CACHE_LINE_BYTES - 1);
    let size = (size + (addr - start) + CACHE_LINE_BYTES - 1) & !(CACHE_LINE_BYTES - 1);
    let cache = crate::peripherals::CACHE::regs();
    cache
        .sync_map()
        .write(|w| unsafe { w.sync_map().bits(CACHE_MAP_FLASH_CACHE) });
    cache
        .sync_addr()
        .write(|w| unsafe { w.sync_addr().bits(start) });
    cache
        .sync_size()
        .write(|w| unsafe { w.sync_size().bits(size) });
    for _ in 0..2 {
        cache.sync_ctrl().write(|w| w.writeback_ena().set_bit());
        while cache.sync_ctrl().read().sync_done().bit_is_clear() {}
    }
}

//! Cache sync engine shared by DMA maintenance and executable PSRAM.

const DCACHE: u8 = 1 << 4;
const BOTH_ICACHES: u8 = (1 << 0) | (1 << 1);
const CACHE_LINE_BYTES: u32 = 64;

// The engine's address, size and command registers are shared by both cores.
// Serialize the entire operation, including writeback then I-cache invalidate.
// Keep the lock usable even when the application places ordinary data in PSRAM.
#[unsafe(link_section = ".data.critical.cache_sync")]
static SYNC_LOCK: esp_sync::RawMutex = esp_sync::RawMutex::new();

#[inline(always)]
fn set_range(map: u8, addr: u32, size: u32) {
    let start = addr & !(CACHE_LINE_BYTES - 1);
    // Callers provide a valid mapped range; cache-line rounding must not wrap.
    let end = addr
        .checked_add(size)
        .and_then(|end| end.checked_add(CACHE_LINE_BYTES - 1))
        .expect("cache range overflows")
        & !(CACHE_LINE_BYTES - 1);
    let cache = crate::peripherals::CACHE::regs();
    cache
        .sync_map()
        .write(|w| unsafe { w.sync_map().bits(map) });
    cache
        .sync_addr()
        .write(|w| unsafe { w.sync_addr().bits(start) });
    cache
        .sync_size()
        .write(|w| unsafe { w.sync_size().bits(end - start) });
}

#[inline(always)]
fn writeback(addr: u32, size: u32) {
    set_range(DCACHE, addr, size);
    let cache = crate::peripherals::CACHE::regs();
    // ESP-IDF's S31 ROM override issues writeback twice. The ROM implementation
    // also rejects unaligned ranges, so align before using the sync engine.
    for _ in 0..2 {
        // SYNC_CTRL resets with INVALIDATE_ENA set; start from zero so the
        // writeback command cannot accidentally request both operations.
        unsafe {
            cache
                .sync_ctrl()
                .write_with_zero(|w| w.writeback_ena().set_bit())
        };
        while !cache.sync_ctrl().read().sync_done().bit_is_set() {}
    }
}

#[inline(always)]
fn invalidate(map: u8, addr: u32, size: u32) {
    set_range(map, addr, size);
    let cache = crate::peripherals::CACHE::regs();
    unsafe {
        cache
            .sync_ctrl()
            .write_with_zero(|w| w.invalidate_ena().set_bit())
    };
    while !cache.sync_ctrl().read().sync_done().bit_is_set() {}
}

/// Writes cached CPU data back so a non-coherent DMA master can observe it.
///
/// # Safety
/// The range must be mapped and no other writer may modify these cache lines.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_writeback_addr(addr: u32, size: u32) {
    if size != 0 {
        SYNC_LOCK.lock(|| writeback(addr, size));
    }
}

/// Invalidates cached CPU data before reading memory written by DMA.
///
/// # Safety
/// The range must be mapped; dirty data in its cache lines must be disposable.
#[doc(hidden)]
#[crate::ram]
pub unsafe fn cache_invalidate_addr(addr: u32, size: u32) {
    if size != 0 {
        SYNC_LOCK.lock(|| invalidate(DCACHE, addr, size));
    }
}

#[crate::ram]
pub(crate) unsafe fn cache_prepare_code_addr(addr: u32, size: u32) {
    if size != 0 {
        SYNC_LOCK.lock(|| {
            writeback(addr, size);
            invalidate(BOTH_ICACHES, addr, size);
        });
    }
}

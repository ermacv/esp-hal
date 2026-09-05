#![cfg_attr(docsrs, procmacros::doc_replace)]
//! # PSRAM (Pseudo-static RAM, SPI RAM) driver
//!
//! ## Overview
//!
//! This module provides support to interface with `PSRAM` devices connected to the MCU.
//! PSRAM provides additional external memory to supplement the internal memory of the MCU,
//! allowing for increased storage capacity and improved performance in certain applications.
#![doc = ""]
#![cfg_attr(
    esp32s3,
    doc = concat!("The ", chip_pretty!(), " can use either Quad SPI or Octal SPI to interface with PSRAM.
        `esp-hal` will try to automatically detect the best option, but manual configuration is also possible and more reliable.")
)]
#![doc = ""]
//! ## Examples
//!
//! ### PSRAM as heap memory
//!
//! This example shows how to use PSRAM as heap-memory via esp-alloc.
//!
//! <section class="warning">
//! The PSRAM example <em>must</em> be built in release mode!
//! </section>
//!
//! ```rust, ignore
//! # {before_snippet}
//! extern crate alloc;
//! use alloc::{string::String, vec::Vec};
//!
//! // Add PSRAM to the heap.
//! esp_alloc::psram_allocator!(peripherals.PSRAM, esp_hal::psram);
//!
//! let mut large_vec: Vec<u32> = Vec::with_capacity(500 * 1024 / 4);
//!
//! for i in 0..(500 * 1024 / 4) {
//!     large_vec.push((i & 0xff) as u32);
//! }
//!
//! let string = String::from("A string allocated in PSRAM");
//! # {after_snippet}
//! ```

use core::ops::Range;

#[cfg(esp32s31)]
mod mapping;

#[cfg(any(esp32s2, esp32s3))]
mod quad_xtensa;

#[cfg_attr(esp32, path = "esp32.rs")]
#[cfg_attr(esp32s2, path = "esp32s2.rs")]
#[cfg_attr(esp32s3, path = "esp32s3.rs")]
#[cfg_attr(any(esp32c5, esp32c61), path = "esp32c5_c61.rs")]
#[cfg_attr(esp32p4, path = "esp32p4.rs")]
#[cfg_attr(esp32s31, path = "esp32s31.rs")]
pub(crate) mod implem;

pub use implem::*;
use portable_atomic::{AtomicUsize, Ordering};

use crate::peripherals::PSRAM;

/// Size of PSRAM.
///
/// [`PsramSize::AutoDetect`] tries to detect the size of PSRAM.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramSize {
    /// Detect PSRAM size
    #[default]
    AutoDetect,
    /// A fixed PSRAM size.
    Size(usize),
}

impl PsramSize {
    pub(crate) fn get(&self) -> usize {
        match self {
            PsramSize::AutoDetect => 0,
            PsramSize::Size(size) => *size,
        }
    }

    pub(crate) fn is_auto(&self) -> bool {
        matches!(self, PsramSize::AutoDetect)
    }
}

const EXTMEM_ORIGIN: usize = property!("psram.extmem_origin");

static MAPPED_PSRAM_START: AtomicUsize = AtomicUsize::new(0);
static MAPPED_PSRAM_END: AtomicUsize = AtomicUsize::new(0);

pub(crate) fn psram_range() -> Range<usize> {
    let end = MAPPED_PSRAM_END.load(Ordering::Acquire);
    let start = MAPPED_PSRAM_START.load(Ordering::Relaxed);
    if end < start { 0..0 } else { start..end }
}

/// # Safety
///
/// Must only be called once.
unsafe fn set_psram_range(range: Range<usize>) {
    MAPPED_PSRAM_START.store(range.start, Ordering::Relaxed);
    MAPPED_PSRAM_END.store(range.end, Ordering::Release);
}

/// Enables externally-connected Pseudo-static RAM.
pub struct Psram {
    _peri: PSRAM<'static>,
}

impl Psram {
    /// Initializes PSRAM.
    pub fn new(peri: PSRAM<'static>, mut config: PsramConfig) -> Self {
        if init_psram(&mut config) {
            info!("PSRAM size: {} MB", config.size.get() / 1_024 / 1_024);
            let range = map_psram(config);

            unsafe { set_psram_range(range) };
        }
        Self { _peri: peri }
    }

    /// Adopts PSRAM initialized and mapped by an earlier boot stage.
    ///
    /// This records the mapping without resetting the device, clocks or caches.
    ///
    /// # Safety
    ///
    /// `range` must be a nonempty, live PSRAM mapping that remains valid while
    /// this driver is in use. PSRAM must not have been initialized or adopted
    /// by another owner in this image, and the previous boot stage must have
    /// relinquished its peripheral ownership.
    pub unsafe fn from_existing_mapping(peri: PSRAM<'static>, range: Range<usize>) -> Self {
        assert!(range.start < range.end, "PSRAM mapping must be nonempty");
        unsafe { set_psram_range(range) };
        Self { _peri: peri }
    }

    /// Returns the address and size of the available in external memory.
    pub fn raw_parts(&self) -> (*mut u8, usize) {
        let range = psram_range();
        (range.start as *mut u8, range.end - range.start)
    }
}

/// Failure to synchronize executable PSRAM.
#[cfg(esp32s31)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum PsramCacheError {
    /// The requested bytes are outside the live PSRAM mapping.
    InvalidAddress,
}

/// Publishes copied PSRAM code to both instruction caches.
///
/// # Safety
///
/// The bytes must contain valid code and must not be modified or executed
/// during synchronization. Other cores must remain quiescent until the caller
/// publishes the entry point; each executing core needs an instruction fence
/// before entering code it could previously have fetched.
#[cfg(esp32s31)]
#[crate::ram]
pub unsafe fn prepare_code(ptr: *const u8, size: usize) -> Result<(), PsramCacheError> {
    let start = ptr as usize;
    if !mapping::contains(psram_range(), start, size) {
        return Err(PsramCacheError::InvalidAddress);
    }
    unsafe { crate::soc::cache_prepare_code_addr(start as u32, size as u32) };
    unsafe { core::arch::asm!("fence.i") };
    Ok(())
}

/// Writes dirty PSRAM cache lines back before a non-coherent peripheral reads them.
///
/// Uses the same cross-core sync-engine lock as HAL DMA and [`prepare_code`].
///
/// # Safety
///
/// The range must be live mapped PSRAM. The complete cache lines containing it
/// must be exclusively owned by the caller and cannot be modified until the
/// peripheral has finished reading. Cache-line alignment may extend the range.
#[cfg(esp32s31)]
#[crate::ram]
pub unsafe fn writeback_for_dma(ptr: *const u8, size: usize) {
    unsafe { crate::soc::cache_writeback_addr(ptr as u32, size as u32) };
}

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
    psram_octal_spi,
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

/// Size of PSRAM
///
/// [PsramSize::AutoDetect] will try to detect the size of PSRAM
#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramSize {
    /// Detect PSRAM size
    #[default]
    AutoDetect,
    /// A fixed PSRAM size
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

/// Error returned while making newly written PSRAM bytes executable.
#[cfg(esp32s31)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PsramCacheError {
    /// The ROM could not write modified data-cache lines back to PSRAM.
    WritebackFailed,
    /// The ROM could not invalidate the instruction-cache lines for the range.
    InvalidateFailed,
}

/// Makes bytes written through the data cache visible to instruction fetches.
///
/// Call this after copying code into PSRAM and before executing that code.
///
/// # Errors
///
/// Returns [`PsramCacheError`] if the ROM rejects either cache operation.
///
/// # Safety
///
/// `address..address + size` must be a valid mapped PSRAM range, and no core
/// may execute from the range while it is being modified.
#[cfg(esp32s31)]
#[instability::unstable]
pub unsafe fn prepare_code(address: *const u8, size: usize) -> Result<(), PsramCacheError> {
    unsafe { crate::soc::cache_prepare_code_addr(address as u32, size as u32) }.map_err(
        |error| match error {
            crate::soc::CachePrepareCodeError::WritebackFailed => PsramCacheError::WritebackFailed,
            crate::soc::CachePrepareCodeError::InvalidateFailed => {
                PsramCacheError::InvalidateFailed
            }
        },
    )?;
    unsafe { core::arch::asm!("fence.i", options(nostack)) };
    Ok(())
}

/// # Safety
///
/// This function must only be called once.
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
            let range = map_psram(config);

            unsafe { set_psram_range(range) };
        }
        Self { _peri: peri }
    }

    /// Adopts a PSRAM mapping initialized by an earlier execution stage.
    ///
    /// This does not configure the PSRAM device, cache, or MMU. It only makes
    /// the existing mapping known to this program so address validation in DMA
    /// drivers and [`Self::raw_parts`] use the correct range. This is intended
    /// for a second-stage program entered without resetting the chip.
    ///
    /// # Panics
    ///
    /// Panics if `mapped_range` is empty.
    ///
    /// # Safety
    ///
    /// The caller must guarantee that `mapped_range` is the exact, live PSRAM
    /// mapping configured by an earlier execution stage and remains mapped for
    /// the lifetime of the returned owner. No other [`Psram`] owner may exist
    /// in the current execution stage, and the mapping must not be changed
    /// while the returned owner is alive.
    pub unsafe fn from_existing_mapping(peri: PSRAM<'static>, mapped_range: Range<usize>) -> Self {
        assert!(!mapped_range.is_empty(), "PSRAM mapping must not be empty");
        unsafe { set_psram_range(mapped_range) };
        Self { _peri: peri }
    }

    /// Returns the address and size of the available in external memory.
    pub fn raw_parts(&self) -> (*mut u8, usize) {
        let range = psram_range();
        (range.start as *mut u8, range.end - range.start)
    }
}

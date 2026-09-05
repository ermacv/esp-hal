//! ESP32-S31 bootloader-configured flash and synchronous MSPI timing tuning.
//!
//! The caller owns the boot policy, scratch storage and disposable XIP mapping.
//! This module preserves an existing ROM configuration and verifies 120 MHz
//! timing against direct reads before publishing the selected setting.

mod tuning;

use crate::peripherals::{FLASH, HP_SYS_CLKRST, SPI0, SPI1};

/// Information detected from the external SPI flash.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashInfo {
    /// JEDEC manufacturer, memory type, and capacity identifier.
    pub jedec_id: u32,
    /// Capacity in bytes.
    pub size: u32,
}

/// Failure to adopt or tune the bootloader-configured flash.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    /// The ROM did not detect a plausible JEDEC identifier.
    InvalidJedecId,
    /// A ROM flash read failed.
    ReadFailed,
    /// The requested address range is outside the detected flash device.
    InvalidAddress,
    /// No stable 120 MHz flash timing window was found.
    TimingTuningFailed,
}

/// Result of the ESP32-S31 120 MHz flash timing sweep.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashTiming {
    /// Config-table entry selected from the middle of the stable window.
    pub config_index: u8,
    /// One bit per timing-table entry; set bits passed the reference read.
    pub direct_pass_mask: u16,
    /// One bit per table entry that also passed an uncached XIP read.
    pub cache_pass_mask: u16,
}

/// A flash region mapped into the CPU's XIP address space.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct FlashXipRegion {
    /// Physical byte address in external flash.
    pub physical_start: u32,
    /// Corresponding CPU virtual address.
    pub virtual_start: usize,
    /// Number of mapped bytes available for verification.
    pub size: usize,
}

// This is the 120 MHz STR table used by ESP-IDF for the closely related P4,
// C5 and C61 MSPI timing block. Keeping it in internal data RAM is required:
// flash is deliberately unreadable while the sweep changes cache timing.
#[unsafe(link_section = ".data.flash_timing")]
static FLASH_120MHZ_TIMING: [(u8, u8, u8); 12] = [
    (2, 0, 1),
    (0, 0, 0),
    (2, 2, 2),
    (2, 1, 2),
    (2, 0, 2),
    (0, 0, 1),
    (2, 2, 3),
    (2, 1, 3),
    (2, 0, 3),
    (0, 0, 2),
    (2, 2, 4),
    (2, 1, 4),
];

const FLASH_TUNING_XIP_PAGE_BYTES: usize = 0x1000;
const FLASH_TUNING_XIP_PAGE_WORDS: usize = FLASH_TUNING_XIP_PAGE_BYTES / size_of::<u32>();
const FLASH_TUNING_MAX_XIP_PAGES: usize = 31;

/// Number of words required by [`Flash::tune_120mhz`] for its XIP reference.
///
/// The storage must live in internal SRAM because flash cache is suspended
/// while the tuning candidates are checked.
pub const FLASH_TUNING_SCRATCH_WORDS: usize =
    FLASH_TUNING_MAX_XIP_PAGES * FLASH_TUNING_XIP_PAGE_WORDS;

unsafe extern "C" {
    static rom_spiflash_legacy_data: *const u32;
    fn esp_rom_spiflash_read(address: u32, destination: *mut u32, length: u32) -> i32;
    fn Cache_Suspend_L1_CORE0_ICache() -> u32;
    fn Cache_Resume_L1_CORE0_ICache(autoload: u32);
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn set_flash_clock_120mhz() {
    unsafe {
        HP_SYS_CLKRST::regs().flash_ctrl0().modify(|_, w| {
            w.sys_clk_en().set_bit();
            w.pll_clk_en().set_bit();
            w.core_clk_en().set_bit();
            w.clk_src_sel().bits(1);
            // SPLL is 480 MHz: divider field stores divisor minus one.
            w.core_clk_div_num().bits(3)
        });
        SPI0::regs().clock().write(|w| w.clk_equ_sysclk().set_bit());
        SPI1::regs().clock().write(|w| w.clk_equ_sysclk().set_bit());
        SPI0::regs().ctrl().modify(|_, w| w.fdummy_rin().set_bit());
        SPI1::regs().ctrl().modify(|_, w| w.fdummy_rin().set_bit());
    }
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn set_flash_timing(din_mode: u8, din_num: u8, extra_dummy: u8, apply_to_cache: bool) {
    unsafe {
        let spi0 = SPI0::regs();
        spi0.din_mode().write(|w| {
            w.din0_mode().bits(din_mode);
            w.din1_mode().bits(din_mode);
            w.din2_mode().bits(din_mode);
            w.din3_mode().bits(din_mode);
            w.din4_mode().bits(din_mode);
            w.din5_mode().bits(din_mode);
            w.din6_mode().bits(din_mode);
            w.din7_mode().bits(din_mode);
            w.dins_mode().bits(din_mode)
        });
        spi0.din_num().write(|w| {
            w.din0_num().bits(din_num);
            w.din1_num().bits(din_num);
            w.din2_num().bits(din_num);
            w.din3_num().bits(din_num);
            w.din4_num().bits(din_num);
            w.din5_num().bits(din_num);
            w.din6_num().bits(din_num);
            w.din7_num().bits(din_num);
            w.dins_num().bits(din_num)
        });
        if apply_to_cache {
            spi0.timing_cali().write(|w| {
                let w = w.timing_clk_ena().set_bit();
                if extra_dummy == 0 {
                    w.timing_cali().clear_bit()
                } else {
                    w.timing_cali().set_bit()
                };
                w.extra_dummy_cyclelen().bits(extra_dummy);
                w.update().set_bit()
            });
        } else {
            // DIN mode/number are shared. Latch them without changing SPI0's
            // cache dummy count while SPI1 performs the candidate read.
            spi0.timing_cali()
                .modify(|_, w| w.timing_clk_ena().set_bit().update().set_bit());
        }

        SPI1::regs().timing_cali().write(|w| {
            if extra_dummy == 0 {
                w.timing_cali().clear_bit()
            } else {
                w.timing_cali().set_bit()
            };
            w.extra_dummy_cyclelen().bits(extra_dummy)
        });
        // SPI1 has no update bit; SPI0's update latches the shared delay path.
        spi0.timing_cali()
            .modify(|_, w| w.timing_clk_ena().set_bit().update().set_bit());
    }
}

/// An attached external SPI flash device.
pub struct Flash {
    _peri: FLASH<'static>,
    info: FlashInfo,
}

impl Flash {
    /// Uses the flash configuration established by the normal bootloader.
    pub fn from_bootloader(peri: FLASH<'static>) -> Result<Self, FlashError> {
        let jedec_id = unsafe {
            let descriptor = core::ptr::addr_of!(rom_spiflash_legacy_data).read_volatile();
            if descriptor.is_null() || !descriptor.is_aligned() {
                return Err(FlashError::InvalidJedecId);
            }
            descriptor.read_volatile()
        };
        let capacity_bits = jedec_id & 0xff;
        if jedec_id == 0 || jedec_id == 0x00ff_ffff || capacity_bits > 31 {
            return Err(FlashError::InvalidJedecId);
        }

        Ok(Self {
            _peri: peri,
            info: FlashInfo {
                jedec_id,
                size: 1u32 << capacity_bits,
            },
        })
    }

    /// Returns information detected while attaching the device.
    pub fn info(&self) -> FlashInfo {
        self.info
    }

    /// Tunes and switches the ESP32-S31 flash bus to 120 MHz STR mode.
    ///
    /// # Safety
    ///
    /// This must run before CPU1 is started, while no interrupt or DMA handler
    /// can access flash. The routine disables CPU0's instruction cache and
    /// interrupts while changing the shared MSPI timing path. Its own code and
    /// timing table are explicitly placed in internal RAM. `xip` must describe
    /// a valid, mapped flash region containing at least 15 distinct 4 KiB
    /// pages. `mapped_reference` must contain at least
    /// [`FLASH_TUNING_SCRATCH_WORDS`] words in internal SRAM. Treat the XIP
    /// region as disposable after this call: rejected timing candidates can
    /// leave their dedicated page cached with corrupted data. These pages must
    /// be cold on entry and must not contain live code or data. The caller's
    /// stack and all storage accessed during tuning must also be internal SRAM.
    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub unsafe fn tune_120mhz(
        &mut self,
        xip: FlashXipRegion,
        mapped_reference: &mut [u32],
    ) -> Result<FlashTiming, FlashError> {
        let page_count = tuning::page_count(
            xip.physical_start,
            xip.virtual_start,
            xip.size,
            self.info.size,
        )
        .ok_or(FlashError::InvalidAddress)?;
        if mapped_reference.len() < FLASH_TUNING_SCRATCH_WORDS {
            return Err(FlashError::TimingTuningFailed);
        }
        let mut reference = [0u32; 32];
        if unsafe { esp_rom_spiflash_read(0, reference.as_mut_ptr(), 128) } != 0 {
            return Err(FlashError::ReadFailed);
        }
        // A 128-byte prefix was insufficient on the ESP32-S31 Function
        // CoreBoard: a candidate could pass every prefix and still corrupt
        // other cache lines in the same page. Keep the complete direct-read
        // reference for each sampled page in caller-owned internal SRAM, then
        // require every word to survive the candidate timing. Caller-owned
        // storage avoids a 124-KiB stack frame during early boot.
        let mut sample = 0usize;
        while sample < page_count {
            let physical = xip.physical_start + sample as u32 * FLASH_TUNING_XIP_PAGE_BYTES as u32;
            if unsafe {
                esp_rom_spiflash_read(
                    physical,
                    mapped_reference
                        .as_mut_ptr()
                        .add(sample * FLASH_TUNING_XIP_PAGE_WORDS),
                    FLASH_TUNING_XIP_PAGE_BYTES as u32,
                )
            } != 0
            {
                return Err(FlashError::ReadFailed);
            }
            sample += 1;
        }

        let original_flash_ctrl = HP_SYS_CLKRST::regs().flash_ctrl0().read().bits();
        let original_spi0_clock = SPI0::regs().clock().read().bits();
        let original_spi1_clock = SPI1::regs().clock().read().bits();
        let original_spi0_ctrl = SPI0::regs().ctrl().read().bits();
        let original_spi1_ctrl = SPI1::regs().ctrl().read().bits();
        let original_din_mode = SPI0::regs().din_mode().read().bits();
        let original_din_num = SPI0::regs().din_num().read().bits();
        let original_spi0_timing = SPI0::regs().timing_cali().read().bits();
        let original_spi1_timing = SPI1::regs().timing_cali().read().bits();

        let previous_mstatus: usize;
        unsafe {
            core::arch::asm!(
                "csrrc {previous}, mstatus, {mie}",
                previous = out(reg) previous_mstatus,
                mie = in(reg) 8usize,
            );
        }
        let previous_predictor: usize;
        unsafe {
            core::arch::asm!(
                "csrrc {previous}, 0x7c1, {mask}",
                previous = out(reg) previous_predictor,
                mask = in(reg) (1usize << 4) | (1 << 5) | (1 << 12),
            );
        }
        let mut autoload = unsafe { Cache_Suspend_L1_CORE0_ICache() };

        set_flash_clock_120mhz();
        let mut pass_mask = 0u16;
        let mut candidate = 0usize;
        while candidate < FLASH_120MHZ_TIMING.len() {
            let (din_mode, din_num, extra_dummy) = FLASH_120MHZ_TIMING[candidate];
            set_flash_timing(din_mode, din_num, extra_dummy, false);

            let mut received = [0u32; 32];
            let read_ok = unsafe { esp_rom_spiflash_read(0, received.as_mut_ptr(), 128) } == 0;
            let mut matches = read_ok;
            let mut word = 0usize;
            while matches && word < reference.len() {
                matches = received[word] == reference[word];
                word += 1;
            }
            if matches {
                pass_mask |= 1 << candidate;
            }
            candidate += 1;
        }

        // Give every candidate a distinct, previously untouched XIP page.
        // This avoids false passes from cache hits without requiring a cache
        // invalidation primitive in the still-incomplete S31 ROM bindings.
        let mut cache_pass_mask = 0u16;
        candidate = 0;
        while candidate < FLASH_120MHZ_TIMING.len() {
            if pass_mask & (1 << candidate) != 0 {
                let (din_mode, din_num, extra_dummy) = FLASH_120MHZ_TIMING[candidate];
                set_flash_timing(din_mode, din_num, extra_dummy, true);
                unsafe { Cache_Resume_L1_CORE0_ICache(autoload) };

                sample = page_count - FLASH_120MHZ_TIMING.len() + candidate;
                let mapped =
                    (xip.virtual_start + sample * FLASH_TUNING_XIP_PAGE_BYTES) as *const u32;
                let mut cache_matches = true;
                let mut word = 0usize;
                while cache_matches && word < FLASH_TUNING_XIP_PAGE_WORDS {
                    cache_matches = unsafe { mapped.add(word).read_volatile() }
                        == mapped_reference[sample * FLASH_TUNING_XIP_PAGE_WORDS + word];
                    word += 1;
                }
                if cache_matches {
                    cache_pass_mask |= 1 << candidate;
                }
                autoload = unsafe { Cache_Suspend_L1_CORE0_ICache() };
            }
            candidate += 1;
        }
        let usable_mask = pass_mask & cache_pass_mask;

        // Require three consecutive candidates; never silently use a default.
        let mut result = if let Some(best) = tuning::select_window(usable_mask) {
            let (din_mode, din_num, extra_dummy) = FLASH_120MHZ_TIMING[best];
            set_flash_timing(din_mode, din_num, extra_dummy, true);
            Ok(FlashTiming {
                config_index: best as u8,
                direct_pass_mask: pass_mask,
                cache_pass_mask,
            })
        } else {
            unsafe {
                HP_SYS_CLKRST::regs()
                    .flash_ctrl0()
                    .write(|w| w.bits(original_flash_ctrl));
                SPI0::regs().clock().write(|w| w.bits(original_spi0_clock));
                SPI1::regs().clock().write(|w| w.bits(original_spi1_clock));
                SPI0::regs().ctrl().write(|w| w.bits(original_spi0_ctrl));
                SPI1::regs().ctrl().write(|w| w.bits(original_spi1_ctrl));
                SPI0::regs().din_mode().write(|w| w.bits(original_din_mode));
                SPI0::regs().din_num().write(|w| w.bits(original_din_num));
                SPI1::regs()
                    .timing_cali()
                    .write(|w| w.bits(original_spi1_timing));
                SPI0::regs()
                    .timing_cali()
                    .write(|w| w.bits(original_spi0_timing).update().set_bit());
            }
            Err(FlashError::TimingTuningFailed)
        };

        unsafe { Cache_Resume_L1_CORE0_ICache(autoload) };

        let mut cache_matches = result.is_ok();
        sample = 0;
        while cache_matches && sample < page_count - FLASH_120MHZ_TIMING.len() {
            let mapped = (xip.virtual_start + sample * FLASH_TUNING_XIP_PAGE_BYTES) as *const u32;
            let mut word = 0usize;
            while cache_matches && word < FLASH_TUNING_XIP_PAGE_WORDS {
                cache_matches = unsafe { mapped.add(word).read_volatile() }
                    == mapped_reference[sample * FLASH_TUNING_XIP_PAGE_WORDS + word];
                word += 1;
            }
            sample += 1;
        }
        if result.is_ok() && !cache_matches {
            let rollback_autoload = unsafe { Cache_Suspend_L1_CORE0_ICache() };
            unsafe {
                HP_SYS_CLKRST::regs()
                    .flash_ctrl0()
                    .write(|w| w.bits(original_flash_ctrl));
                SPI0::regs().clock().write(|w| w.bits(original_spi0_clock));
                SPI1::regs().clock().write(|w| w.bits(original_spi1_clock));
                SPI0::regs().ctrl().write(|w| w.bits(original_spi0_ctrl));
                SPI1::regs().ctrl().write(|w| w.bits(original_spi1_ctrl));
                SPI0::regs().din_mode().write(|w| w.bits(original_din_mode));
                SPI0::regs().din_num().write(|w| w.bits(original_din_num));
                SPI1::regs()
                    .timing_cali()
                    .write(|w| w.bits(original_spi1_timing));
                SPI0::regs()
                    .timing_cali()
                    .write(|w| w.bits(original_spi0_timing).update().set_bit());
                Cache_Resume_L1_CORE0_ICache(rollback_autoload);
            }
            result = Err(FlashError::TimingTuningFailed);
        }

        unsafe {
            core::arch::asm!(
                "csrs 0x7c1, {bits}",
                bits = in(reg) previous_predictor & ((1usize << 4) | (1 << 5) | (1 << 12)),
            );
            if previous_mstatus & 8 != 0 {
                core::arch::asm!("csrs mstatus, {mie}", mie = in(reg) 8usize);
            }
        }
        result
    }
}

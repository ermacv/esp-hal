//! External SPI flash support.
//!
//! ESP32-S31 programs downloaded directly to RAM do not pass through the
//! normal second-stage bootloader, so the ROM flash driver has to be attached
//! and configured before ROM read/write functions can be used.

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

/// Error returned while attaching the ROM flash driver.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum FlashError {
    /// The ROM did not detect a plausible JEDEC identifier.
    InvalidJedecId,
    /// The ROM could not configure the flash controller.
    ConfigurationFailed,
    /// A ROM flash read failed.
    ReadFailed,
    /// The requested address range is outside the detected flash device.
    InvalidAddress,
    /// The ROM could not clear the flash protection bits before a mutation.
    UnlockFailed,
    /// A ROM flash sector erase failed.
    EraseFailed,
    /// A ROM flash program operation failed.
    WriteFailed,
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

unsafe extern "C" {
    fn esp_rom_spiflash_attach(ishspi: u32, legacy: bool);
    fn esp_rom_spi_flash_update_id();
    fn esp_rom_spiflash_config_clk(divider: u8, spi_num: u8) -> i32;
    fn esp_rom_spiflash_config_readmode(mode: i32) -> i32;
    fn esp_rom_spiflash_config_param(
        device_id: u32,
        chip_size: u32,
        block_size: u32,
        sector_size: u32,
        page_size: u32,
        status_mask: u32,
    ) -> i32;
    fn esp_rom_spiflash_read(address: u32, destination: *mut u32, length: u32) -> i32;
    fn esp_rom_spiflash_unlock() -> i32;
    fn esp_rom_spiflash_erase_sector(sector_number: u32) -> i32;
    fn esp_rom_spiflash_write(address: u32, source: *const u32, length: u32) -> i32;
    fn Cache_Suspend_L1_CORE0_ICache() -> u32;
    fn Cache_Resume_L1_CORE0_ICache(autoload: u32);
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn set_flash_clock_120mhz() {
    unsafe {
        HP_SYS_CLKRST::regs().flash_ctrl0().modify(|_, w| {
            w.reg_flash_sys_clk_en().set_bit();
            w.reg_flash_pll_clk_en().set_bit();
            w.reg_flash_core_clk_en().set_bit();
            w.reg_flash_clk_src_sel().bits(1);
            // SPLL is 480 MHz: divider field stores divisor minus one.
            w.reg_flash_core_clk_div_num().bits(3)
        });
        SPI0::regs().clock().write(|w| w.bits(1 << 31));
        SPI1::regs().clock().write(|w| w.bits(1 << 31));
        SPI0::regs().ctrl().modify(|_, w| w.fdummy_rin().set_bit());
        SPI1::regs().ctrl().modify(|_, w| w.fdummy_rin().set_bit());
    }
}

#[inline(always)]
#[unsafe(link_section = ".rwtext")]
fn set_flash_timing(din_mode: u8, din_num: u8, extra_dummy: u8, apply_to_cache: bool) {
    let mut mode_bits = 0u32;
    let mut num_bits = 0u32;
    let mut signal = 0;
    while signal < 9 {
        mode_bits |= (din_mode as u32) << (signal * 3);
        num_bits |= (din_num as u32) << (signal * 2);
        signal += 1;
    }

    unsafe {
        let spi0 = SPI0::regs();
        spi0.din_mode().write(|w| w.bits(mode_bits));
        spi0.din_num().write(|w| w.bits(num_bits));
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
    /// Erase size of the ROM SPI flash driver.
    pub const SECTOR_SIZE: u32 = 0x1000;
    /// Program and read alignment of the ROM SPI flash driver.
    pub const WORD_SIZE: u32 = 4;

    /// Uses the flash configuration established by the normal bootloader.
    pub fn from_bootloader(peri: FLASH<'static>) -> Result<Self, FlashError> {
        let jedec_id = unsafe {
            let descriptor = (0x2f07_ffe0 as *const *const u32).read_volatile();
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

    /// Attaches and configures the ROM flash driver after a RAM download.
    ///
    /// This is not needed during a normal flash boot: the ROM and second-stage
    /// bootloader have already initialized the controller in that case.
    pub fn new_ram_download(peri: FLASH<'static>) -> Result<Self, FlashError> {
        unsafe {
            esp_rom_spiflash_attach(0, false);
            if esp_rom_spiflash_config_readmode(2) != 0 || esp_rom_spiflash_config_clk(2, 0) != 0 {
                return Err(FlashError::ConfigurationFailed);
            }
            esp_rom_spi_flash_update_id();

            // Pointer to the legacy flash descriptor exported by ESP32-S31 ROM.
            let descriptor = (0x2f07_ffe0 as *const *const u32).read_volatile();
            let jedec_id = descriptor.read_volatile();
            let capacity_bits = jedec_id & 0xff;
            if jedec_id == 0 || jedec_id == 0x00ff_ffff || capacity_bits > 31 {
                return Err(FlashError::InvalidJedecId);
            }
            let size = 1u32 << capacity_bits;
            if esp_rom_spiflash_config_param(jedec_id, size, 0x1_0000, 0x1000, 0x100, 0xffff) != 0 {
                return Err(FlashError::ConfigurationFailed);
            }

            Ok(Self {
                _peri: peri,
                info: FlashInfo { jedec_id, size },
            })
        }
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
    /// a valid, mapped flash region containing at least 15 distinct 4 KiB pages.
    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub unsafe fn tune_120mhz(&mut self, xip: FlashXipRegion) -> Result<FlashTiming, FlashError> {
        let page_count = ((xip.size.saturating_sub(128)) / 0x1000 + 1).min(31);
        if page_count < FLASH_120MHZ_TIMING.len() + 3 {
            return Err(FlashError::TimingTuningFailed);
        }
        let mut reference = [0u32; 32];
        if unsafe { esp_rom_spiflash_read(0, reference.as_mut_ptr(), 128) } != 0 {
            return Err(FlashError::ReadFailed);
        }
        let mut mapped_reference = [0u32; 31 * 32];
        let mut sample = 0usize;
        while sample < page_count {
            let physical = xip.physical_start + sample as u32 * 0x1000;
            if unsafe {
                esp_rom_spiflash_read(
                    physical,
                    mapped_reference.as_mut_ptr().add(sample * 32),
                    128,
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
                let mapped = (xip.virtual_start + sample * 0x1000) as *const u32;
                let mut cache_matches = true;
                let mut word = 0usize;
                while cache_matches && word < 32 {
                    cache_matches = unsafe { mapped.add(word).read_volatile() }
                        == mapped_reference[sample * 32 + word];
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

        let mut longest_start = 0usize;
        let mut longest_len = 0usize;
        let mut current_start = 0usize;
        let mut current_len = 0usize;
        candidate = 0;
        while candidate < FLASH_120MHZ_TIMING.len() {
            if usable_mask & (1 << candidate) != 0 {
                if current_len == 0 {
                    current_start = candidate;
                }
                current_len += 1;
                if current_len > longest_len {
                    longest_start = current_start;
                    longest_len = current_len;
                }
            } else {
                current_len = 0;
            }
            candidate += 1;
        }

        // ESP-IDF requires at least three consecutive passing entries. The
        // P4 table's documented default is index 2, but do not silently fall
        // back: this port should only enable 120 MHz after a real stable window.
        let mut result = if longest_len >= 3 {
            let best = longest_start + longest_len / 2;
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
                    .write(|w| w.bits(original_spi0_timing | (1 << 6)));
            }
            Err(FlashError::TimingTuningFailed)
        };

        unsafe { Cache_Resume_L1_CORE0_ICache(autoload) };

        let mut cache_matches = result.is_ok();
        sample = 0;
        while cache_matches && sample < page_count - FLASH_120MHZ_TIMING.len() {
            let mapped = (xip.virtual_start + sample * 0x1000) as *const u32;
            let mut word = 0usize;
            while cache_matches && word < 32 {
                cache_matches = unsafe { mapped.add(word).read_volatile() }
                    == mapped_reference[sample * 32 + word];
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
                    .write(|w| w.bits(original_spi0_timing | (1 << 6)));
                Cache_Resume_L1_CORE0_ICache(rollback_autoload);
            }
            result = Err(FlashError::TimingTuningFailed);
        }

        unsafe {
            if previous_mstatus & 8 != 0 {
                core::arch::asm!("csrs mstatus, {mie}", mie = in(reg) 8usize);
            }
        }
        result
    }

    /// Reads aligned words using the ROM flash driver.
    pub fn read_words(&self, address: u32, destination: &mut [u32]) -> Result<(), FlashError> {
        let length = destination
            .len()
            .checked_mul(size_of::<u32>())
            .and_then(|length| u32::try_from(length).ok())
            .ok_or(FlashError::ReadFailed)?;
        if !address.is_multiple_of(Self::WORD_SIZE)
            || address
                .checked_add(length)
                .is_none_or(|end| end > self.info.size)
        {
            return Err(FlashError::InvalidAddress);
        }
        if unsafe { esp_rom_spiflash_read(address, destination.as_mut_ptr(), length) } == 0 {
            Ok(())
        } else {
            Err(FlashError::ReadFailed)
        }
    }

    /// Erases one 4 KiB sector before the second CPU and interrupt-driven
    /// flash users have been started.
    ///
    /// # Safety
    ///
    /// CPU1 must be stopped, no DMA operation may access flash, and the caller
    /// must ensure that no interrupt handler needs flash. This function masks
    /// CPU0 interrupts and runs from internal RAM while the ROM mutates flash.
    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub unsafe fn erase_sector_boot(&mut self, address: u32) -> Result<(), FlashError> {
        if !address.is_multiple_of(Self::SECTOR_SIZE)
            || address
                .checked_add(Self::SECTOR_SIZE)
                .is_none_or(|end| end > self.info.size)
        {
            return Err(FlashError::InvalidAddress);
        }

        let previous_mstatus: usize;
        unsafe {
            core::arch::asm!(
                "csrrc {previous}, mstatus, {mie}",
                previous = out(reg) previous_mstatus,
                mie = in(reg) 8usize,
            );
        }
        let unlock = unsafe { esp_rom_spiflash_unlock() };
        let erased = if unlock == 0 {
            unsafe { esp_rom_spiflash_erase_sector(address / Self::SECTOR_SIZE) }
        } else {
            -1
        };
        unsafe {
            if previous_mstatus & 8 != 0 {
                core::arch::asm!("csrs mstatus, {mie}", mie = in(reg) 8usize);
            }
        }
        if unlock != 0 {
            Err(FlashError::UnlockFailed)
        } else if erased != 0 {
            Err(FlashError::EraseFailed)
        } else {
            Ok(())
        }
    }

    /// Programs aligned words before the second CPU and interrupt-driven flash
    /// users have been started. The destination must already be erased.
    ///
    /// # Safety
    ///
    /// The same boot-time exclusion requirements as
    /// [`Flash::erase_sector_boot`] apply.
    #[inline(never)]
    #[unsafe(link_section = ".rwtext")]
    pub unsafe fn write_words_boot(
        &mut self,
        address: u32,
        source: &[u32],
    ) -> Result<(), FlashError> {
        let length = source
            .len()
            .checked_mul(size_of::<u32>())
            .and_then(|length| u32::try_from(length).ok())
            .ok_or(FlashError::InvalidAddress)?;
        if !address.is_multiple_of(Self::WORD_SIZE)
            || address
                .checked_add(length)
                .is_none_or(|end| end > self.info.size)
        {
            return Err(FlashError::InvalidAddress);
        }

        let previous_mstatus: usize;
        unsafe {
            core::arch::asm!(
                "csrrc {previous}, mstatus, {mie}",
                previous = out(reg) previous_mstatus,
                mie = in(reg) 8usize,
            );
        }
        let unlock = unsafe { esp_rom_spiflash_unlock() };
        let written = if unlock == 0 {
            unsafe { esp_rom_spiflash_write(address, source.as_ptr(), length) }
        } else {
            -1
        };
        unsafe {
            if previous_mstatus & 8 != 0 {
                core::arch::asm!("csrs mstatus, {mie}", mie = in(reg) 8usize);
            }
        }
        if unlock != 0 {
            Err(FlashError::UnlockFailed)
        } else if written == 0 {
            Ok(())
        } else {
            Err(FlashError::WriteFailed)
        }
    }
}

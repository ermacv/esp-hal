//! External SPI flash support.
//!
//! ESP32-S31 programs downloaded directly to RAM do not pass through the
//! normal second-stage bootloader, so the ROM flash driver has to be attached
//! and configured before ROM read/write functions can be used.

use crate::peripherals::FLASH;

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
}

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

    /// Reads aligned words using the ROM flash driver.
    pub fn read_words(&self, address: u32, destination: &mut [u32]) -> Result<(), FlashError> {
        let length = destination
            .len()
            .checked_mul(size_of::<u32>())
            .and_then(|length| u32::try_from(length).ok())
            .ok_or(FlashError::ReadFailed)?;
        if unsafe { esp_rom_spiflash_read(address, destination.as_mut_ptr(), length) } == 0 {
            Ok(())
        } else {
            Err(FlashError::ReadFailed)
        }
    }
}

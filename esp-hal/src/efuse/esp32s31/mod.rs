//! Minimal eFuse access for ESP32-S31 bring-up.

use crate::peripherals::EFUSE;

mod fields;
pub(crate) use fields::*;

/// Get status of SPI boot encryption.
#[instability::unstable]
pub fn flash_encryption() -> bool {
    false
}

/// Get the multiplier for the RWDT stage timeout.
#[instability::unstable]
pub fn rwdt_multiplier() -> u8 {
    0
}

pub(crate) fn major_chip_version() -> u8 {
    0
}

pub(crate) fn minor_chip_version() -> u8 {
    0
}

#[derive(Debug, Clone, Copy, strum::FromRepr)]
#[repr(u32)]
pub(crate) enum EfuseBlock {
    Block0,
    Block1,
    Block2,
    Block3,
    Block4,
    Block5,
    Block6,
    Block7,
    Block8,
    Block9,
}

impl EfuseBlock {
    pub(crate) fn address(self) -> *const u32 {
        let efuse = EFUSE::regs();
        match self {
            Self::Block0 => efuse.rd_wr_dis().as_ptr(),
            Self::Block1 => efuse.rd_mac_sys0().as_ptr(),
            Self::Block2 => efuse.rd_sys_part1_data(0).as_ptr(),
            Self::Block3 => efuse.rd_usr_data(0).as_ptr(),
            Self::Block4 => efuse.rd_key0_data(0).as_ptr(),
            Self::Block5 => efuse.rd_key1_data(0).as_ptr(),
            Self::Block6 => efuse.rd_key2_data(0).as_ptr(),
            Self::Block7 => efuse.rd_key3_data(0).as_ptr(),
            Self::Block8 => efuse.rd_key4_data(0).as_ptr(),
            Self::Block9 => efuse.rd_sys_part2_data0().as_ptr(),
        }
    }
}

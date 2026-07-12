//! Low-level ESP32-S31 Gigabit Ethernet MAC support.
//!
//! This module owns the SoC-specific clock, access-control, RGMII and MDIO
//! setup. PHY policy and board reset wiring intentionally remain outside HAL.

use crate::peripherals::{ETH, HP_SYS_CLKRST};

const GMAC_BASE: usize = 0x2035_0000;
const CNNT_SYS_BASE: usize = 0x2035_9000;
const IO_MUX_BASE: usize = 0x2058_2000;
const CNNT_IO_MUX_BASE: usize = 0x2058_8000;

/// Ethernet link speed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    /// 10 Mbit/s.
    Mbps10,
    /// 100 Mbit/s.
    Mbps100,
    /// 1000 Mbit/s.
    Mbps1000,
}

/// Errors produced by the low-level GMAC block.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// The MDIO controller did not complete an operation.
    MdioTimeout,
}

/// Owned handle to the ESP32-S31 GMAC peripheral.
pub struct Gmac {
    _peri: ETH<'static>,
    phy_address: u8,
}

impl Gmac {
    /// Enables the GMAC clock/reset path and opens DMA access to internal RAM.
    pub fn new(peri: ETH<'static>, phy_address: u8) -> Self {
        // Bare-metal startup leaves non-CPU bus masters behind APM filters.
        unsafe {
            (0x2070_6cbc as *mut u32).write_volatile(0);
            (0x2050_44c4 as *mut u32).write_volatile(0);
            (0x2050_48c4 as *mut u32).write_volatile(0);
        }
        HP_SYS_CLKRST::regs()
            .emac_ctrl0()
            .modify(|_, w| w.reg_emac_sys_clk_en().set_bit());
        unsafe {
            modify(CNNT_SYS_BASE + 0x3c, 0, 1 << 1);
            modify(CNNT_SYS_BASE + 0x3c, 1 << 1, 0);
        }
        Self {
            _peri: peri,
            phy_address,
        }
    }

    /// Returns the Synopsys GMAC version register.
    pub fn version(&self) -> u32 {
        unsafe { ((GMAC_BASE + 0x20) as *const u32).read_volatile() }
    }

    /// Configures the Function-CoreBoard RGMII set-1 data plane (GPIO8..19).
    pub fn configure_rgmii_set1(&self) {
        unsafe {
            for pin in 8..=19 {
                let input_enable = if pin >= 14 { 1 << 9 } else { 0 };
                modify(
                    IO_MUX_BASE + pin * 4,
                    (0x7 << 12) | (1 << 9) | (1 << 8) | (1 << 7),
                    (2 << 12) | input_enable,
                );
            }
            modify(CNNT_IO_MUX_BASE + 0x3f4, 0, 1 << 1);
            modify(CNNT_SYS_BASE + 0x40, 0x0000_ff07, (3 << 8) | (1 << 2));
            modify(CNNT_SYS_BASE + 0x44, (1 << 1) | (1 << 2), 0);
            modify(CNNT_SYS_BASE + 0x48, (1 << 0) | (1 << 1), 1 << 2);
            modify(CNNT_SYS_BASE + 0x4c, 0x0f, (1 << 0) | (1 << 2) | (1 << 3));
            modify(CNNT_SYS_BASE + 0x50, 0x0f, 1 << 3);
            modify(CNNT_SYS_BASE + 0x60, 0x7 << 2, 1 << 2);
        }
    }

    /// Selects the RGMII reference clock for the negotiated link speed.
    pub fn set_speed(&self, speed: Speed) {
        let divider = match speed {
            Speed::Mbps10 => 199,
            Speed::Mbps100 => 19,
            Speed::Mbps1000 => 3,
        };
        unsafe { modify(CNNT_SYS_BASE + 0x40, 0xff << 8, divider << 8) };
    }

    /// Reads one IEEE 802.3 Clause-22 PHY register.
    pub fn mdio_read(&self, register: u8) -> Result<u16, Error> {
        let address = (GMAC_BASE + 0x10) as *mut u32;
        let data = (GMAC_BASE + 0x14) as *const u32;
        unsafe {
            address.write_volatile(
                (u32::from(self.phy_address) << 11) | (u32::from(register) << 6) | (5 << 2) | 1,
            );
            for _ in 0..1_000_000 {
                if address.read_volatile() & 1 == 0 {
                    return Ok(data.read_volatile() as u16);
                }
            }
        }
        Err(Error::MdioTimeout)
    }

    /// Writes one IEEE 802.3 Clause-22 PHY register.
    pub fn mdio_write(&self, register: u8, value: u16) -> Result<(), Error> {
        let address = (GMAC_BASE + 0x10) as *mut u32;
        let data = (GMAC_BASE + 0x14) as *mut u32;
        unsafe {
            data.write_volatile(u32::from(value));
            address.write_volatile(
                (u32::from(self.phy_address) << 11)
                    | (u32::from(register) << 6)
                    | (5 << 2)
                    | (1 << 1)
                    | 1,
            );
            for _ in 0..1_000_000 {
                if address.read_volatile() & 1 == 0 {
                    return Ok(());
                }
            }
        }
        Err(Error::MdioTimeout)
    }
}

unsafe fn modify(address: usize, clear: u32, set: u32) {
    let register = address as *mut u32;
    unsafe { register.write_volatile((register.read_volatile() & !clear) | set) };
}

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
    /// The DMA controller did not leave software reset.
    DmaResetTimeout,
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

    /// Resets the DMA engine and returns its hardware feature register.
    pub fn reset_dma(&self) -> Result<u32, Error> {
        let bus_mode = (GMAC_BASE + 0x1000) as *mut u32;
        unsafe {
            bus_mode.write_volatile(bus_mode.read_volatile() | 1);
            for _ in 0..1_000_000 {
                if bus_mode.read_volatile() & 1 == 0 {
                    return Ok(((GMAC_BASE + 0x1058) as *const u32).read_volatile());
                }
            }
        }
        Err(Error::DmaResetTimeout)
    }

    /// Configures enhanced chained descriptors and their list heads.
    pub fn configure_descriptor_lists(&self, rx_base: u32, tx_base: u32) {
        unsafe {
            ((GMAC_BASE + 0x1000) as *mut u32)
                .write_volatile((1 << 7) | (16 << 8) | (1 << 25) | (1 << 26));
            ((GMAC_BASE + 0x100c) as *mut u32).write_volatile(rx_base);
            ((GMAC_BASE + 0x1010) as *mut u32).write_volatile(tx_base);
        }
    }

    /// Starts MAC RX/TX and DMA for the negotiated mode.
    pub fn start(&self, speed: Speed, full_duplex: bool, rx_base: u32) {
        self.set_speed(speed);
        unsafe {
            let mac_config = GMAC_BASE as *mut u32;
            let mut config = mac_config.read_volatile() & !((1 << 15) | (1 << 14) | (1 << 11));
            match speed {
                Speed::Mbps1000 => {}
                Speed::Mbps100 => config |= (1 << 15) | (1 << 14),
                Speed::Mbps10 => config |= 1 << 15,
            }
            if full_duplex {
                config |= 1 << 11;
            }
            ((GMAC_BASE + 0x04) as *mut u32)
                .write_volatile(((GMAC_BASE + 0x04) as *const u32).read_volatile() | 1);
            mac_config.write_volatile(config | (1 << 2) | (1 << 3));
            let operation = (GMAC_BASE + 0x1018) as *mut u32;
            operation.write_volatile(operation.read_volatile() | (1 << 1) | (1 << 13));
            ((GMAC_BASE + 0x100c) as *mut u32).write_volatile(rx_base);
            ((GMAC_BASE + 0x1014) as *mut u32).write_volatile(1 << 7);
            self.demand_rx_poll();
        }
    }

    /// Stops MAC RX/TX and both DMA directions.
    pub fn stop(&self) {
        unsafe {
            let operation = (GMAC_BASE + 0x1018) as *mut u32;
            operation.write_volatile(operation.read_volatile() & !((1 << 1) | (1 << 13)));
            let config = GMAC_BASE as *mut u32;
            config.write_volatile(config.read_volatile() & !((1 << 2) | (1 << 3)));
        }
    }

    /// Wakes a suspended RX DMA engine.
    pub fn demand_rx_poll(&self) {
        unsafe { ((GMAC_BASE + 0x1008) as *mut u32).write_volatile(1) }
    }

    /// Wakes a suspended TX DMA engine.
    pub fn demand_tx_poll(&self) {
        unsafe { ((GMAC_BASE + 0x1004) as *mut u32).write_volatile(1) }
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

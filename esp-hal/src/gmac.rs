//! Low-level ESP32-S31 Gigabit Ethernet MAC support.
//!
//! This module owns the SoC-specific clock, access-control, RGMII and MDIO
//! setup. PHY policy and board reset wiring intentionally remain outside HAL.

use core::{
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
    task::Context,
};

use embassy_net_driver_02::{Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken};

use crate::{
    asynch::AtomicWaker,
    peripherals::{ETH, HP_SYS_CLKRST},
};

const GMAC_BASE: usize = 0x2035_0000;
const CNNT_SYS_BASE: usize = 0x2035_9000;
const IO_MUX_BASE: usize = 0x2058_2000;
const CNNT_IO_MUX_BASE: usize = 0x2058_8000;
const BUFFER_SIZE: usize = 1536;

#[repr(C, align(32))]
struct Descriptor([u32; 8]);

#[repr(C, align(64))]
struct Buffer([u8; BUFFER_SIZE]);

/// Statically allocated enhanced descriptor rings and packet buffers.
pub struct DmaStorage<const RX: usize, const TX: usize> {
    rx_descriptors: [Descriptor; RX],
    tx_descriptors: [Descriptor; TX],
    rx_buffers: [Buffer; RX],
    tx_buffers: [Buffer; TX],
}

impl<const RX: usize, const TX: usize> DmaStorage<RX, TX> {
    /// Creates zero-initialized DMA storage suitable for a `static` cell.
    pub const fn new() -> Self {
        Self {
            rx_descriptors: [const { Descriptor([0; 8]) }; RX],
            tx_descriptors: [const { Descriptor([0; 8]) }; TX],
            rx_buffers: [const { Buffer([0; BUFFER_SIZE]) }; RX],
            tx_buffers: [const { Buffer([0; BUFFER_SIZE]) }; TX],
        }
    }
}

impl<const RX: usize, const TX: usize> Default for DmaStorage<RX, TX> {
    fn default() -> Self {
        Self::new()
    }
}

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
    /// The detected PHY identifier is not a YT8531.
    UnexpectedPhyId,
    /// A PHY configuration value is outside its valid range.
    InvalidPhyConfiguration,
}

/// Negotiated Ethernet mode.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LinkMode {
    /// Negotiated line speed.
    pub speed: Speed,
    /// Whether full-duplex operation was negotiated.
    pub full_duplex: bool,
}

/// State transition produced by [`Gmac::poll_link`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LinkEvent {
    /// Link state did not change.
    Unchanged,
    /// Link became active with the negotiated mode.
    Up(LinkMode),
    /// Link was lost and DMA was stopped.
    Down,
}

/// Motorcomm YT8531 Gigabit Ethernet PHY policy.
pub struct Yt8531 {
    rgmii_tx_delay: u8,
}

impl Yt8531 {
    /// Creates a PHY policy with TX delay in 150 ps steps.
    pub fn new(rgmii_tx_delay: u8) -> Result<Self, Error> {
        if rgmii_tx_delay > 15 {
            return Err(Error::InvalidPhyConfiguration);
        }
        Ok(Self { rgmii_tx_delay })
    }

    /// Verifies and configures the PHY after board-level reset and pin routing.
    pub fn initialize(&self, gmac: &Gmac) -> Result<(), Error> {
        if (gmac.mdio_read(2)?, gmac.mdio_read(3)?) != (0x4f51, 0xe91b) {
            return Err(Error::UnexpectedPhyId);
        }
        let rgmii = self.read_extended(gmac, 0xa003)?;
        self.write_extended(
            gmac,
            0xa003,
            (rgmii & !0x000f) | u16::from(self.rgmii_tx_delay),
        )
    }

    /// Returns the current link status.
    pub fn link_up(&self, gmac: &Gmac) -> Result<bool, Error> {
        let _ = gmac.mdio_read(1)?;
        Ok(gmac.mdio_read(1)? & (1 << 2) != 0)
    }

    /// Resolves the negotiated speed and duplex mode.
    pub fn link_mode(&self, gmac: &Gmac) -> Result<Option<LinkMode>, Error> {
        if !self.link_up(gmac)? {
            return Ok(None);
        }
        let bmcr = gmac.mdio_read(0)?;
        if bmcr & (1 << 12) == 0 {
            let speed = if bmcr & (1 << 6) != 0 {
                Speed::Mbps1000
            } else if bmcr & (1 << 13) != 0 {
                Speed::Mbps100
            } else {
                Speed::Mbps10
            };
            return Ok(Some(LinkMode {
                speed,
                full_duplex: bmcr & (1 << 8) != 0,
            }));
        }
        let gigabit_common = gmac.mdio_read(9)? & (gmac.mdio_read(10)? >> 2);
        if gigabit_common & (1 << 9) != 0 {
            return Ok(Some(LinkMode {
                speed: Speed::Mbps1000,
                full_duplex: true,
            }));
        }
        if gigabit_common & (1 << 8) != 0 {
            return Ok(Some(LinkMode {
                speed: Speed::Mbps1000,
                full_duplex: false,
            }));
        }
        let common = gmac.mdio_read(4)? & gmac.mdio_read(5)?;
        let mode = if common & (1 << 8) != 0 {
            LinkMode {
                speed: Speed::Mbps100,
                full_duplex: true,
            }
        } else if common & (1 << 7) != 0 {
            LinkMode {
                speed: Speed::Mbps100,
                full_duplex: false,
            }
        } else if common & (1 << 6) != 0 {
            LinkMode {
                speed: Speed::Mbps10,
                full_duplex: true,
            }
        } else if common & (1 << 5) != 0 {
            LinkMode {
                speed: Speed::Mbps10,
                full_duplex: false,
            }
        } else {
            return Ok(None);
        };
        Ok(Some(mode))
    }

    fn read_extended(&self, gmac: &Gmac, register: u16) -> Result<u16, Error> {
        gmac.mdio_write(0x1e, register)?;
        gmac.mdio_read(0x1f)
    }

    fn write_extended(&self, gmac: &Gmac, register: u16, value: u16) -> Result<(), Error> {
        gmac.mdio_write(0x1e, register)?;
        gmac.mdio_write(0x1f, value)
    }
}

/// Owned handle to the ESP32-S31 GMAC peripheral.
pub struct Gmac {
    _peri: ETH<'static>,
    phy_address: u8,
    started: AtomicBool,
    ring_generation: AtomicU32,
    net_waker: AtomicWaker,
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
            started: AtomicBool::new(false),
            ring_generation: AtomicU32::new(0),
            net_waker: AtomicWaker::new(),
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

    /// Initializes enhanced chained RX and TX descriptor rings.
    pub fn configure_rings<const RX: usize, const TX: usize>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
    ) {
        assert!(RX > 0 && TX > 0);
        for index in 0..RX {
            let next = (index + 1) % RX;
            storage.rx_descriptors[index].0 = [0; 8];
            storage.rx_descriptors[index].0[0] = 1 << 31;
            storage.rx_descriptors[index].0[1] = (BUFFER_SIZE as u32 & 0x1fff) | (1 << 14);
            storage.rx_descriptors[index].0[2] = storage.rx_buffers[index].0.as_mut_ptr() as u32;
            storage.rx_descriptors[index].0[3] =
                core::ptr::addr_of!(storage.rx_descriptors[next]) as u32;
        }
        for index in 0..TX {
            let next = (index + 1) % TX;
            storage.tx_descriptors[index].0 = [0; 8];
            storage.tx_descriptors[index].0[0] = 1 << 20;
            storage.tx_descriptors[index].0[2] = storage.tx_buffers[index].0.as_mut_ptr() as u32;
            storage.tx_descriptors[index].0[3] =
                core::ptr::addr_of!(storage.tx_descriptors[next]) as u32;
        }
        self.configure_descriptor_lists(
            storage.rx_descriptors.as_ptr() as u32,
            storage.tx_descriptors.as_ptr() as u32,
        );
        self.ring_generation.fetch_add(1, Ordering::AcqRel);
        self.net_waker.wake();
    }

    /// Returns the RX descriptor-list base address.
    pub fn rx_base<const RX: usize, const TX: usize>(&self, storage: &DmaStorage<RX, TX>) -> u32 {
        storage.rx_descriptors.as_ptr() as u32
    }

    /// Returns whether an RX descriptor contains one complete valid frame.
    pub fn rx_ready<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
        index: usize,
    ) -> bool {
        let status = unsafe {
            storage.rx_descriptors[index % RX]
                .0
                .as_ptr()
                .read_volatile()
        };
        status & (1 << 31) == 0
            && status & (1 << 15) == 0
            && status & (1 << 9) != 0
            && status & (1 << 8) != 0
    }

    /// Returns whether a TX descriptor is owned by the CPU.
    pub fn tx_ready<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
        index: usize,
    ) -> bool {
        unsafe {
            storage.tx_descriptors[index % TX]
                .0
                .as_ptr()
                .read_volatile()
                & (1 << 31)
                == 0
        }
    }

    /// Gives a received frame to `consume`, then returns its descriptor to DMA.
    pub fn consume_rx<const RX: usize, const TX: usize, R>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
        index: usize,
        consume: impl FnOnce(&mut [u8]) -> R,
    ) -> R {
        let index = index % RX;
        let status = unsafe { storage.rx_descriptors[index].0.as_ptr().read_volatile() };
        let length = (((status >> 16) & 0x3fff) as usize)
            .saturating_sub(4)
            .min(BUFFER_SIZE);
        let result = consume(&mut storage.rx_buffers[index].0[..length]);
        unsafe {
            storage.rx_descriptors[index]
                .0
                .as_mut_ptr()
                .write_volatile(1 << 31);
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
        self.demand_rx_poll();
        result
    }

    /// Gives a TX buffer to `fill`, then hands its descriptor to DMA.
    pub fn consume_tx<const RX: usize, const TX: usize, R>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
        index: usize,
        length: usize,
        fill: impl FnOnce(&mut [u8]) -> R,
    ) -> R {
        let index = index % TX;
        let length = length.min(1514);
        let result = fill(&mut storage.tx_buffers[index].0[..length]);
        unsafe {
            storage.tx_descriptors[index]
                .0
                .as_mut_ptr()
                .add(1)
                .write_volatile(length as u32);
            storage.tx_descriptors[index]
                .0
                .as_mut_ptr()
                .write_volatile((1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 20));
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
        self.demand_tx_poll();
        result
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
        self.started.store(true, Ordering::Release);
        self.net_waker.wake();
    }

    /// Stops MAC RX/TX and both DMA directions.
    pub fn stop(&self) {
        unsafe {
            let operation = (GMAC_BASE + 0x1018) as *mut u32;
            operation.write_volatile(operation.read_volatile() & !((1 << 1) | (1 << 13)));
            let config = GMAC_BASE as *mut u32;
            config.write_volatile(config.read_volatile() & !((1 << 2) | (1 << 3)));
        }
        self.started.store(false, Ordering::Release);
        self.net_waker.wake();
    }

    /// Wakes a suspended RX DMA engine.
    pub fn demand_rx_poll(&self) {
        unsafe { ((GMAC_BASE + 0x1008) as *mut u32).write_volatile(1) }
    }

    /// Wakes a suspended TX DMA engine.
    pub fn demand_tx_poll(&self) {
        unsafe { ((GMAC_BASE + 0x1004) as *mut u32).write_volatile(1) }
    }

    /// Wakes the network executor after polling hardware without interrupts.
    pub fn wake_network(&self) {
        self.net_waker.wake();
    }

    /// Polls the PHY and applies link transitions to MAC and DMA state.
    pub fn poll_link<const RX: usize, const TX: usize>(
        &self,
        phy: &Yt8531,
        storage: &mut DmaStorage<RX, TX>,
    ) -> Result<LinkEvent, Error> {
        match phy.link_mode(self)? {
            Some(mode) if !self.started.load(Ordering::Acquire) => {
                self.configure_rings(storage);
                self.start(mode.speed, mode.full_duplex, self.rx_base(storage));
                Ok(LinkEvent::Up(mode))
            }
            Some(_) => Ok(LinkEvent::Unchanged),
            None if self.started.load(Ordering::Acquire) => {
                self.stop();
                Ok(LinkEvent::Down)
            }
            None => Ok(LinkEvent::Unchanged),
        }
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

/// `embassy-net-driver` adapter for ESP32-S31 GMAC DMA storage.
pub struct NetDriver<const RX: usize, const TX: usize> {
    gmac: &'static Gmac,
    storage: *mut DmaStorage<RX, TX>,
    mac: [u8; 6],
    rx_index: usize,
    tx_index: usize,
    generation: u32,
}

unsafe impl<const RX: usize, const TX: usize> Send for NetDriver<RX, TX> {}

impl<const RX: usize, const TX: usize> NetDriver<RX, TX> {
    /// Creates an Embassy adapter over statically allocated DMA storage.
    pub fn new(
        gmac: &'static Gmac,
        storage: &'static mut DmaStorage<RX, TX>,
        mac: [u8; 6],
    ) -> Self {
        Self {
            gmac,
            storage,
            mac,
            rx_index: 0,
            tx_index: 0,
            generation: gmac.ring_generation.load(Ordering::Acquire),
        }
    }

    fn synchronize(&mut self) {
        let generation = self.gmac.ring_generation.load(Ordering::Acquire);
        if generation != self.generation {
            self.rx_index = 0;
            self.tx_index = 0;
            self.generation = generation;
        }
    }
}

/// GMAC receive token.
pub struct GmacRxToken<'a, const RX: usize, const TX: usize> {
    gmac: &'static Gmac,
    storage: *mut DmaStorage<RX, TX>,
    index: &'a mut usize,
}

/// GMAC transmit token.
pub struct GmacTxToken<'a, const RX: usize, const TX: usize> {
    gmac: &'static Gmac,
    storage: *mut DmaStorage<RX, TX>,
    index: &'a mut usize,
}

impl<const RX: usize, const TX: usize> RxToken for GmacRxToken<'_, RX, TX> {
    fn consume<R, F>(self, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let index = *self.index;
        let result = unsafe { self.gmac.consume_rx(&mut *self.storage, index, f) };
        *self.index = (index + 1) % RX;
        result
    }
}

impl<const RX: usize, const TX: usize> TxToken for GmacTxToken<'_, RX, TX> {
    fn consume<R, F>(self, length: usize, f: F) -> R
    where
        F: FnOnce(&mut [u8]) -> R,
    {
        let index = *self.index;
        let result = unsafe { self.gmac.consume_tx(&mut *self.storage, index, length, f) };
        *self.index = (index + 1) % TX;
        result
    }
}

impl<const RX: usize, const TX: usize> Driver for NetDriver<RX, TX> {
    type RxToken<'a>
        = GmacRxToken<'a, RX, TX>
    where
        Self: 'a;
    type TxToken<'a>
        = GmacTxToken<'a, RX, TX>
    where
        Self: 'a;

    fn receive(&mut self, cx: &mut Context<'_>) -> Option<(Self::RxToken<'_>, Self::TxToken<'_>)> {
        self.synchronize();
        self.gmac.net_waker.register(cx.waker());
        let storage = unsafe { &*self.storage };
        if !self.gmac.started.load(Ordering::Acquire)
            || !self.gmac.rx_ready(storage, self.rx_index)
            || !self.gmac.tx_ready(storage, self.tx_index)
        {
            return None;
        }
        Some((
            GmacRxToken {
                gmac: self.gmac,
                storage: self.storage,
                index: &mut self.rx_index,
            },
            GmacTxToken {
                gmac: self.gmac,
                storage: self.storage,
                index: &mut self.tx_index,
            },
        ))
    }

    fn transmit(&mut self, cx: &mut Context<'_>) -> Option<Self::TxToken<'_>> {
        self.synchronize();
        self.gmac.net_waker.register(cx.waker());
        let ready = self.gmac.started.load(Ordering::Acquire)
            && unsafe { self.gmac.tx_ready(&*self.storage, self.tx_index) };
        ready.then_some(GmacTxToken {
            gmac: self.gmac,
            storage: self.storage,
            index: &mut self.tx_index,
        })
    }

    fn link_state(&mut self, cx: &mut Context<'_>) -> LinkState {
        self.gmac.net_waker.register(cx.waker());
        if self.gmac.started.load(Ordering::Acquire) {
            LinkState::Up
        } else {
            LinkState::Down
        }
    }

    fn capabilities(&self) -> Capabilities {
        let mut capabilities = Capabilities::default();
        capabilities.max_transmission_unit = 1514;
        capabilities
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac)
    }
}

unsafe fn modify(address: usize, clear: u32, set: u32) {
    let register = address as *mut u32;
    unsafe { register.write_volatile((register.read_volatile() & !clear) | set) };
}

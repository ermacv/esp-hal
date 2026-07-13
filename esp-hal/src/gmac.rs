//! Low-level ESP32-S31 Gigabit Ethernet MAC support.
//!
//! This module owns the SoC-specific clock, access-control, RGMII and MDIO
//! setup. PHY policy and board reset wiring intentionally remain outside HAL.

use core::{
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering},
    task::Context,
};

use embassy_net_driver_02::{Capabilities, Driver, HardwareAddress, LinkState, RxToken, TxToken};

use crate::{
    asynch::AtomicWaker,
    gpio::{
        DriveStrength, InputConfig, InputSignal, OutputConfig, OutputSignal,
        interconnect::{self, PeripheralInput, PeripheralOutput},
    },
    interrupt,
    peripherals::{ETH, HP_SYS_CLKRST, Interrupt},
    system::Cpu,
};

#[inline]
fn gmac_regs() -> &'static crate::pac::gmac::RegisterBlock {
    unsafe { &*crate::pac::GMAC::ptr() }
}

#[inline]
fn io_mux_regs() -> &'static crate::pac::io_mux::RegisterBlock {
    unsafe { &*crate::pac::IO_MUX::ptr() }
}

#[inline]
fn cnnt_io_mux_regs() -> &'static crate::pac::cnnt_io_mux::RegisterBlock {
    unsafe { &*crate::pac::CNNT_IO_MUX::ptr() }
}

#[inline]
fn cnnt_sys_regs() -> &'static crate::pac::cnnt_sys::RegisterBlock {
    unsafe { &*crate::pac::CNNT_SYS::ptr() }
}

const BUFFER_SIZE: usize = 1536;
static INTERRUPT_GMAC: AtomicPtr<Gmac> = AtomicPtr::new(ptr::null_mut());

#[crate::handler]
fn gmac_interrupt() {
    let regs = gmac_regs();
    let status = regs.register5_statusregister().read();
    regs.register5_statusregister()
        .write(|w| unsafe { w.bits(status.bits()) });
    if status.gli().bit_is_set() {
        regs.register54_sgmii_rgmii_smiicontrolandstatusregister()
            .read();
    }
    let gmac = INTERRUPT_GMAC.load(Ordering::Acquire);
    if !gmac.is_null() {
        unsafe { (*gmac).net_waker.wake() };
    }
}

#[repr(C, align(32))]
struct Descriptor([u32; 8]);

impl Descriptor {
    #[inline]
    fn read_word(&self, index: usize) -> u32 {
        unsafe { self.0.as_ptr().add(index).read_volatile() }
    }

    #[inline]
    fn write_word(&mut self, index: usize, value: u32) {
        unsafe { self.0.as_mut_ptr().add(index).write_volatile(value) };
    }
}

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

/// Raw GMAC state intended for bring-up diagnostics.
#[derive(Clone, Copy, Debug)]
pub struct DiagnosticSnapshot {
    /// Configured Clause-22 PHY address.
    pub phy_address: u8,
    /// DMA status register.
    pub dma_status: u32,
    /// DMA operation mode register.
    pub dma_operation_mode: u32,
    /// Configured TX descriptor-list base.
    pub tx_descriptor_base: u32,
    /// Current TX descriptor address observed by DMA.
    pub current_tx_descriptor: u32,
    /// First RX descriptor status word.
    pub rx_descriptor: u32,
    /// First TX descriptor status word.
    pub tx_descriptor: u32,
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

    /// Captures the minimum raw state needed to diagnose early GMAC bring-up.
    pub fn diagnostic_snapshot<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
    ) -> DiagnosticSnapshot {
        DiagnosticSnapshot {
            phy_address: self.phy_address,
            dma_status: gmac_regs().register5_statusregister().read().bits(),
            dma_operation_mode: gmac_regs().register6_operationmoderegister().read().bits(),
            tx_descriptor_base: gmac_regs()
                .register4_transmitdescriptorlistaddressregister()
                .read()
                .bits(),
            current_tx_descriptor: gmac_regs()
                .register18_currenthosttransmitdescriptorregister()
                .read()
                .bits(),
            rx_descriptor: storage.rx_descriptors[0].read_word(0),
            tx_descriptor: storage.tx_descriptors[0].read_word(0),
        }
    }
    /// Enables the GMAC clock/reset path.
    pub fn new(peri: ETH<'static>, phy_address: u8) -> Self {
        interrupt::disable(Cpu::current(), Interrupt::SBD);
        // Mask every MAC-level source. In particular, an RGMII in-band link
        // transition otherwise keeps the shared SBD line asserted.
        gmac_regs()
            .register15_interruptmaskregister()
            .write(|w| unsafe { w.bits(u32::MAX) });
        gmac_regs()
            .register54_sgmii_rgmii_smiicontrolandstatusregister()
            .read();
        HP_SYS_CLKRST::regs()
            .emac_ctrl0()
            .modify(|_, w| w.reg_emac_sys_clk_en().set_bit());
        cnnt_sys_regs()
            .hp_emac_ctrl()
            .modify(|_, w| w.emac_rst_en().set_bit());
        cnnt_sys_regs()
            .hp_emac_ctrl()
            .modify(|_, w| w.emac_rst_en().clear_bit());
        Self {
            _peri: peri,
            phy_address,
            started: AtomicBool::new(false),
            ring_generation: AtomicU32::new(0),
            net_waker: AtomicWaker::new(),
        }
    }

    /// Routes the GMAC management interface through the GPIO matrix.
    pub fn configure_mdio_pins<'d>(
        &self,
        mdc: impl PeripheralOutput<'d>,
        mdio: impl PeripheralInput<'d> + PeripheralOutput<'d>,
    ) {
        let mdc: interconnect::OutputSignal<'_> = mdc.into();
        mdc.apply_output_config(&OutputConfig::default().with_drive_strength(DriveStrength::_20mA));
        OutputSignal::EMAC_MDC.connect_to(&mdc);

        let mdio: interconnect::OutputSignal<'_> = mdio.into();
        mdio.apply_output_config(
            &OutputConfig::default().with_drive_strength(DriveStrength::_20mA),
        );
        mdio.apply_input_config(&InputConfig::default());
        InputSignal::EMAC_MDI.connect_to(&mdio);
        OutputSignal::EMAC_MDO.connect_to(&mdio);
        mdio.set_input_enable(true);
    }

    /// Returns the Synopsys GMAC version register.
    pub fn version(&self) -> u32 {
        gmac_regs().register8_versionregister().read().bits()
    }

    /// Configures the Function-CoreBoard RGMII set-1 data plane (GPIO8..19).
    pub fn configure_rgmii_set1(&self) {
        for pin in 8..=19 {
            let input_enable = if pin >= 14 { 1 << 9 } else { 0 };
            io_mux_regs().gpio(pin).modify(|r, w| unsafe {
                w.bits(
                    (r.bits() & !((0x7 << 12) | (1 << 9) | (1 << 8) | (1 << 7)))
                        | (2 << 12)
                        | input_enable,
                )
            });
        }
        cnnt_io_mux_regs()
            .ctrl()
            .modify(|_, w| w.gmac_pad_pin_ctrl_ded_sel().set_bit());
        let regs = cnnt_sys_regs();
        regs.hp_emac_ref_ctrl().modify(|_, w| unsafe {
            w.emac_ref_clk_sel()
                .bits(0)
                .emac_ref_clk_en()
                .set_bit()
                .emac_ref_clk_div_num()
                .bits(3)
        });
        regs.hp_emac_rmii_pad_ctrl().modify(|_, w| {
            w.emac_rmii_pad_clk_en()
                .clear_bit()
                .emac_rmii_pad_clk_inv_en()
                .clear_bit()
        });
        regs.hp_emac_rmii_ctrl().modify(|_, w| {
            w.emac_rmii_clk_sel()
                .clear_bit()
                .emac_rmii_clk_en()
                .clear_bit()
                .emac_rmii_pad_out_clk_en()
                .set_bit()
        });
        regs.hp_emac_rx_ctrl().modify(|_, w| {
            w.emac_rx_pad_clk_en()
                .set_bit()
                .emac_rx_pad_clk_inv_en()
                .clear_bit()
                .emac_rx_clk_sel()
                .set_bit()
                .emac_rx_180_clk_en()
                .set_bit()
        });
        regs.hp_emac_tx_ctrl().modify(|_, w| {
            w.emac_tx_pad_clk_en()
                .clear_bit()
                .emac_tx_pad_clk_inv_en()
                .clear_bit()
                .emac_tx_clk_sel()
                .clear_bit()
                .emac_tx_180_clk_en()
                .set_bit()
        });
        regs.gmac_ctrl0()
            .modify(|_, w| unsafe { w.phy_intf_sel().bits(1) });
    }

    /// Selects the RGMII reference clock for the negotiated link speed.
    pub fn set_speed(&self, speed: Speed) {
        let divider = match speed {
            Speed::Mbps10 => 199,
            Speed::Mbps100 => 19,
            Speed::Mbps1000 => 3,
        };
        cnnt_sys_regs()
            .hp_emac_ref_ctrl()
            .modify(|_, w| unsafe { w.emac_ref_clk_div_num().bits(divider) });
    }

    /// Resets the DMA engine and returns its hardware feature register.
    pub fn reset_dma(&self) -> Result<u32, Error> {
        let regs = gmac_regs();
        regs.register7_interruptenableregister().reset();
        let pending = regs.register5_statusregister().read().bits();
        regs.register5_statusregister()
            .write(|w| unsafe { w.bits(pending) });
        regs.register0_busmoderegister()
            .modify(|_, w| w.swr().set_bit());
        for _ in 0..1_000_000 {
            if regs.register0_busmoderegister().read().swr().bit_is_clear() {
                return Ok(regs.register22_hwfeatureregister().read().bits());
            }
        }
        Err(Error::DmaResetTimeout)
    }

    /// Configures enhanced chained descriptors and their list heads.
    pub fn configure_descriptor_lists(&self, rx_base: u32, tx_base: u32) {
        let regs = gmac_regs();
        regs.register0_busmoderegister().write(|w| unsafe {
            w.atds()
                .set_bit()
                .pbl()
                .bits(16)
                .aal()
                .set_bit()
                .mb()
                .set_bit()
        });
        regs.register3_receivedescriptorlistaddressregister()
            .write(|w| unsafe { w.rdesla().bits(rx_base) });
        regs.register4_transmitdescriptorlistaddressregister()
            .write(|w| unsafe { w.tdesla().bits(tx_base) });
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
        let status = storage.rx_descriptors[index % RX].read_word(0);
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
        storage.tx_descriptors[index % TX].read_word(0) & (1 << 31) == 0
    }

    /// Gives a received frame to `consume`, then returns its descriptor to DMA.
    pub fn consume_rx<const RX: usize, const TX: usize, R>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
        index: usize,
        consume: impl FnOnce(&mut [u8]) -> R,
    ) -> R {
        let index = index % RX;
        let status = storage.rx_descriptors[index].read_word(0);
        let length = (((status >> 16) & 0x3fff) as usize)
            .saturating_sub(4)
            .min(BUFFER_SIZE);
        let result = consume(&mut storage.rx_buffers[index].0[..length]);
        storage.rx_descriptors[index].write_word(0, 1 << 31);
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
        storage.tx_descriptors[index].write_word(1, length as u32);
        storage.tx_descriptors[index]
            .write_word(0, (1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 20));
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
        self.demand_tx_poll();
        result
    }

    /// Starts MAC RX/TX and DMA for the negotiated mode.
    pub fn start(&self, speed: Speed, full_duplex: bool, rx_base: u32) {
        self.set_speed(speed);
        let regs = gmac_regs();
        regs.register1_macframefilter()
            .modify(|_, w| w.ra().set_bit());
        regs.register0_macconfigurationregister().modify(|_, w| {
            w.ps()
                .bit(speed != Speed::Mbps1000)
                .fes()
                .bit(speed == Speed::Mbps100)
                .dm()
                .bit(full_duplex)
                .re()
                .set_bit()
                .te()
                .set_bit()
        });
        regs.register6_operationmoderegister()
            .modify(|_, w| w.sr().set_bit().st().set_bit());
        regs.register3_receivedescriptorlistaddressregister()
            .write(|w| unsafe { w.rdesla().bits(rx_base) });
        regs.register5_statusregister().write(|w| w.ru().set_bit());
        self.demand_rx_poll();
        self.started.store(true, Ordering::Release);
        self.net_waker.wake();
        self.enable_interrupts();
    }

    /// Stops MAC RX/TX and both DMA directions.
    pub fn stop(&self) {
        let regs = gmac_regs();
        regs.register6_operationmoderegister()
            .modify(|_, w| w.sr().clear_bit().st().clear_bit());
        regs.register0_macconfigurationregister()
            .modify(|_, w| w.re().clear_bit().te().clear_bit());
        self.started.store(false, Ordering::Release);
        self.net_waker.wake();
    }

    /// Wakes a suspended RX DMA engine.
    pub fn demand_rx_poll(&self) {
        gmac_regs()
            .register2_receivepolldemandregister()
            .write(|w| unsafe { w.rpd().bits(1) });
    }

    /// Wakes a suspended TX DMA engine.
    pub fn demand_tx_poll(&self) {
        gmac_regs()
            .register1_transmitpolldemandregister()
            .write(|w| unsafe { w.tpd().bits(1) });
    }

    /// Wakes the network executor after polling hardware without interrupts.
    pub fn wake_network(&self) {
        self.net_waker.wake();
    }

    /// Enables RX/TX DMA interrupts and binds them to the Embassy waker.
    fn enable_interrupts(&self) {
        INTERRUPT_GMAC.store(self as *const Self as *mut Self, Ordering::Release);
        let regs = gmac_regs();
        let pending = regs.register5_statusregister().read().bits();
        regs.register5_statusregister()
            .write(|w| unsafe { w.bits(pending) });
        interrupt::bind_handler(Interrupt::SBD, gmac_interrupt);
        regs.register7_interruptenableregister()
            .write(|w| w.rie().set_bit().nie().set_bit());
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
        let regs = gmac_regs();
        // Keep this as one command-word store. An A/B test on S31 rev 0 showed
        // that the extra masking instructions emitted by the equivalent PAC
        // field-writer chain make the subsequent GMAC start path timing
        // sensitive and prevent RX/DHCP. Board initialization validates the
        // PHY address and the Clause-22 driver uses five-bit register numbers.
        regs.register4_gmiiaddressregister().write(|w| unsafe {
            w.bits(
                (u32::from(self.phy_address & 0x1f) << 11)
                    | (u32::from(register & 0x1f) << 6)
                    | (5 << 2)
                    | 1,
            )
        });
        for _ in 0..1_000_000 {
            if regs
                .register4_gmiiaddressregister()
                .read()
                .gb()
                .bit_is_clear()
            {
                return Ok(regs.register5_gmiidataregister().read().gd().bits());
            }
        }
        Err(Error::MdioTimeout)
    }

    /// Writes one IEEE 802.3 Clause-22 PHY register.
    pub fn mdio_write(&self, register: u8, value: u16) -> Result<(), Error> {
        let regs = gmac_regs();
        regs.register5_gmiidataregister()
            .write(|w| unsafe { w.gd().bits(value) });
        regs.register4_gmiiaddressregister().write(|w| unsafe {
            w.bits(
                (u32::from(self.phy_address & 0x1f) << 11)
                    | (u32::from(register & 0x1f) << 6)
                    | (5 << 2)
                    | (1 << 1)
                    | 1,
            )
        });
        for _ in 0..1_000_000 {
            if regs
                .register4_gmiiaddressregister()
                .read()
                .gb()
                .bit_is_clear()
            {
                return Ok(());
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

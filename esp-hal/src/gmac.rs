//! Low-level ESP32-S31 Gigabit Ethernet MAC support.
//!
//! This module owns the SoC-specific clock, access-control, RGMII and MDIO
//! setup. PHY policy and board reset wiring intentionally remain outside HAL.

use core::{
    mem::MaybeUninit,
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, Ordering},
    task::Context,
};

use embassy_net_driver_02::{
    Capabilities, Checksum, Driver, HardwareAddress, LinkState, RxToken, TxToken,
};

use crate::{
    asynch::AtomicWaker,
    gpio::{
        DriveStrength, InputConfig, InputSignal, OutputConfig, OutputSignal,
        interconnect::{self, PeripheralInput, PeripheralOutput},
    },
    interrupt,
    peripherals::{ETH, HP_SYS_CLKRST, Interrupt, LP_AON_CLK_RST},
    system::Cpu,
};

#[inline]
fn gmac_regs() -> &'static crate::pac::gmac::RegisterBlock {
    unsafe { &*crate::pac::GMAC::PTR }
}

#[inline]
fn io_mux_regs() -> &'static crate::pac::io_mux::RegisterBlock {
    unsafe { &*crate::pac::IO_MUX::PTR }
}

#[inline]
fn cnnt_io_mux_regs() -> &'static crate::pac::cnnt_io_mux::RegisterBlock {
    unsafe { &*crate::pac::CNNT_IO_MUX::PTR }
}

#[inline]
fn cnnt_sys_regs() -> &'static crate::pac::cnnt_sys::RegisterBlock {
    unsafe { &*crate::pac::CNNT_SYS::PTR }
}

const BUFFER_SIZE: usize = 1536;
const DMA_GUARD_WORD: u32 = 0xa55a_c33c;
const DMA_STATUS_TX_UNDERFLOW: u32 = 1 << 5;
const DMA_STATUS_FATAL_BUS_ERROR: u32 = 1 << 13;
const DMA_STATUS_ERROR_BITS: u32 = 0x7 << 23;
const DMA_DEFERRED_EVENT_MASK: u32 =
    DMA_STATUS_TX_UNDERFLOW | DMA_STATUS_FATAL_BUS_ERROR | DMA_STATUS_ERROR_BITS;
// ESP32-S31 ICM_SYS registers are not exposed by the current PAC yet.  These
// addresses and field positions come from the official ESP-IDF
// soc/icm_sys_reg.h and hal/axi_icm_ll.h definitions.
#[cfg(feature = "esp32s31-diagnostics")]
const ICM_MST_ARQOS_REG0: *mut u32 = 0x2051_0428 as *mut u32;
#[cfg(feature = "esp32s31-diagnostics")]
const ICM_MST_AWQOS_REG0: *mut u32 = 0x2051_0430 as *mut u32;
#[cfg(feature = "esp32s31-diagnostics")]
const ICM_GMAC_QOS_SHIFT: u32 = 16;
#[cfg(feature = "esp32s31-diagnostics")]
const ICM_GMAC_QOS_MASK: u32 = 0x0f << ICM_GMAC_QOS_SHIFT;
static INTERRUPT_GMAC: AtomicPtr<Gmac> = AtomicPtr::new(ptr::null_mut());

// The receive-complete interrupt is on the throughput-critical path.  Keep it
// in internal RAM, as ESP-IDF does for its EMAC ISR, so an instruction-cache
// miss on XIP flash cannot delay RX FIFO service or the network-task wakeup.
#[crate::ram]
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
        let gmac = unsafe { &*gmac };
        // Do not pay mask/rearm MMIO overhead for isolated frames such as TCP
        // ACKs. Once the runner is already draining a burst, suppress further
        // receive-complete interrupts until it reaches an empty descriptor.
        if status.ri().bit_is_set() {
            let now: u32;
            unsafe { core::arch::asm!("rdcycle {value}", value = out(reg) now) };
            let previous = gmac.last_rx_irq_cycle.swap(now, Ordering::AcqRel);
            let burst = now.wrapping_sub(previous) < 40_960; // 128 us at 320 MHz.
            if burst || gmac.rx_draining.load(Ordering::Acquire) {
                regs.register7_interruptenableregister()
                    .modify(|_, w| w.rie().clear_bit());
                gmac.rx_interrupt_masked.store(true, Ordering::Release);
            }
        }
        gmac.deferred_dma_events
            .fetch_or(status.bits() & DMA_DEFERRED_EVENT_MASK, Ordering::Release);
        gmac.net_waker.wake();
    }
}

#[repr(C, align(64))]
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

#[repr(C, align(64))]
struct DmaGuard([u32; 16]);

/// Statically allocated enhanced descriptor rings and packet buffers.
#[repr(C, align(64))]
pub struct DmaStorage<const RX: usize, const TX: usize> {
    leading_guard: DmaGuard,
    rx_descriptors: [Descriptor; RX],
    tx_descriptors: [Descriptor; TX],
    rx_buffers: [Buffer; RX],
    tx_buffers: [Buffer; TX],
    trailing_guard: DmaGuard,
}

impl<const RX: usize, const TX: usize> DmaStorage<RX, TX> {
    /// Creates zero-initialized DMA storage suitable for a `static` cell.
    pub const fn new() -> Self {
        Self {
            leading_guard: DmaGuard([DMA_GUARD_WORD; 16]),
            rx_descriptors: [const { Descriptor([0; 8]) }; RX],
            tx_descriptors: [const { Descriptor([0; 8]) }; TX],
            rx_buffers: [const { Buffer([0; BUFFER_SIZE]) }; RX],
            tx_buffers: [const { Buffer([0; BUFFER_SIZE]) }; TX],
            trailing_guard: DmaGuard([DMA_GUARD_WORD; 16]),
        }
    }

    /// Initializes DMA storage directly in its final static location.
    ///
    /// Unlike returning [`Self::new`] by value, this method guarantees that a
    /// storage-sized temporary is not placed on the caller's stack. This is
    /// important for configurations with many full-size Ethernet buffers.
    pub fn init_in_place(storage: &mut MaybeUninit<Self>) -> &mut Self {
        let storage = storage.as_mut_ptr();
        // SAFETY: `storage` is exclusively borrowed uninitialized memory with
        // the correct size and alignment. Every field consists only of u8/u32
        // arrays, so the all-zero bit pattern is valid. The two guards are
        // written before the initialized reference is created.
        unsafe {
            storage.write_bytes(0, 1);
            ptr::addr_of_mut!((*storage).leading_guard).write(DmaGuard([DMA_GUARD_WORD; 16]));
            ptr::addr_of_mut!((*storage).trailing_guard).write(DmaGuard([DMA_GUARD_WORD; 16]));
            &mut *storage
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
    rgmii_rx_delay: u8,
    rgmii_tx_delay: u8,
}

impl Yt8531 {
    const LINK_INTERRUPT_MASK: u16 = (1 << 14) | (1 << 13) | (1 << 11) | (1 << 10);

    /// Creates a PHY policy with TX delay in 150 ps steps.
    pub fn new(rgmii_tx_delay: u8) -> Result<Self, Error> {
        if rgmii_tx_delay > 15 {
            return Err(Error::InvalidPhyConfiguration);
        }
        Ok(Self {
            rgmii_rx_delay: 0,
            rgmii_tx_delay,
        })
    }

    /// Configures the RGMII RX clock delay in 150 ps steps.
    pub fn with_rx_delay(mut self, rgmii_rx_delay: u8) -> Result<Self, Error> {
        if rgmii_rx_delay > 15 {
            return Err(Error::InvalidPhyConfiguration);
        }
        self.rgmii_rx_delay = rgmii_rx_delay;
        Ok(self)
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
            (rgmii & !((0x000f << 10) | 0x000f))
                | (u16::from(self.rgmii_rx_delay) << 10)
                | u16::from(self.rgmii_tx_delay),
        )
    }

    /// Enables active-low INT_N events for link, speed, and duplex changes.
    pub fn enable_link_interrupts(&self, gmac: &Gmac) -> Result<(), Error> {
        self.acknowledge_link_interrupt(gmac)?;
        gmac.mdio_write(0x12, Self::LINK_INTERRUPT_MASK)
    }

    /// Reads and clears the PHY interrupt status register.
    pub fn acknowledge_link_interrupt(&self, gmac: &Gmac) -> Result<u16, Error> {
        gmac.mdio_read(0x13)
    }

    /// Enables or disables IEEE 802.3 PHY power-down mode.
    pub fn set_power_down(&self, gmac: &Gmac, power_down: bool) -> Result<(), Error> {
        let control = gmac.mdio_read(0)?;
        let control = if power_down {
            control | (1 << 11)
        } else {
            control & !(1 << 11)
        };
        gmac.mdio_write(0, control)
    }

    /// Enables or disables 1000BASE-T full-duplex advertisement.
    ///
    /// The new advertisement becomes active after link-down, PHY reset, or an
    /// explicit [`Self::restart_autonegotiation`] call.
    pub fn set_gigabit_advertisement(&self, gmac: &Gmac, enabled: bool) -> Result<(), Error> {
        let control = gmac.mdio_read(9)?;
        let control = if enabled {
            control | (1 << 9)
        } else {
            control & !(1 << 9)
        };
        gmac.mdio_write(9, control)
    }

    /// Restarts IEEE 802.3 auto-negotiation without resetting the PHY.
    pub fn restart_autonegotiation(&self, gmac: &Gmac) -> Result<(), Error> {
        let control = gmac.mdio_read(0)?;
        gmac.mdio_write(0, control | (1 << 12) | (1 << 9))
    }

    /// Returns whether smart-speed selected a lower speed after failed
    /// high-speed negotiation attempts.
    pub fn wirespeed_downgraded(&self, gmac: &Gmac) -> Result<bool, Error> {
        Ok(gmac.mdio_read(0x11)? & (1 << 5) != 0)
    }

    /// Enables or disables the IEEE 802.3 PHY loopback path.
    ///
    /// The negotiated speed and duplex fields are preserved. Loopback should
    /// only be enabled while operating full-duplex.
    pub fn set_loopback(&self, gmac: &Gmac, enabled: bool) -> Result<(), Error> {
        let control = gmac.mdio_read(0)?;
        let control = if enabled {
            control | (1 << 14)
        } else {
            control & !(1 << 14)
        };
        gmac.mdio_write(0, control)
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
    interrupt_driven: AtomicBool,
    rx_interrupt_masked: AtomicBool,
    rx_draining: AtomicBool,
    last_rx_irq_cycle: AtomicU32,
    ring_generation: AtomicU32,
    deferred_dma_events: AtomicU32,
    configured_dma_bus_mode: AtomicU32,
    rx_frame_count: AtomicU32,
    rx_error_drop_count: AtomicU32,
    last_rx_error_status: AtomicU32,
    dma_guard_status: AtomicU32,
    rx_ring_integrity_status: AtomicU32,
    tx_frame_count: AtomicU32,
    tx_payload_checksum_error_count: AtomicU32,
    tx_ip_header_error_count: AtomicU32,
    tx_error_summary_count: AtomicU32,
    last_tx_descriptor_status: AtomicU32,
    rx_dhcp_count: AtomicU32,
    tx_dhcp_count: AtomicU32,
    tx_underflow_count: AtomicU32,
    fatal_bus_error_count: AtomicU32,
    fatal_bus_recovery_failure_count: AtomicU32,
    last_fatal_bus_error: AtomicU32,
    net_waker: AtomicWaker,
}

impl Gmac {
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
            .modify(|_, w| w.emac_sys_clk_en().set_bit());
        cnnt_sys_regs()
            .sys_hp_emac_ctrl()
            .modify(|_, w| w.sys_emac_rst_en().set_bit());
        cnnt_sys_regs()
            .sys_hp_emac_ctrl()
            .modify(|_, w| w.sys_emac_rst_en().clear_bit());
        Self {
            _peri: peri,
            phy_address,
            started: AtomicBool::new(false),
            interrupt_driven: AtomicBool::new(true),
            rx_interrupt_masked: AtomicBool::new(false),
            rx_draining: AtomicBool::new(false),
            last_rx_irq_cycle: AtomicU32::new(0),
            ring_generation: AtomicU32::new(0),
            deferred_dma_events: AtomicU32::new(0),
            configured_dma_bus_mode: AtomicU32::new(0),
            rx_frame_count: AtomicU32::new(0),
            rx_error_drop_count: AtomicU32::new(0),
            last_rx_error_status: AtomicU32::new(0),
            dma_guard_status: AtomicU32::new(0),
            rx_ring_integrity_status: AtomicU32::new(0),
            tx_frame_count: AtomicU32::new(0),
            tx_payload_checksum_error_count: AtomicU32::new(0),
            tx_ip_header_error_count: AtomicU32::new(0),
            tx_error_summary_count: AtomicU32::new(0),
            last_tx_descriptor_status: AtomicU32::new(0),
            rx_dhcp_count: AtomicU32::new(0),
            tx_dhcp_count: AtomicU32::new(0),
            tx_underflow_count: AtomicU32::new(0),
            fatal_bus_error_count: AtomicU32::new(0),
            fatal_bus_recovery_failure_count: AtomicU32::new(0),
            last_fatal_bus_error: AtomicU32::new(0),
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
}

#[cfg(feature = "esp32s31-diagnostics")]
impl Gmac {
    /// Returns the Synopsys GMAC version register.
    pub fn version(&self) -> u32 {
        gmac_regs().register8_versionregister().read().bits()
    }

    /// Enables or disables the DWC GMAC-local MII/GMII loopback path.
    pub fn set_mac_loopback(&self, enabled: bool) {
        gmac_regs()
            .register0_macconfigurationregister()
            .modify(|_, w| w.lm().bit(enabled));
    }

    /// Returns the raw DWC DMA status register for diagnostics.
    pub fn dma_status(&self) -> u32 {
        gmac_regs().register5_statusregister().read().bits()
    }

    /// Returns the raw DMA bus-mode register for diagnostics.
    pub fn dma_bus_mode(&self) -> u32 {
        gmac_regs().register0_busmoderegister().read().bits()
    }

    /// Returns the BMR readback captured immediately after configuration.
    pub fn configured_dma_bus_mode(&self) -> u32 {
        self.configured_dma_bus_mode.load(Ordering::Relaxed)
    }

    /// Returns the raw DMA operation-mode register for diagnostics.
    pub fn dma_operation_mode(&self) -> u32 {
        gmac_regs().register6_operationmoderegister().read().bits()
    }

    /// Returns the raw GMAC AXI bus-mode register for diagnostics.
    pub fn axi_bus_mode(&self) -> u32 {
        gmac_regs().register10_axibusmoderegister().read().bits()
    }

    /// Returns the raw GMAC AXI status register for diagnostics.
    pub fn axi_status(&self) -> u32 {
        gmac_regs()
            .register11_ahboraxistatusregister()
            .read()
            .bits()
    }

    /// Returns GMAC read/write QoS nibbles as `ARQOS | AWQOS << 4`.
    pub fn interconnect_qos(&self) -> u32 {
        unsafe {
            ((ICM_MST_ARQOS_REG0.read_volatile() & ICM_GMAC_QOS_MASK) >> ICM_GMAC_QOS_SHIFT)
                | (((ICM_MST_AWQOS_REG0.read_volatile() & ICM_GMAC_QOS_MASK) >> ICM_GMAC_QOS_SHIFT)
                    << 4)
        }
    }

    /// Selects timer-driven polling instead of DMA interrupts.
    ///
    /// This is primarily useful for isolating interrupt-controller and waker
    /// faults during low-level GMAC bring-up.
    pub fn use_polling(&self) {
        self.interrupt_driven.store(false, Ordering::Release);
        gmac_regs().register7_interruptenableregister().reset();
        interrupt::disable(Cpu::current(), Interrupt::SBD);
    }

    /// Returns the number of RX frames handed to the network stack.
    pub fn rx_frame_count(&self) -> u32 {
        self.rx_frame_count.load(Ordering::Relaxed)
    }

    /// Returns the number of invalid RX descriptors recycled without delivery.
    pub fn rx_error_drop_count(&self) -> u32 {
        self.rx_error_drop_count.load(Ordering::Relaxed)
    }

    /// Returns the status word from the latest invalid RX descriptor.
    pub fn last_rx_error_status(&self) -> u32 {
        self.last_rx_error_status.load(Ordering::Relaxed)
    }

    /// Returns bit 0 for a damaged leading DMA guard and bit 1 for trailing.
    pub fn dma_guard_status(&self) -> u32 {
        self.dma_guard_status.load(Ordering::Relaxed)
    }

    /// Returns a bit mask of RX descriptors with corrupted immutable fields.
    pub fn rx_ring_integrity_status(&self) -> u32 {
        self.rx_ring_integrity_status.load(Ordering::Relaxed)
    }

    /// Returns the raw DWC DMA missed-frame and RX-buffer-overflow counter.
    pub fn missed_frame_and_buffer_overflow(&self) -> u32 {
        gmac_regs()
            .register8_missedframeandbufferoverflowcounterregister()
            .read()
            .bits()
    }

    /// Returns the number of TX frames handed to DMA.
    pub fn tx_frame_count(&self) -> u32 {
        self.tx_frame_count.load(Ordering::Relaxed)
    }

    /// Returns the number of completed TX descriptors with TDES0.PCE set.
    pub fn tx_payload_checksum_error_count(&self) -> u32 {
        self.tx_payload_checksum_error_count.load(Ordering::Relaxed)
    }

    /// Returns the number of completed TX descriptors with TDES0.IHE set.
    pub fn tx_ip_header_error_count(&self) -> u32 {
        self.tx_ip_header_error_count.load(Ordering::Relaxed)
    }

    /// Returns the number of completed TX descriptors with TDES0.ES set.
    pub fn tx_error_summary_count(&self) -> u32 {
        self.tx_error_summary_count.load(Ordering::Relaxed)
    }

    /// Returns the latest TX descriptor write-back status observed on reuse.
    pub fn last_tx_descriptor_status(&self) -> u32 {
        self.last_tx_descriptor_status.load(Ordering::Relaxed)
    }

    /// Returns the number of DHCP server-to-client frames received.
    pub fn rx_dhcp_count(&self) -> u32 {
        self.rx_dhcp_count.load(Ordering::Relaxed)
    }

    /// Returns the number of DHCP client-to-server frames transmitted.
    pub fn tx_dhcp_count(&self) -> u32 {
        self.tx_dhcp_count.load(Ordering::Relaxed)
    }

    /// Returns the number of TX FIFO underflows handled since construction.
    pub fn tx_underflow_count(&self) -> u32 {
        self.tx_underflow_count.load(Ordering::Relaxed)
    }

    /// Returns the number of fatal DMA bus errors observed by the ISR.
    pub fn fatal_bus_error_count(&self) -> u32 {
        self.fatal_bus_error_count.load(Ordering::Relaxed)
    }

    /// Returns the number of fatal DMA bus errors whose reset timed out.
    pub fn fatal_bus_recovery_failure_count(&self) -> u32 {
        self.fatal_bus_recovery_failure_count
            .load(Ordering::Relaxed)
    }

    /// Returns the DWC GMAC `EB[2:0]` code from the latest fatal bus error.
    pub fn last_fatal_bus_error(&self) -> Option<u8> {
        (self.fatal_bus_error_count() != 0)
            .then(|| self.last_fatal_bus_error.load(Ordering::Relaxed) as u8)
    }
}

impl Gmac {
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
        regs.sys_hp_emac_ref_ctrl().modify(|_, w| unsafe {
            w.sys_emac_ref_clk_sel()
                .bits(0)
                .sys_emac_ref_clk_en()
                .set_bit()
                .sys_emac_ref_clk_div_num()
                .bits(3)
        });
        regs.sys_hp_emac_rmii_pad_ctrl().modify(|_, w| {
            w.sys_emac_rmii_pad_clk_en()
                .clear_bit()
                .sys_emac_rmii_pad_clk_inv_en()
                .clear_bit()
        });
        regs.sys_hp_emac_rmii_ctrl().modify(|_, w| {
            w.sys_emac_rmii_clk_sel()
                .clear_bit()
                .sys_emac_rmii_clk_en()
                .clear_bit()
                .sys_emac_rmii_pad_out_clk_en()
                .set_bit()
        });
        regs.sys_hp_emac_rx_ctrl().modify(|_, w| {
            w.sys_emac_rx_pad_clk_en()
                .set_bit()
                .sys_emac_rx_pad_clk_inv_en()
                .clear_bit()
                .sys_emac_rx_clk_sel()
                .set_bit()
                .sys_emac_rx_180_clk_en()
                .set_bit()
        });
        regs.sys_hp_emac_tx_ctrl().modify(|_, w| {
            w.sys_emac_tx_pad_clk_en()
                .clear_bit()
                .sys_emac_tx_pad_clk_inv_en()
                .clear_bit()
                .sys_emac_tx_clk_sel()
                .clear_bit()
                .sys_emac_tx_180_clk_en()
                .set_bit()
        });
        regs.sys_gmac_ctrl0()
            .modify(|_, w| unsafe { w.sys_phy_intf_sel().bits(1) });
    }

    /// Selects the RGMII reference clock for the negotiated link speed.
    pub fn set_speed(&self, speed: Speed) {
        let target_khz = match speed {
            Speed::Mbps10 => 2_500,
            Speed::Mbps100 => 25_000,
            Speed::Mbps1000 => 125_000,
        };
        let fb_div = LP_AON_CLK_RST::regs()
            .mspi_div()
            .read()
            .mspi_fb_div()
            .bits() as u32;
        let mpll_khz: u32 = 40_000 * (fb_div + 1) / 2;
        assert!(mpll_khz.is_multiple_of(target_khz));
        // The register stores divisor - 1. Both the 400 MHz PSRAM clock plan
        // (10/100) and the 500 MHz plan (10/100/1000) are exact.
        let divider = (mpll_khz / target_khz - 1) as u8;
        cnnt_sys_regs()
            .sys_hp_emac_ref_ctrl()
            .modify(|_, w| unsafe { w.sys_emac_ref_clk_div_num().bits(divider) });
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
        // Use the literal register image from ESP-IDF's S31 defaults here so
        // the diagnostic readback also catches an SVD field-accessor error.
        // ATDS=1, PBL=16, AAL=1, MB=1; burst 32 is forbidden on S31.
        regs.register0_busmoderegister()
            .write(|w| unsafe { w.bits((1 << 7) | (16 << 8) | (1 << 25) | (1 << 26)) });
        self.configured_dma_bus_mode.store(
            regs.register0_busmoderegister().read().bits(),
            Ordering::Relaxed,
        );
        // Reset defaults allow two outstanding AXI reads/writes. Four is the
        // maximum implemented by this GMAC configuration and hides SRAM/ICM
        // arbitration latency without using the unsupported 32-beat burst.
        regs.register10_axibusmoderegister()
            .modify(|_, w| unsafe { w.rd_osr_lmt().bits(3).wr_osr_lmt().bits(3) });
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
        unsafe {
            crate::soc::cache_writeback_addr(
                storage as *mut DmaStorage<RX, TX> as u32,
                core::mem::size_of::<DmaStorage<RX, TX>>() as u32,
            )
        };
        core::sync::atomic::fence(Ordering::SeqCst);
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

    /// Returns the TX descriptor-list base address.
    pub fn tx_base<const RX: usize, const TX: usize>(&self, storage: &DmaStorage<RX, TX>) -> u32 {
        storage.tx_descriptors.as_ptr() as u32
    }

    /// Returns whether an RX descriptor contains one complete valid frame.
    pub fn rx_ready<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
        index: usize,
    ) -> bool {
        unsafe {
            crate::soc::cache_invalidate_addr(
                core::ptr::addr_of!(storage.rx_descriptors[index % RX]) as u32,
                core::mem::size_of::<Descriptor>() as u32,
            )
        };
        let status = storage.rx_descriptors[index % RX].read_word(0);
        status & (1 << 31) == 0
            && status & (1 << 15) == 0
            && status & (1 << 9) != 0
            && status & (1 << 8) != 0
    }

    /// Recycles invalid CPU-owned descriptors until the current RX slot is
    /// either a complete frame or is still owned by DMA.
    ///
    /// A descriptor carrying an error summary or an incomplete frame must not
    /// remain at the software ring head. DMA continues with later descriptors,
    /// but the network stack would otherwise keep polling the same invalid slot
    /// forever and eventually exhaust the complete RX ring.
    fn prepare_rx<const RX: usize, const TX: usize>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
        index: &mut usize,
    ) -> bool {
        // Guard validation is diagnostic work, not per-frame work. Checking
        // once per complete ring still detects corruption promptly without
        // scanning 32 volatile guard words for every received packet.
        if *index == 0 && self.audit_dma_guards(storage) != 0 {
            self.stop();
            return false;
        }
        for _ in 0..RX {
            let current = *index;
            let status = self.rx_descriptor_status(storage, current);
            if !self.rx_descriptor_integrity(storage, current) {
                self.rx_ring_integrity_status
                    .fetch_or(1 << current.min(31), Ordering::Relaxed);
                self.stop();
                return false;
            }
            if status & (1 << 31) != 0 {
                self.rx_draining.store(false, Ordering::Release);
                self.rearm_rx_interrupt();
                return false;
            }
            if status & (1 << 15) == 0 && status & (1 << 9) != 0 && status & (1 << 8) != 0 {
                self.rx_draining.store(true, Ordering::Release);
                return true;
            }

            storage.rx_descriptors[current].write_word(0, 1 << 31);
            unsafe {
                crate::soc::cache_writeback_addr(
                    core::ptr::addr_of!(storage.rx_descriptors[current]) as u32,
                    core::mem::size_of::<Descriptor>() as u32,
                )
            };
            core::sync::atomic::fence(Ordering::Release);
            self.last_rx_error_status.store(status, Ordering::Relaxed);
            self.rx_error_drop_count.fetch_add(1, Ordering::Relaxed);
            *index = (current + 1) % RX;
            self.demand_rx_poll();
        }
        false
    }

    fn audit_dma_guards<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
    ) -> u32 {
        unsafe {
            crate::soc::cache_invalidate_addr(
                core::ptr::addr_of!(storage.leading_guard) as u32,
                core::mem::size_of::<DmaGuard>() as u32,
            );
            crate::soc::cache_invalidate_addr(
                core::ptr::addr_of!(storage.trailing_guard) as u32,
                core::mem::size_of::<DmaGuard>() as u32,
            );
        }
        let leading_bad = storage
            .leading_guard
            .0
            .iter()
            .any(|word| unsafe { core::ptr::read_volatile(word) } != DMA_GUARD_WORD);
        let trailing_bad = storage
            .trailing_guard
            .0
            .iter()
            .any(|word| unsafe { core::ptr::read_volatile(word) } != DMA_GUARD_WORD);
        let status = u32::from(leading_bad) | (u32::from(trailing_bad) << 1);
        self.dma_guard_status.fetch_or(status, Ordering::Relaxed);
        status
    }

    fn rx_descriptor_integrity<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
        index: usize,
    ) -> bool {
        let index = index % RX;
        let next = (index + 1) % RX;
        let descriptor = &storage.rx_descriptors[index];
        descriptor.read_word(1) == (BUFFER_SIZE as u32 & 0x1fff) | (1 << 14)
            && descriptor.read_word(2) == storage.rx_buffers[index].0.as_ptr() as u32
            && descriptor.read_word(3) == core::ptr::addr_of!(storage.rx_descriptors[next]) as u32
    }

    /// Returns one raw RX descriptor status word after cache invalidation.
    pub fn rx_descriptor_status<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
        index: usize,
    ) -> u32 {
        let index = index % RX;
        unsafe {
            crate::soc::cache_invalidate_addr(
                core::ptr::addr_of!(storage.rx_descriptors[index]) as u32,
                core::mem::size_of::<Descriptor>() as u32,
            )
        };
        storage.rx_descriptors[index].read_word(0)
    }

    /// Returns whether a TX descriptor is owned by the CPU.
    pub fn tx_ready<const RX: usize, const TX: usize>(
        &self,
        storage: &DmaStorage<RX, TX>,
        index: usize,
    ) -> bool {
        unsafe {
            crate::soc::cache_invalidate_addr(
                core::ptr::addr_of!(storage.tx_descriptors[index % TX]) as u32,
                core::mem::size_of::<Descriptor>() as u32,
            )
        };
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
        unsafe {
            crate::soc::cache_invalidate_addr(
                storage.rx_buffers[index].0.as_ptr() as u32,
                length as u32,
            )
        };
        let frame = &mut storage.rx_buffers[index].0[..length];
        if is_dhcp_frame(frame, 67, 68) {
            self.rx_dhcp_count.fetch_add(1, Ordering::Relaxed);
        }
        let result = consume(frame);
        self.rx_frame_count.fetch_add(1, Ordering::Relaxed);
        storage.rx_descriptors[index].write_word(0, 1 << 31);
        unsafe {
            crate::soc::cache_writeback_addr(
                core::ptr::addr_of!(storage.rx_descriptors[index]) as u32,
                core::mem::size_of::<Descriptor>() as u32,
            )
        };
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
        // `tx_ready` invalidated this descriptor before handing out the token.
        // Preserve DMA write-back diagnostics before replacing TDES0 control
        // bits for the next frame.
        let completed_status = storage.tx_descriptors[index].read_word(0);
        self.last_tx_descriptor_status
            .store(completed_status, Ordering::Relaxed);
        if completed_status & (1 << 12) != 0 {
            self.tx_payload_checksum_error_count
                .fetch_add(1, Ordering::Relaxed);
        }
        if completed_status & (1 << 16) != 0 {
            self.tx_ip_header_error_count
                .fetch_add(1, Ordering::Relaxed);
        }
        if completed_status & (1 << 15) != 0 {
            self.tx_error_summary_count.fetch_add(1, Ordering::Relaxed);
        }
        let result = fill(&mut storage.tx_buffers[index].0[..length]);
        if is_dhcp_frame(&storage.tx_buffers[index].0[..length], 68, 67) {
            self.tx_dhcp_count.fetch_add(1, Ordering::Relaxed);
        }
        self.tx_frame_count.fetch_add(1, Ordering::Relaxed);
        storage.tx_descriptors[index].write_word(1, length as u32);
        storage.tx_descriptors[index]
            .write_word(0, (1 << 31) | (1 << 30) | (1 << 29) | (1 << 28) | (1 << 20));
        unsafe {
            crate::soc::cache_writeback_addr(
                storage.tx_buffers[index].0.as_ptr() as u32,
                length as u32,
            );
            crate::soc::cache_writeback_addr(
                core::ptr::addr_of!(storage.tx_descriptors[index]) as u32,
                core::mem::size_of::<Descriptor>() as u32,
            );
        }
        core::sync::atomic::fence(core::sync::atomic::Ordering::Release);
        self.demand_tx_poll();
        result
    }

    /// Starts MAC RX/TX and DMA for the negotiated mode.
    pub fn start(&self, speed: Speed, full_duplex: bool, rx_base: u32, tx_base: u32) {
        // Changing the RGMII reference divider can reset the DMA clock domain
        // on ESP32-S31. Configure every DMA bus parameter only after the final
        // link clock is selected; merely restoring the descriptor heads leaves
        // PBL at its reset value of one beat and cannot sustain gigabit RX.
        self.set_speed(speed);
        let regs = gmac_regs();
        self.configure_descriptor_lists(rx_base, tx_base);
        regs.register1_macframefilter()
            .modify(|_, w| w.ra().set_bit());
        regs.register0_macconfigurationregister().modify(|_, w| {
            w.ps()
                .bit(speed != Speed::Mbps1000)
                .fes()
                .bit(speed == Speed::Mbps100)
                .dm()
                .bit(full_duplex)
                .ipc()
                .set_bit()
                .re()
                .set_bit()
                .te()
                .set_bit()
        });
        regs.register6_operationmoderegister()
            .modify(|_, w| w.osf().set_bit().sr().set_bit().st().set_bit());
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
        unsafe {
            gmac_regs()
                .register2_receivepolldemandregister()
                .as_ptr()
                .write_volatile(1);
        }
    }

    /// Wakes a suspended TX DMA engine.
    pub fn demand_tx_poll(&self) {
        unsafe {
            gmac_regs()
                .register1_transmitpolldemandregister()
                .as_ptr()
                .write_volatile(1);
        }
    }

    /// Wakes the network executor after polling hardware without interrupts.
    pub fn wake_network(&self) {
        self.net_waker.wake();
    }

    /// Acknowledges DMA events and wakes the network task without an ISR.
    pub fn poll_dma_events(&self) {
        let regs = gmac_regs();
        let status = regs.register5_statusregister().read().bits();
        if status != 0 {
            regs.register5_statusregister()
                .write(|w| unsafe { w.bits(status) });
            self.deferred_dma_events
                .fetch_or(status & DMA_DEFERRED_EVENT_MASK, Ordering::Release);
        }
        self.net_waker.wake();
    }

    /// Enables RX/TX DMA interrupts and binds them to the Embassy waker.
    fn enable_interrupts(&self) {
        if !self.interrupt_driven.load(Ordering::Acquire) {
            gmac_regs().register7_interruptenableregister().reset();
            return;
        }
        INTERRUPT_GMAC.store(self as *const Self as *mut Self, Ordering::Release);
        self.rx_interrupt_masked.store(false, Ordering::Release);
        let regs = gmac_regs();
        let pending = regs.register5_statusregister().read().bits();
        regs.register5_statusregister()
            .write(|w| unsafe { w.bits(pending) });
        interrupt::bind_handler(Interrupt::SBD, gmac_interrupt);
        interrupt::enable(Interrupt::SBD, interrupt::Priority::min());
        regs.register7_interruptenableregister().write(|w| {
            w.tie()
                .set_bit()
                .une()
                .set_bit()
                .rie()
                .set_bit()
                .fbe()
                .set_bit()
                .aie()
                .set_bit()
                .nie()
                .set_bit()
        });
    }

    #[inline]
    fn rearm_rx_interrupt(&self) {
        if self.interrupt_driven.load(Ordering::Acquire)
            && self.rx_interrupt_masked.swap(false, Ordering::AcqRel)
        {
            gmac_regs()
                .register7_interruptenableregister()
                .modify(|_, w| w.rie().set_bit());
        }
    }

    /// Handles DMA events captured by the ISR in the network executor context.
    fn handle_deferred_dma_events<const RX: usize, const TX: usize>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
    ) {
        let events = self.deferred_dma_events.swap(0, Ordering::AcqRel);
        if events & DMA_STATUS_FATAL_BUS_ERROR != 0 {
            self.fatal_bus_error_count.fetch_add(1, Ordering::Relaxed);
            self.last_fatal_bus_error
                .store((events & DMA_STATUS_ERROR_BITS) >> 23, Ordering::Relaxed);
            if self.recover_dma(storage).is_err() {
                self.fatal_bus_recovery_failure_count
                    .fetch_add(1, Ordering::Relaxed);
            }
        } else if events & DMA_STATUS_TX_UNDERFLOW != 0 {
            self.recover_tx_underflow();
        }
    }

    /// Resets DMA, reconstructs both descriptor rings, and restores link mode.
    ///
    /// This is the hard-recovery path for a DWC GMAC fatal bus error. It must be
    /// called from the context that owns `storage`, never from the ISR.
    pub fn recover_dma<const RX: usize, const TX: usize>(
        &self,
        storage: &mut DmaStorage<RX, TX>,
    ) -> Result<(), Error> {
        let mac_configuration = gmac_regs().register0_macconfigurationregister().read();
        let speed = if mac_configuration.ps().bit_is_clear() {
            Speed::Mbps1000
        } else if mac_configuration.fes().bit_is_set() {
            Speed::Mbps100
        } else {
            Speed::Mbps10
        };
        let full_duplex = mac_configuration.dm().bit_is_set();

        self.stop();
        self.reset_dma()?;
        self.configure_rings(storage);
        self.start(
            speed,
            full_duplex,
            self.rx_base(storage),
            self.tx_base(storage),
        );
        Ok(())
    }

    /// Increases the TX FIFO threshold after an underflow, then resumes DMA.
    ///
    /// This mirrors the staged DWC GMAC recovery used by Linux stmmac. Register
    /// 6 may only change its threshold/store-and-forward fields while TX DMA is
    /// stopped, so none of this work is performed in the interrupt handler.
    fn recover_tx_underflow(&self) {
        let regs = gmac_regs();
        self.tx_underflow_count.fetch_add(1, Ordering::Relaxed);

        regs.register6_operationmoderegister()
            .modify(|_, w| w.st().clear_bit());
        let mut stopped = false;
        for _ in 0..10_000 {
            if regs.register5_statusregister().read().ts().bits() == 0 {
                stopped = true;
                break;
            }
        }

        if stopped {
            regs.register6_operationmoderegister().modify(|r, w| {
                let threshold = r.ttc().bits();
                if threshold < 3 {
                    unsafe { w.ttc().bits(threshold + 1) };
                } else {
                    w.tsf().set_bit();
                }
                w.osf().set_bit().st().set_bit()
            });
        } else {
            // Preserve service even if the hardware did not report Stopped in
            // the bounded interval. A later underflow retries the escalation.
            regs.register6_operationmoderegister()
                .modify(|_, w| w.st().set_bit());
        }
        self.demand_tx_poll();
    }

    /// Polls the PHY and applies link transitions to MAC and DMA state.
    pub fn poll_link<const RX: usize, const TX: usize>(
        &self,
        phy: &Yt8531,
        storage: &mut DmaStorage<RX, TX>,
    ) -> Result<LinkEvent, Error> {
        match phy.link_mode(self)? {
            Some(mode) if self.running_link_mode() != Some(mode) => {
                if self.started.load(Ordering::Acquire) {
                    self.stop();
                }
                self.configure_rings(storage);
                self.start(
                    mode.speed,
                    mode.full_duplex,
                    self.rx_base(storage),
                    self.tx_base(storage),
                );
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

    fn running_link_mode(&self) -> Option<LinkMode> {
        if !self.started.load(Ordering::Acquire) {
            return None;
        }
        let configuration = gmac_regs().register0_macconfigurationregister().read();
        let speed = if configuration.ps().bit_is_clear() {
            Speed::Mbps1000
        } else if configuration.fes().bit_is_set() {
            Speed::Mbps100
        } else {
            Speed::Mbps10
        };
        Some(LinkMode {
            speed,
            full_duplex: configuration.dm().bit_is_set(),
        })
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
            w.bits((u32::from(self.phy_address) << 11) | (u32::from(register) << 6) | (5 << 2) | 1)
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
                (u32::from(self.phy_address) << 11)
                    | (u32::from(register) << 6)
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

fn is_dhcp_frame(frame: &[u8], source_port: u16, destination_port: u16) -> bool {
    if frame.len() < 14 + 20 + 8 || frame[12..14] != [0x08, 0x00] {
        return false;
    }
    let ip_header_len = usize::from(frame[14] & 0x0f) * 4;
    if ip_header_len < 20 || frame[23] != 17 || frame.len() < 14 + ip_header_len + 8 {
        return false;
    }
    let udp = 14 + ip_header_len;
    u16::from_be_bytes([frame[udp], frame[udp + 1]]) == source_port
        && u16::from_be_bytes([frame[udp + 2], frame[udp + 3]]) == destination_port
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
        self.gmac
            .handle_deferred_dma_events(unsafe { &mut *self.storage });
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
        let storage = unsafe { &mut *self.storage };
        if !self.gmac.started.load(Ordering::Acquire)
            || !self.gmac.prepare_rx(storage, &mut self.rx_index)
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
        // The Type-2 receive checksum engine validates IPv4 headers and
        // IPv4/IPv6 TCP, UDP and ICMP payloads. The DMA drops checksum-error
        // frames while `Checksum::Tx` keeps software checksums enabled for
        // transmission. ESP32-S31 advertises TX checksum offload, but its
        // payload engine fails for Ethernet frames of 880 bytes and larger.
        capabilities.checksum.ipv4 = Checksum::Tx;
        capabilities.checksum.udp = Checksum::Tx;
        capabilities.checksum.tcp = Checksum::Tx;
        capabilities.checksum.icmpv4 = Checksum::Tx;
        capabilities.checksum.icmpv6 = Checksum::Tx;
        capabilities
    }

    fn hardware_address(&self) -> HardwareAddress {
        HardwareAddress::Ethernet(self.mac)
    }
}

//! # Interrupt support
//!
//! This module contains code to configure and handle peripheral interrupts.
//!
//! ## Overview
//!
//! Peripheral interrupt requests are not handled by the CPU directly. Instead, they are routed
//! through the interrupt matrix, which maps peripheral interrupts to CPU interrupts. There are
//! more peripheral interrupt signals than CPU interrupts, and multiple peripheral interrupts can
//! be routed to the same CPU interrupt. A set of CPU interrupts are configured to run a default
//! handler routine, which polls the interrupt controller and calls the appropriate handlers for the
//! pending peripheral interrupts.
//!
//! This default handler implements interrupt nesting - meaning a higher [`Priority`] interrupt can
//! preempt a lower priority interrupt handler. The number of priorities is a chip-specific detail.
//!
//! ## Usage
//!
//! Peripheral drivers manage interrupts for you. Where appropriate, a `set_interrupt_handler`
//! function is provided to register a function that handles interrupts at a chosen priority
//! level. Interrupt handler functions need to be marked by the [`#[handler]`][crate::handler]
//! attribute. These drivers also provide `listen` and `unlisten` functions that control whether an
//! interrupt will be generated for the matching event or not. For more information and examples,
//! consult the documentation of the specific peripheral drivers.
//!
//! If you are writing your own peripheral driver, you will need to first register interrupt
//! handlers using the [peripheral singletons'][crate::peripherals::I2C0] `bind_X_interrupt`
//! functions. You can use the matching `enable` and `disable` functions to control the peripheral
//! interrupt in the interrupt matrix, or you can, depending on the peripheral, set or clear the
//! appropriate enable bits in the `int_ena` register.
//!
//! ## Software Interrupts
//!
//! The [`software`] module implements software interrupts using peripheral interrupt signals.
#![cfg_attr(
    multi_core,
    doc = "Those signals can also carry cross-core communication."
)]
#![cfg_attr(
    all(feature = "rt", multi_core),
    doc = "
## Inter-Processor Call (IPC)

The [`ipc`] module posts a function to a CPU, and raises the IPC interrupt of that CPU.
"
)]
#![cfg_attr(
    all(feature = "rt", multi_core, xtensa),
    doc = "The HAL reserves `FROM_CPU_INTR0` and `FROM_CPU_INTR1` for this path."
)]
#![cfg_attr(
    all(feature = "rt", single_core, context_switch_source = "from_cpu"),
    doc = "The HAL reserves `FROM_CPU_INTR0` to switch tasks."
)]
#![cfg_attr(
    all(feature = "rt", context_switch_source = "software0"),
    doc = "
## Context Switching

The HAL defines the `Software0` interrupt handler, and switches tasks in that interrupt.
"
)]

use crate::{peripherals::Interrupt, system::Cpu};

cfg_select! {
    esp32 => {
        use crate::peripherals::{DPORT as INTERRUPT_CORE0, DPORT as INTERRUPT_CORE1};
    }
    _ => {
        use crate::peripherals::INTERRUPT_CORE0;
        #[cfg(multi_core)]
        use crate::peripherals::INTERRUPT_CORE1;
    }
}

#[cfg_attr(riscv, path = "riscv.rs")]
#[cfg_attr(xtensa, path = "xtensa.rs")]
mod arch;
pub use arch::*;

use crate::pac;

unstable_driver! {
    pub mod software;

    #[cfg(all(feature = "rt", multi_core))]
    pub mod ipc;

    #[cfg(feature = "rt")]
    #[doc(hidden)]
    pub mod __rtos_implementation;
}

#[cfg(feature = "rt")]
#[unsafe(no_mangle)]
extern "C" fn EspDefaultHandler() {
    panic!("Unhandled interrupt on {:?}", crate::system::Cpu::current());
}

/// Default (unhandled) interrupt handler
#[instability::unstable]
pub const DEFAULT_INTERRUPT_HANDLER: InterruptHandler = InterruptHandler::new(
    {
        unsafe extern "C" {
            fn EspDefaultHandler();
        }

        unsafe {
            core::mem::transmute::<unsafe extern "C" fn(), extern "C" fn()>(EspDefaultHandler)
        }
    },
    Priority::min(),
);

/// Trait implemented by drivers that support registering an
/// [`InterruptHandler`].
#[instability::unstable]
pub trait InterruptConfigurable: crate::private::Sealed {
    #[doc = cfg_select! {
        multi_core => "Registers an interrupt handler for the peripheral on the current core.",
        _ => "Registers an interrupt handler for the peripheral.",
    }]
    #[doc = ""]
    /// Replaces any previously registered interrupt handlers. Some peripherals
    /// offer a shared interrupt handler for multiple purposes. The caller must
    /// honor this constraint.
    ///
    /// The default/unhandled interrupt handler can be restored with
    /// [`DEFAULT_INTERRUPT_HANDLER`].
    fn set_interrupt_handler(&mut self, handler: InterruptHandler);
}

/// Represents an ISR callback function.
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct IsrCallback {
    f: extern "C" fn(),
}

impl IsrCallback {
    /// Creates a new callback from the callback function.
    pub fn new(f: extern "C" fn()) -> Self {
        // a valid fn pointer is non zero
        Self { f }
    }

    /// Returns the address of the callback.
    pub fn address(self) -> usize {
        self.f as usize
    }

    /// The callback function.
    ///
    /// This is aligned and can be called.
    pub fn callback(self) -> extern "C" fn() {
        self.f
    }
}

impl PartialEq for IsrCallback {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::fn_addr_eq(self.callback(), other.callback())
    }
}

/// An interrupt handler.
#[cfg_attr(
    multi_core,
    doc = r"

**Note**: Interrupts are handled on the core they were setup on. If a driver is initialized
on core 0, and moved to core 1, core 0 will still handle the interrupt."
)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct InterruptHandler {
    f: extern "C" fn(),
    prio: Priority,
}

impl InterruptHandler {
    /// Creates a new [`InterruptHandler`] which will call the given function at
    /// the given priority.
    pub const fn new(f: extern "C" fn(), prio: Priority) -> Self {
        Self { f, prio }
    }

    /// The Isr callback.
    #[inline]
    pub fn handler(&self) -> IsrCallback {
        IsrCallback::new(self.f)
    }

    /// Returns the priority used when registering the interrupt.
    #[inline]
    pub fn priority(&self) -> Priority {
        self.prio
    }
}

const STATUS_WORDS: usize = property!("interrupts.status_registers");

/// Representation of peripheral-interrupt status bits.
#[derive(Clone, Copy, Default, Debug)]
#[instability::unstable]
pub struct InterruptStatus {
    status: [u32; STATUS_WORDS],
}

impl InterruptStatus {
    /// Returns the status of a particular peripheral interrupt.
    #[instability::unstable]
    #[inline]
    pub fn is_pending(interrupt: Interrupt) -> bool {
        let word = (interrupt as usize) / 32;
        let bit = (interrupt as usize) % 32;
        let mut status_word = 0;
        for cpu in Cpu::all() {
            status_word |= Self::interrupt_status_word(cpu, word);
        }
        (status_word & (1 << bit)) != 0
    }

    #[inline(always)]
    fn interrupt_status_word(cpu: Cpu, word: usize) -> u32 {
        match cpu {
            Cpu::ProCpu => {
                cfg_select! {
                    esp32p4 => {
                        if word == 4 {
                            // Discontiguous, cannot be part of the standard status array
                            return INTERRUPT_CORE0::regs().core_0_intr_status4().read().bits();
                        }
                    }
                    _ => {}
                }

                INTERRUPT_CORE0::regs()
                    .core_0_intr_status(word)
                    .read()
                    .bits()
            }
            #[cfg(multi_core)]
            Cpu::AppCpu => {
                cfg_select! {
                    esp32p4 => {
                        if word == 4 {
                            // Discontiguous, cannot be part of the standard status array
                            return INTERRUPT_CORE1::regs().core_1_intr_status4().read().bits();
                        }
                    }
                    _ => {}
                }

                INTERRUPT_CORE1::regs()
                    .core_1_intr_status(word)
                    .read()
                    .bits()
            }
        }
    }

    /// Returns the status of peripheral interrupts.
    // Inlined into the RAM-resident interrupt dispatcher.
    #[inline(always)]
    #[instability::unstable]
    pub fn current() -> InterruptStatus {
        let cpu = Cpu::current();
        InterruptStatus {
            status: core::array::from_fn(|word| Self::interrupt_status_word(cpu, word)),
        }
    }

    /// Is the given interrupt bit set.
    #[instability::unstable]
    pub fn is_set(&self, interrupt: u8) -> bool {
        (self.status[interrupt as usize / 32] & (1 << (interrupt % 32))) != 0
    }

    /// Sets the given interrupt status bit.
    #[instability::unstable]
    pub fn set(&mut self, interrupt: u8) {
        self.status[interrupt as usize / 32] |= 1 << (interrupt % 32);
    }

    /// Returns an iterator over the set interrupt status bits.
    #[inline(always)]
    #[instability::unstable]
    pub fn iterator(&self) -> InterruptStatusIterator {
        InterruptStatusIterator {
            status: *self,
            idx: 0,
        }
    }
}

/// Iterator over set interrupt status bits
#[derive(Debug, Clone)]
#[instability::unstable]
pub struct InterruptStatusIterator {
    status: InterruptStatus,
    idx: usize,
}

impl Iterator for InterruptStatusIterator {
    type Item = u8;

    #[inline(always)]
    fn next(&mut self) -> Option<Self::Item> {
        for i in self.idx..STATUS_WORDS {
            if self.status.status[i] != 0 {
                let bit = self.status.status[i].trailing_zeros();
                self.status.status[i] ^= 1 << bit;
                self.idx = i;
                return Some((bit + 32 * i as u32) as u8);
            }
        }
        self.idx = usize::MAX;
        None
    }
}

// Peripheral interrupt API.

fn vector_entry(interrupt: Interrupt) -> &'static pac::Vector {
    unsafe extern "Rust" {
        #[cfg(xtensa)]
        static __INTERRUPTS: pac::Vector;
        #[cfg(riscv)]
        static __EXTERNAL_INTERRUPTS: pac::Vector;
    }

    // SAFETY: Interrupt numbers are guaranteed to be valid and in range because we use the
    // Interrupt enum, which is generated from the list of valid peripheral interrupts in the PAC.
    unsafe {
        cfg_select! {
            xtensa => (&__INTERRUPTS as *const pac::Vector)
                .add(interrupt as usize)
                .as_ref_unchecked(),
            riscv => (&__EXTERNAL_INTERRUPTS as *const pac::Vector)
                .add(interrupt as usize)
                .as_ref_unchecked(),
            _ => {
                compile_error!("Unsupported architecture");
            }
        }
    }
}

/// Returns the currently bound interrupt handler.
#[instability::unstable]
pub fn bound_handler(interrupt: Interrupt) -> Option<IsrCallback> {
    unsafe {
        let vector = vector_entry(interrupt);

        let addr = vector._handler;
        if addr as usize == 0 {
            return None;
        }

        Some(IsrCallback::new(core::mem::transmute::<
            unsafe extern "C" fn(),
            extern "C" fn(),
        >(addr)))
    }
}

/// Binds the given handler to a peripheral interrupt.
///
/// The interrupt handler will be enabled at the specified priority level.
///
/// The interrupt handler will be called on the core where it is registered.
/// Only one interrupt handler can be bound to a peripheral interrupt.
#[cfg(not(feature = "static-interrupts"))]
#[instability::unstable]
pub fn bind_handler(interrupt: Interrupt, handler: InterruptHandler) {
    bind_vector(interrupt, handler);
    enable(interrupt, handler.priority());
}

/// Binds `handler` to `interrupt` without enabling the peripheral interrupt.
#[cfg(not(feature = "static-interrupts"))]
pub(crate) fn bind_vector(interrupt: Interrupt, handler: InterruptHandler) {
    unsafe {
        let vector = vector_entry(interrupt);
        let ptr = (&raw const vector._handler).cast::<usize>().cast_mut();

        #[cfg(all(riscv, write_vec_table_monitoring))]
        if crate::soc::trap_section_protected() {
            crate::debugger::DEBUGGER_LOCK.lock(|| {
                let wp = crate::debugger::clear_watchpoint(1);
                ptr.write_volatile(handler.handler().address());
                crate::debugger::restore_watchpoint(1, wp);
            });
            return;
        }

        ptr.write_volatile(handler.handler().address());
    }
}

/// Enables a peripheral interrupt at a given priority, using vectored CPU interrupts.
///
/// Interrupts must still be enabled globally for interrupts to be serviced.
///
/// Internally maps the interrupt to the appropriate CPU interrupt
/// for the specified priority level.
#[cfg(not(feature = "static-interrupts"))]
#[inline]
#[instability::unstable]
pub fn enable(interrupt: Interrupt, level: Priority) {
    enable_on_cpu(Cpu::current(), interrupt, level);
}

pub(crate) fn enable_on_cpu(cpu: Cpu, interrupt: Interrupt, level: Priority) {
    let cpu_interrupt = priority_to_cpu_interrupt(interrupt, level);
    map_raw(cpu, interrupt, cpu_interrupt as u32);
}

/// Disables the given peripheral interrupt.
///
/// Internally maps the interrupt to a disabled CPU interrupt.
#[cfg(not(feature = "static-interrupts"))]
#[inline]
#[instability::unstable]
pub fn disable(core: Cpu, interrupt: Interrupt) {
    map_raw(core, interrupt, DISABLED_CPU_INTERRUPT)
}

/// A driver's binding under `static-interrupts`: the image's interrupt table
/// must already hold a handler for `interrupt`, which then counts as required
/// (see [`required_routes`]); the driver's own handler is not installed.
#[cfg(feature = "static-interrupts")]
pub(crate) fn bind_handler(interrupt: Interrupt, _handler: InterruptHandler) {
    require_route(interrupt);
}

/// A driver's route enable under `static-interrupts`: only the holder of
/// [`InterruptRoutes`] maps sources, so this requires the route instead.
#[cfg(feature = "static-interrupts")]
#[allow(dead_code, reason = "called only by some chips' drivers")]
pub(crate) fn enable(interrupt: Interrupt, _level: Priority) {
    require_route(interrupt);
}

/// A driver's route disable under `static-interrupts`: the table owns the
/// route, so a driver leaves it as it is.
#[cfg(feature = "static-interrupts")]
pub(crate) fn disable(_core: Cpu, _interrupt: Interrupt) {}

#[cfg(feature = "static-interrupts")]
static REQUIRED: [portable_atomic::AtomicU32; 8] = [const { portable_atomic::AtomicU32::new(0) }; 8];

/// Panic unless the image's table holds a handler for `interrupt` and its
/// owner has routed it on a core, and record it as required. A driver whose
/// interrupt nobody routes would otherwise wait forever: the owner routes the
/// source before it hands the driver its interrupt (`into_async`).
#[cfg(feature = "static-interrupts")]
fn require_route(interrupt: Interrupt) {
    let bound = bound_handler(interrupt)
        .is_some_and(|handler| handler != DEFAULT_INTERRUPT_HANDLER.handler());
    if !bound {
        panic!("{:?} has no handler in the image's interrupt table", interrupt);
    }
    if !Cpu::all().any(|cpu| mapped_to(cpu, interrupt).is_some()) {
        panic!("{:?} is not routed: its owner routes it before the driver needs it", interrupt);
    }
    let number = interrupt as usize;
    REQUIRED[number / 32].fetch_or(1 << (number % 32), portable_atomic::Ordering::AcqRel);
}

/// The peripheral interrupts a driver has required so far, by number: each
/// must be routed by the holder of [`InterruptRoutes`] before interrupts are
/// enabled.
#[cfg(feature = "static-interrupts")]
pub fn required_routes() -> impl Iterator<Item = u16> {
    (0..REQUIRED.len() * 32).filter_map(|number| {
        let word = REQUIRED[number / 32].load(portable_atomic::Ordering::Acquire);
        (word & (1 << (number % 32)) != 0).then_some(number as u16)
    })
}

/// The one capability that maps peripheral interrupts to CPU interrupts when
/// the `static-interrupts` feature makes the image's interrupt table their
/// only owner.
#[cfg(feature = "static-interrupts")]
#[derive(Debug)]
#[non_exhaustive]
pub struct InterruptRoutes;

#[cfg(feature = "static-interrupts")]
impl InterruptRoutes {
    /// Take the capability; `None` after the first call.
    pub fn take() -> Option<&'static Self> {
        use portable_atomic::{AtomicBool, Ordering};
        static TAKEN: AtomicBool = AtomicBool::new(false);
        static ROUTES: InterruptRoutes = InterruptRoutes;
        (!TAKEN.swap(true, Ordering::AcqRel)).then_some(&ROUTES)
    }

    /// Route `interrupt` on `cpu` to the vectored CPU interrupt of `level`.
    pub fn enable(&self, cpu: Cpu, interrupt: Interrupt, level: Priority) {
        enable_on_cpu(cpu, interrupt, level);
    }

    /// Route `interrupt` on `cpu` to the disabled CPU interrupt.
    pub fn disable(&self, cpu: Cpu, interrupt: Interrupt) {
        map_raw(cpu, interrupt, DISABLED_CPU_INTERRUPT)
    }
}

pub(super) fn map_raw(core: Cpu, interrupt: Interrupt, cpu_interrupt: u32) {
    match core {
        Cpu::ProCpu => {
            INTERRUPT_CORE0::regs()
                .core_0_intr_map(interrupt as usize)
                .modify(|_, w| unsafe { w.map().bits(cpu_interrupt as u8) });
        }
        #[cfg(multi_core)]
        Cpu::AppCpu => {
            INTERRUPT_CORE1::regs()
                .core_1_intr_map(interrupt as usize)
                .modify(|_, w| unsafe { w.map().bits(cpu_interrupt as u8) });
        }
    }
}

/// Returns the CPU interrupt `interrupt` is routed to on `cpu`, or `None` while it is disabled
/// there. It reads the interrupt matrix, so it needs no runtime.
#[instability::unstable]
pub fn mapped_to(cpu: Cpu, interrupt: Interrupt) -> Option<CpuInterrupt> {
    mapped_to_raw(cpu, interrupt as u32)
}

/// The enabled routes on `cpu`, by physical interrupt-matrix source number.
///
/// This includes every PAC-defined matrix slot, even a source absent from the
/// `Interrupt` enum. Fatal-error diagnostics can read these routes without
/// converting source numbers through compiler-generated enum switch tables,
/// which may live in cached memory. The iterator reads each route when visited
/// and needs neither allocation nor initialized runtime state.
#[cfg(esp32s31)]
#[instability::unstable]
#[inline(always)]
pub fn mapped_sources(cpu: Cpu) -> impl Iterator<Item = (usize, CpuInterrupt)> {
    // Iterate the PAC's register references directly: indexing by a derived
    // source count can retain bounds-check panic metadata in cached memory.
    let core0 = matches!(cpu, Cpu::ProCpu)
        .then(|| INTERRUPT_CORE0::regs().core_0_intr_map_iter())
        .into_iter()
        .flatten()
        .enumerate()
        .filter_map(|(source, route)| {
            CpuInterrupt::from_u32(u32::from(route.read().map().bits()))
                .map(|line| (source, line))
        });
    let core1 = matches!(cpu, Cpu::AppCpu)
        .then(|| INTERRUPT_CORE1::regs().core_1_intr_map_iter())
        .into_iter()
        .flatten()
        .enumerate()
        .filter_map(|(source, route)| {
            CpuInterrupt::from_u32(u32::from(route.read().map().bits()))
                .map(|line| (source, line))
        });
    core0.chain(core1)
}

pub(crate) fn mapped_to_raw(cpu: Cpu, interrupt: u32) -> Option<CpuInterrupt> {
    let cpu_intr = match cpu {
        Cpu::ProCpu => INTERRUPT_CORE0::regs()
            .core_0_intr_map(interrupt as usize)
            .read()
            .map()
            .bits() as u32,
        #[cfg(multi_core)]
        Cpu::AppCpu => INTERRUPT_CORE1::regs()
            .core_1_intr_map(interrupt as usize)
            .read()
            .map()
            .bits() as u32,
    };
    CpuInterrupt::from_u32(cpu_intr)
}

/// Represents the priority level of running code.
///
/// Interrupts at or below this level are masked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum RunLevel {
    /// Thread mode.
    ///
    /// This is the lowest level. No interrupts are masked by the run level.
    ThreadMode,

    /// An elevated level, usually an interrupt handler's.
    Interrupt(ElevatedRunLevel),
}

impl RunLevel {
    #[procmacros::doc_replace]
    /// Returns the current run level.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::interrupt::RunLevel;
    ///
    /// let level = RunLevel::current();
    /// println!("Current run level: {:?}", level);
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn current() -> Self {
        let raw = current_raw_runlevel();
        unwrap!(Self::try_from_u32(raw))
    }

    /// Changes the current run level to the specified level and returns the previous level.
    ///
    /// # Safety
    ///
    /// Must only be used to raise the run level and to restore it to a previous
    /// value. Must not be used to arbitrarily lower the run level.
    #[inline]
    #[instability::unstable]
    pub unsafe fn change(to: Self) -> Self {
        unsafe { change_current_runlevel(to) }
    }

    #[procmacros::doc_replace]
    /// Returns whether the run level indicates thread mode.
    ///
    /// Thread mode means the CPU is not executing an interrupt handler.
    ///
    /// # Examples
    ///
    /// ```rust, no_run
    /// # {before_snippet}
    /// use esp_hal::interrupt::RunLevel;
    ///
    /// let level = RunLevel::current();
    ///
    /// if level.is_thread() {
    ///     println!("Running in thread mode");
    /// } else {
    ///     println!("Running in an interrupt handler");
    /// }
    /// # {after_snippet}
    /// ```
    #[inline]
    pub fn is_thread(&self) -> bool {
        matches!(self, RunLevel::ThreadMode)
    }

    pub(crate) fn try_from_u32(priority: u32) -> Result<Self, PriorityError> {
        if priority == 0 {
            Ok(RunLevel::ThreadMode)
        } else {
            ElevatedRunLevel::try_from_u32(priority).map(RunLevel::Interrupt)
        }
    }
}

impl PartialEq<ElevatedRunLevel> for RunLevel {
    fn eq(&self, other: &ElevatedRunLevel) -> bool {
        *self == RunLevel::Interrupt(*other)
    }
}

impl PartialEq<Priority> for RunLevel {
    fn eq(&self, other: &Priority) -> bool {
        let runlevel = ElevatedRunLevel::from(*other);
        *self == RunLevel::Interrupt(runlevel)
    }
}

impl From<RunLevel> for u32 {
    fn from(level: RunLevel) -> Self {
        match level {
            RunLevel::ThreadMode => 0,
            RunLevel::Interrupt(priority) => priority as u32,
        }
    }
}

/// Priority Level Error.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub enum PriorityError {
    /// The priority is not valid.
    InvalidInterruptPriority,
}

#[instability::unstable]
impl TryFrom<u32> for RunLevel {
    type Error = PriorityError;

    fn try_from(priority: u32) -> Result<Self, Self::Error> {
        Self::try_from_u32(priority)
    }
}

#[cfg(feature = "rt")]
pub(crate) fn setup_interrupts() {
    // disable all known peripheral interrupts
    for peripheral_interrupt in 0..255 {
        crate::peripherals::Interrupt::try_from(peripheral_interrupt)
            .map(|intr| {
                #[cfg(multi_core)]
                map_raw(Cpu::AppCpu, intr, DISABLED_CPU_INTERRUPT);
                map_raw(Cpu::ProCpu, intr, DISABLED_CPU_INTERRUPT);
            })
            .ok();
    }

    unsafe { crate::interrupt::init_vectoring() };
}

/// Reinitializes this core's interrupt controller and vector table after handoff.
///
/// All named peripheral interrupt mappings on every core are disabled and, on
/// ESP32-S31, their pass levels are reset to zero (the ESP32-C5 matrix has no
/// pass level). This permits a separately linked second-stage image to install
/// its own handlers with a known interrupt-matrix baseline. Peripheral drivers
/// must be initialized after this function returns.
///
/// # Safety
///
/// Interrupts must be globally disabled, any other core must be stopped, and
/// no live peripheral driver may rely on the previous interrupt mappings.
#[cfg(all(any(esp32s31, esp32c5), feature = "rt"))]
#[instability::unstable]
pub unsafe fn reinitialize_vectoring_after_handoff() {
    // Normal interrupt routing preserves pass levels. A second-stage image
    // instead takes ownership of both matrices and establishes its baseline.
    #[cfg(esp32s31)]
    for peripheral_interrupt in 0..255 {
        if let Ok(interrupt) = Interrupt::try_from(peripheral_interrupt) {
            INTERRUPT_CORE0::regs()
                .core_0_intr_map(interrupt as usize)
                .modify(|_, w| unsafe { w.pass_level().bits(0) });
            INTERRUPT_CORE1::regs()
                .core_1_intr_map(interrupt as usize)
                .modify(|_, w| unsafe { w.pass_level().bits(0) });
        }
    }
    setup_interrupts();
}

#[inline(always)]
#[cfg(feature = "rt")]
fn should_handle(core: Cpu, interrupt_nr: u32, level: u32) -> bool {
    if let Some(cpu_interrupt) = crate::interrupt::mapped_to_raw(core, interrupt_nr)
        && cpu_interrupt.is_vectored()
        && cpu_interrupt.level() == level
    {
        true
    } else {
        false
    }
}

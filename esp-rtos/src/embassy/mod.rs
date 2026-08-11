//! OS-aware embassy executors.

use core::{cell::UnsafeCell, mem::MaybeUninit, ptr::NonNull, sync::atomic::Ordering};

use embassy_executor::{SendSpawner, Spawner, raw};
use esp_hal::{
    interrupt::{InterruptHandler, Priority, software::SoftwareInterrupt},
    system::Cpu,
    time::{Duration, Instant},
};
use macros::ram;
use portable_atomic::AtomicPtr;

#[cfg(esp32s31)]
use crate::task::Task;
use crate::{SCHEDULER, task::read_thread_pointer};
#[cfg(not(esp32s31))]
use crate::{scheduler::SchedulerState, task::TaskPtr};

#[cfg(not(esp32s31))]
struct SchedulerLocked<T> {
    inner: UnsafeCell<T>,
}

#[cfg(not(esp32s31))]
unsafe impl<T: Send> Sync for SchedulerLocked<T> {}
#[cfg(not(esp32s31))]
unsafe impl<T: Send> Send for SchedulerLocked<T> {}

#[cfg(not(esp32s31))]
impl<T> SchedulerLocked<T> {
    fn new(inner: T) -> Self {
        Self {
            inner: UnsafeCell::new(inner),
        }
    }

    fn with<'s>(&'s self, _scheduler: &'s mut SchedulerState) -> &'s mut T {
        // The scheduler borrow proves exclusive access to the inner value.
        unsafe { &mut *self.inner.get() }
    }
}

#[cfg(not(esp32s31))]
struct FlagsInner {
    owner: TaskPtr,
    waiting: Option<TaskPtr>,
    set: bool,
}

#[cfg(not(esp32s31))]
impl FlagsInner {
    fn take(&mut self) -> bool {
        if self.set {
            self.set = false;
            true
        } else {
            self.waiting = Some(self.owner);
            false
        }
    }
}

#[cfg(not(esp32s31))]
struct ThreadFlag {
    inner: SchedulerLocked<FlagsInner>,
}

#[cfg(not(esp32s31))]
impl ThreadFlag {
    fn new() -> Self {
        let owner = SCHEDULER.with(|scheduler| {
            if let Some(current_task) = NonNull::new(read_thread_pointer()) {
                current_task
            } else {
                let current_cpu = Cpu::current();
                NonNull::from(&scheduler.per_cpu[current_cpu as usize].main_task)
            }
        });
        Self {
            inner: SchedulerLocked::new(FlagsInner {
                owner,
                waiting: None,
                set: false,
            }),
        }
    }

    fn with<R>(&self, scheduler: &mut SchedulerState, f: impl FnOnce(&mut FlagsInner) -> R) -> R {
        f(self.inner.with(scheduler))
    }

    fn set(&self) {
        SCHEDULER.with(|scheduler| {
            let to_resume = self.with(scheduler, |inner| {
                let to_resume = inner.waiting.take();
                if to_resume.is_none() {
                    inner.set = true;
                }
                to_resume
            });

            if let Some(waiting) = to_resume {
                scheduler.resume_task(waiting);
            }
        });
    }

    fn get(&self) -> bool {
        SCHEDULER.with(|scheduler| self.with(scheduler, |inner| inner.set))
    }

    fn wait(&self) {
        SCHEDULER.with(|scheduler| {
            let owner_to_suspend = self.with(scheduler, |inner| {
                if !inner.take() {
                    Some(inner.owner)
                } else {
                    None
                }
            });

            if let Some(owner) = owner_to_suspend {
                scheduler.sleep_task_until(owner, Instant::EPOCH + Duration::MAX);
                crate::task::yield_task();
            }
        });
    }
}

/// A single event bit, optimized for the thread-mode embassy executor.
///
/// This takes shortcuts, which make it unsuitable for general purpose use (such as no wait
/// queue, no timeout, assumes a single thread waits for the flag, there is only a single bit of
/// flag information).
#[cfg(esp32s31)]
struct ThreadFlag {
    owner: AtomicPtr<Task>,
    state: AtomicPtr<Task>,
}

#[cfg(esp32s31)]
impl ThreadFlag {
    const SET: *mut Task = core::ptr::without_provenance_mut(1);

    fn new() -> Self {
        let owner = SCHEDULER.with(|scheduler| {
            if let Some(current_task) = NonNull::new(read_thread_pointer()) {
                current_task
            } else {
                // We're cheating, the task hasn't been initialized yet.
                let current_cpu = Cpu::current();
                NonNull::from(&scheduler.per_cpu[current_cpu as usize].main_task)
            }
        });
        Self {
            owner: AtomicPtr::new(owner.as_ptr()),
            state: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    fn set(&self) {
        // Most calls to `__pender` happen while the executor is polling and therefore don't need
        // the scheduler at all. This is important: a waker may run while the current CPU already
        // holds a mutable scheduler borrow, so unconditionally locking it here is reentrant.
        let mut state = self.state.load(Ordering::Acquire);
        loop {
            if state == Self::SET {
                return;
            }

            if state.is_null() {
                match self.state.compare_exchange(
                    core::ptr::null_mut(),
                    Self::SET,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => return,
                    Err(actual) => state = actual,
                }
            } else {
                match self.state.compare_exchange(
                    state,
                    core::ptr::null_mut(),
                    Ordering::AcqRel,
                    Ordering::Acquire,
                ) {
                    Ok(_) => {
                        let waiting = unwrap!(NonNull::new(state));
                        SCHEDULER.with(|scheduler| scheduler.resume_task(waiting));
                        return;
                    }
                    Err(actual) => state = actual,
                }
            }
        }
    }

    fn get(&self) -> bool {
        self.state.load(Ordering::Acquire) == Self::SET
    }

    fn wait(&self) {
        // SCHEDULER.sleep_until, but we know the current task's ID, and we know there
        // is no timeout.
        SCHEDULER.with(|scheduler| {
            let owner_ptr = self.owner.load(Ordering::Relaxed);
            let owner = unwrap!(NonNull::new(owner_ptr));
            let mut state = self.state.load(Ordering::Acquire);

            loop {
                if state == Self::SET {
                    match self.state.compare_exchange(
                        Self::SET,
                        core::ptr::null_mut(),
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    ) {
                        Ok(_) => return,
                        Err(actual) => state = actual,
                    }
                } else if state.is_null() {
                    match self.state.compare_exchange(
                        core::ptr::null_mut(),
                        owner_ptr,
                        Ordering::AcqRel,
                        Ordering::Acquire,
                    ) {
                        Ok(_) => break,
                        Err(actual) => state = actual,
                    }
                } else {
                    unreachable!("a ThreadFlag has only one waiting task")
                }
            }

            // If `set` claims the waiter before this call, it blocks on the scheduler lock until
            // the task is sleeping. It can then safely put the task back in the ready queue.
            scheduler.sleep_task_until(owner, Instant::EPOCH + Duration::MAX);
            crate::task::yield_task();
        });
    }
}

#[unsafe(export_name = "__pender")]
#[ram]
fn __pender(context: *mut ()) {
    match context as usize {
        0 => unsafe { SoftwareInterrupt::<0>::steal().raise() },
        1 => unsafe { SoftwareInterrupt::<1>::steal().raise() },
        2 => unsafe { SoftwareInterrupt::<2>::steal().raise() },
        3 => unsafe { SoftwareInterrupt::<3>::steal().raise() },
        _ => {
            // This forces us to keep the embassy timer queue separate, otherwise we'd need to
            // reentrantly lock SCHEDULER.
            let flags = unwrap!(unsafe { context.cast::<ThreadFlag>().as_ref() });
            flags.set();
        }
    }
}

/// Callbacks to run code before/after polling the task queue.
pub trait Callbacks {
    /// Called just before polling the executor.
    fn before_poll(&mut self);

    /// Called after the executor is polled, if there is no work scheduled.
    ///
    /// Note that tasks can become ready at any point during the execution
    /// of this function.
    fn on_idle(&mut self);
}

/// Thread-mode executor.
///
/// This executor runs in an OS thread, meaning the scheduler needs to be started before using any
/// async operations. If you wish to write async code without the scheduler running, consider
/// using the [`InterruptExecutor`].
#[cfg_attr(
    multi_core,
    doc = r"

If you want to start the executor on the second core, you will need to start the second core using
[`crate::start_second_core`], or [`crate::start_on_second_core_only`] if the first core should stay
bare-metal.
If you are looking for a way to run code on the second core without the scheduler, use the [`InterruptExecutor`].
"
)]
pub struct Executor {
    executor: UnsafeCell<MaybeUninit<raw::Executor>>,
}

impl Executor {
    /// Create a new thread-mode executor.
    pub const fn new() -> Self {
        Self {
            executor: UnsafeCell::new(MaybeUninit::uninit()),
        }
    }

    /// Run the executor.
    ///
    /// The `init` closure is called with a [`Spawner`] that spawns tasks on
    /// this executor. Use it to spawn the initial task(s). After `init`
    /// returns, the executor starts running the tasks.
    ///
    /// To spawn more tasks later, you may keep copies of the [`Spawner`] (it is
    /// `Copy`), for example by passing it as an argument to the initial
    /// tasks.
    ///
    /// This function requires `&'static mut self`. This means you have to store
    /// the Executor instance in a place where it'll live forever and grants
    /// you mutable access. There's a few ways to do this:
    ///
    /// - a [StaticCell](https://docs.rs/static_cell/latest/static_cell/) (safe)
    /// - a `static mut` (unsafe, not recommended)
    /// - a local variable in a function you know never returns (like `fn main() -> !`), upgrading
    ///   its lifetime with `transmute`. (unsafe)
    ///
    /// This function never returns.
    pub fn run(&'static mut self, init: impl FnOnce(Spawner)) -> ! {
        let flags = ThreadFlag::new();
        struct NoHooks;

        impl Callbacks for NoHooks {
            fn before_poll(&mut self) {}

            fn on_idle(&mut self) {}
        }

        self.run_inner(init, &flags, NoHooks)
    }

    /// Run the executor with callbacks.
    ///
    /// See [Callbacks] on when the callbacks are called.
    ///
    /// See [Self::run] for more information about running the executor.
    ///
    /// This function never returns.
    pub fn run_with_callbacks(
        &'static mut self,
        init: impl FnOnce(Spawner),
        callbacks: impl Callbacks,
    ) -> ! {
        let flags = ThreadFlag::new();
        struct Hooks<'a, CB: Callbacks>(CB, &'a ThreadFlag);

        impl<CB: Callbacks> Callbacks for Hooks<'_, CB> {
            fn before_poll(&mut self) {
                self.0.before_poll()
            }

            fn on_idle(&mut self) {
                // Make sure we only call on_idle if the executor would otherwise go to sleep.
                if !self.1.get() {
                    self.0.on_idle();
                }
            }
        }

        self.run_inner(init, &flags, Hooks(callbacks, &flags))
    }

    fn run_inner(
        &'static self,
        init: impl FnOnce(Spawner),
        flags: &ThreadFlag,
        mut hooks: impl Callbacks,
    ) -> ! {
        let executor = unsafe {
            (&mut *self.executor.get()).write(raw::Executor::new(
                (flags as *const ThreadFlag).cast::<()>().cast_mut(),
            ))
        };

        // The main task may start the scheduler from inside this executor, so we cannot require the
        // scheduler to run already. We can, however, refuse to run on a CPU that the scheduler
        // never runs on.
        #[cfg(multi_core)]
        if crate::SCHEDULER.with(|scheduler| !scheduler.active_cores.contains(Cpu::current())) {
            panic!("Executor cannot be started: the scheduler is not running on the current CPU.");
        }

        init(executor.spawner());

        loop {
            hooks.before_poll();

            unsafe { executor.poll() };

            hooks.on_idle();

            // Wait for work to become available.
            flags.wait();
        }
    }
}

impl Default for Executor {
    fn default() -> Self {
        Self::new()
    }
}

/// Interrupt mode executor.
///
/// This executor runs tasks in interrupt mode. The interrupt handler is set up
/// to poll tasks, and when a task is woken the interrupt is pended from
/// software.
///
/// Interrupt executors have potentially lower latency than thread-mode executors, but only a
/// limited number can be created.
pub struct InterruptExecutor<const SWI: u8> {
    executor: UnsafeCell<MaybeUninit<raw::Executor>>,
    interrupt: SoftwareInterrupt<'static, SWI>,
}

const COUNT: usize = 4;
static INTERRUPT_EXECUTORS: [InterruptExecutorStorage; COUNT] =
    [const { InterruptExecutorStorage::new() }; COUNT];

unsafe impl<const SWI: u8> Send for InterruptExecutor<SWI> {}
unsafe impl<const SWI: u8> Sync for InterruptExecutor<SWI> {}

struct InterruptExecutorStorage {
    raw_executor: AtomicPtr<raw::Executor>,
}

impl InterruptExecutorStorage {
    const fn new() -> Self {
        Self {
            raw_executor: AtomicPtr::new(core::ptr::null_mut()),
        }
    }

    /// # Safety:
    ///
    /// The caller must ensure `set` has been called before.
    #[inline(always)]
    unsafe fn get(&self) -> &raw::Executor {
        unsafe { &*self.raw_executor.load(Ordering::Relaxed) }
    }

    fn set(&self, executor: *mut raw::Executor) {
        self.raw_executor.store(executor, Ordering::Relaxed);
    }
}

extern "C" fn handle_interrupt<const NUM: u8>() {
    let swi = unsafe { SoftwareInterrupt::<NUM>::steal() };
    swi.reset();

    unsafe {
        // SAFETY: The executor is always initialized before the interrupt is enabled.
        let executor = INTERRUPT_EXECUTORS[NUM as usize].get();
        executor.poll();
    }
}

impl<const SWI: u8> InterruptExecutor<SWI> {
    /// Create a new `InterruptExecutor`.
    /// This takes the software interrupt to be used internally.
    #[inline]
    pub const fn new(interrupt: SoftwareInterrupt<'static, SWI>) -> Self {
        Self {
            executor: UnsafeCell::new(MaybeUninit::uninit()),
            interrupt,
        }
    }

    /// Start the executor at the given priority level.
    ///
    /// This initializes the executor, enables the interrupt, and returns.
    /// The executor keeps running in the background through the interrupt.
    ///
    /// This returns a [`SendSpawner`] you can use to spawn tasks on it. A
    /// [`SendSpawner`] is returned instead of a [`Spawner`] because the
    /// executor effectively runs in a different "thread" (the interrupt),
    /// so spawning tasks on it is effectively sending them.
    ///
    /// To obtain a [`Spawner`] for this executor, use [`Spawner::for_current_executor`]
    /// from a task running in it.
    pub fn start(&'static mut self, priority: Priority) -> SendSpawner {
        unsafe {
            (*self.executor.get()).write(raw::Executor::new((SWI as usize) as *mut ()));

            INTERRUPT_EXECUTORS[SWI as usize].set((*self.executor.get()).as_mut_ptr());
        }

        let swi_handler = match SWI {
            0 => handle_interrupt::<0>,
            1 => handle_interrupt::<1>,
            2 => handle_interrupt::<2>,
            3 => handle_interrupt::<3>,
            _ => unreachable!(),
        };

        self.interrupt
            .set_interrupt_handler(InterruptHandler::new(swi_handler, priority));

        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.spawner().make_send()
    }

    /// Get a SendSpawner for this executor
    ///
    /// This returns a [`SendSpawner`] you can use to spawn tasks on this
    /// executor.
    ///
    /// This MUST only be called on an executor that has already been started.
    /// The function will panic otherwise.
    pub fn spawner(&'static self) -> SendSpawner {
        if INTERRUPT_EXECUTORS[SWI as usize]
            .raw_executor
            .load(Ordering::Acquire)
            .is_null()
        {
            panic!("InterruptExecutor::spawner() called on uninitialized executor.");
        }
        let executor = unsafe { (*self.executor.get()).assume_init_ref() };
        executor.spawner().make_send()
    }
}

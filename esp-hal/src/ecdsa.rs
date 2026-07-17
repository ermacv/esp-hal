//! ECDSA signature accelerator.
//!
//! The ESP32-S31 peripheral consumes little-endian curve components. This
//! driver exposes the conventional big-endian SEC1/raw-signature formats and
//! performs the conversion at the hardware boundary.

use core::{
    future::poll_fn,
    mem::size_of,
    sync::atomic::{AtomicBool, Ordering},
    task::Poll,
};

use crate::{
    asynch::AtomicWaker,
    handler,
    peripherals::ECDSA,
    ram,
    system::{self, GenericPeripheralGuard},
};

const P256_COMPONENT_LEN: usize = 32;
const P256_RAW_SIGNATURE_LEN: usize = 64;
const P256_SEC1_PUBLIC_KEY_LEN: usize = 65;

// Order of NIST P-256 (secp256r1), big-endian.
const P256_ORDER: [u8; P256_COMPONENT_LEN] = [
    0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xbc, 0xe6, 0xfa, 0xad, 0xa7, 0x17, 0x9e, 0x84, 0xf3, 0xb9, 0xca, 0xc2, 0xfc, 0x63, 0x25, 0x51,
];

static WAKER: AtomicWaker = AtomicWaker::new();
static INTERRUPT_FIRED: AtomicBool = AtomicBool::new(false);

#[handler]
#[ram]
fn interrupt_handler() {
    Ecdsa::disable_interrupts();
    INTERRUPT_FIRED.store(true, Ordering::Release);
    WAKER.wake();
}

/// A SHA-256 digest prepared for ECDSA P-256 verification.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct P256Digest([u8; P256_COMPONENT_LEN]);

impl P256Digest {
    /// Wrap a SHA-256 digest in its algorithm-specific type.
    pub const fn new(bytes: [u8; P256_COMPONENT_LEN]) -> Self {
        Self(bytes)
    }
}

/// A raw ECDSA P-256 signature (`r || s`), in big-endian byte order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct P256Signature {
    r: [u8; P256_COMPONENT_LEN],
    s: [u8; P256_COMPONENT_LEN],
}

impl P256Signature {
    /// Parse a raw P-256 signature and reject scalars outside `[1, n - 1]`.
    pub fn from_bytes(bytes: [u8; P256_RAW_SIGNATURE_LEN]) -> Result<Self, InvalidSignature> {
        let mut r = [0; P256_COMPONENT_LEN];
        let mut s = [0; P256_COMPONENT_LEN];
        r.copy_from_slice(&bytes[..P256_COMPONENT_LEN]);
        s.copy_from_slice(&bytes[P256_COMPONENT_LEN..]);

        if !scalar_is_in_range(&r) || !scalar_is_in_range(&s) {
            return Err(InvalidSignature);
        }

        Ok(Self { r, s })
    }
}

/// An uncompressed SEC1 P-256 public key (`0x04 || x || y`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct P256PublicKey {
    x: [u8; P256_COMPONENT_LEN],
    y: [u8; P256_COMPONENT_LEN],
}

impl P256PublicKey {
    /// Parse an uncompressed SEC1 public key.
    ///
    /// Point-on-curve verification is performed by the accelerator during
    /// signature verification. The identity is rejected here explicitly.
    pub fn from_sec1_bytes(
        bytes: [u8; P256_SEC1_PUBLIC_KEY_LEN],
    ) -> Result<Self, InvalidPublicKey> {
        if bytes[0] != 0x04 {
            return Err(InvalidPublicKey);
        }

        let mut x = [0; P256_COMPONENT_LEN];
        let mut y = [0; P256_COMPONENT_LEN];
        x.copy_from_slice(&bytes[1..=P256_COMPONENT_LEN]);
        y.copy_from_slice(&bytes[P256_COMPONENT_LEN + 1..]);

        if x.iter().all(|byte| *byte == 0) && y.iter().all(|byte| *byte == 0) {
            return Err(InvalidPublicKey);
        }

        Ok(Self { x, y })
    }
}

/// A raw signature has an invalid scalar encoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidSignature;

/// A SEC1 public key has an invalid encoding.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct InvalidPublicKey;

/// ESP32-S31 ECDSA accelerator driver.
pub struct Ecdsa<'d> {
    _ecdsa: ECDSA<'d>,
    _memory: EccMemoryPowerGuard,
    _ecc_guard: GenericPeripheralGuard<{ system::Peripheral::Ecc as u8 }>,
    _ecdsa_guard: GenericPeripheralGuard<{ system::Peripheral::Ecdsa as u8 }>,
}

impl<'d> Ecdsa<'d> {
    /// Create the ECDSA driver and enable its shared ECC arithmetic engine.
    pub fn new(ecdsa: ECDSA<'d>) -> Self {
        ecdsa.disable_peri_interrupt_on_all_cores();
        ecdsa.bind_peri_interrupt(interrupt_handler);
        ecdsa.enable_peri_interrupt(interrupt_handler.priority());

        let ecdsa_guard = GenericPeripheralGuard::new();
        let ecc_guard = GenericPeripheralGuard::new();
        let memory = EccMemoryPowerGuard::new();

        while Self::state() != State::Idle {}

        Self {
            _ecdsa: ecdsa,
            _memory: memory,
            _ecc_guard: ecc_guard,
            _ecdsa_guard: ecdsa_guard,
        }
    }

    /// Verify an ECDSA P-256 signature over a caller-supplied SHA-256 digest.
    ///
    /// This method waits synchronously for the accelerator. Use
    /// [`Self::verify_p256_prehashed_async`] to yield the CPU between hardware
    /// stage interrupts.
    pub fn verify_p256_prehashed(
        &mut self,
        digest: &P256Digest,
        signature: &P256Signature,
        public_key: &P256PublicKey,
    ) -> bool {
        self.start();
        while Self::state() != State::Load {
            core::hint::spin_loop();
        }
        Self::load(digest, signature, public_key);
        while Self::state() != State::Idle {
            core::hint::spin_loop();
        }

        let valid = Self::result();
        Self::disable_interrupts();
        clear_interrupts();
        valid
    }

    /// Asynchronously verify an ECDSA P-256 signature over a SHA-256 digest.
    ///
    /// The CPU is yielded while the accelerator prepares its input window and
    /// while the ECC calculation is in progress. Dropping the returned future
    /// resets the peripheral, so cancellation cannot strand it in `LOAD` or
    /// `BUSY` state.
    pub async fn verify_p256_prehashed_async(
        &mut self,
        digest: &P256Digest,
        signature: &P256Signature,
        public_key: &P256PublicKey,
    ) -> bool {
        let mut operation = OperationGuard::new(self);
        operation.start();
        wait_for_interrupt().await;
        debug_assert_eq!(Self::state(), State::Load);
        Self::load(digest, signature, public_key);
        INTERRUPT_FIRED.store(false, Ordering::Release);
        Self::enable_post_interrupt();
        wait_for_interrupt().await;
        debug_assert_eq!(Self::state(), State::Idle);
        operation.finish(Self::result())
    }

    fn start(&mut self) {
        let regs = ECDSA::regs();
        regs.conf().write(|w| unsafe {
            w.work_mode()
                .bits(0)
                .ecc_curve()
                .bits(1)
                .software_set_z()
                .set_bit()
                .use_hardware_key()
                .clear_bit()
        });
        Self::disable_interrupts();
        clear_interrupts();
        regs.start().write(|w| w.start().set_bit());
    }

    fn load(digest: &P256Digest, signature: &P256Signature, public_key: &P256PublicKey) {
        let regs = ECDSA::regs();
        write_reversed_words(regs.z_mem(0).as_ptr().cast(), &digest.0);
        write_reversed_words(regs.r_mem(0).as_ptr().cast(), &signature.r);
        write_reversed_words(regs.s_mem(0).as_ptr().cast(), &signature.s);
        write_reversed_words(regs.qax_mem(0).as_ptr().cast(), &public_key.x);
        write_reversed_words(regs.qay_mem(0).as_ptr().cast(), &public_key.y);
        regs.start().write(|w| w.load_done().set_bit());
    }

    fn result() -> bool {
        ECDSA::regs()
            .result()
            .read()
            .operation_result()
            .bit_is_set()
    }

    fn disable_interrupts() {
        ECDSA::regs().int_ena().write(|w| w);
    }

    fn enable_prep_interrupt() {
        ECDSA::regs()
            .int_ena()
            .write(|w| w.prep_done_int_ena().set_bit());
    }

    fn enable_post_interrupt() {
        ECDSA::regs()
            .int_ena()
            .write(|w| w.post_done_int_ena().set_bit());
    }

    fn state() -> State {
        match ECDSA::regs().state().read().busy().bits() {
            0 => State::Idle,
            1 => State::Load,
            2 => State::Get,
            _ => State::Busy,
        }
    }
}

struct OperationGuard<'a, 'd> {
    driver: &'a mut Ecdsa<'d>,
    active: bool,
}

impl<'a, 'd> OperationGuard<'a, 'd> {
    fn new(driver: &'a mut Ecdsa<'d>) -> Self {
        Self {
            driver,
            active: true,
        }
    }

    fn start(&mut self) {
        INTERRUPT_FIRED.store(false, Ordering::Release);
        self.driver.start();
        Ecdsa::enable_prep_interrupt();
    }

    fn finish(mut self, result: bool) -> bool {
        Ecdsa::disable_interrupts();
        clear_interrupts();
        self.active = false;
        result
    }
}

impl Drop for OperationGuard<'_, '_> {
    fn drop(&mut self) {
        if !self.active {
            return;
        }

        Ecdsa::disable_interrupts();
        clear_interrupts();
        reset_peripheral();
    }
}

async fn wait_for_interrupt() {
    poll_fn(|cx| {
        WAKER.register(cx.waker());
        if INTERRUPT_FIRED.swap(false, Ordering::Acquire) {
            Poll::Ready(())
        } else {
            Poll::Pending
        }
    })
    .await;
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Idle,
    Load,
    Get,
    Busy,
}

struct EccMemoryPowerGuard;

impl EccMemoryPowerGuard {
    fn new() -> Self {
        crate::peripherals::HP_SYSTEM::regs()
            .ecc_mem_lp_ctrl()
            .modify(|_, w| {
                w.ecc_mem_lp_en()
                    .clear_bit()
                    .ecc_mem_lp_force_ctrl()
                    .set_bit()
            });
        Self
    }
}

impl Drop for EccMemoryPowerGuard {
    fn drop(&mut self) {
        crate::peripherals::HP_SYSTEM::regs()
            .ecc_mem_lp_ctrl()
            .modify(|_, w| {
                w.ecc_mem_lp_force_ctrl()
                    .clear_bit()
                    .ecc_mem_lp_en()
                    .set_bit()
            });
    }
}

fn clear_interrupts() {
    ECDSA::regs().int_clr().write(|w| {
        w.prep_done_int_clr()
            .set_bit()
            .proc_done_int_clr()
            .set_bit()
            .post_done_int_clr()
            .set_bit()
            .sha_release_int_clr()
            .set_bit()
    });
}

fn reset_peripheral() {
    crate::peripherals::HP_SYS_CLKRST::regs()
        .crypto_ctrl0()
        .modify(|_, w| w.reg_crypto_ecdsa_rst_en().set_bit());
    crate::peripherals::HP_SYS_CLKRST::regs()
        .crypto_ctrl0()
        .modify(|_, w| {
            w.reg_crypto_ecdsa_rst_en()
                .clear_bit()
                .reg_crypto_rst_en()
                .clear_bit()
        });
}

fn write_reversed_words(base: *mut u32, input: &[u8; P256_COMPONENT_LEN]) {
    for index in 0..P256_COMPONENT_LEN / size_of::<u32>() {
        let end = P256_COMPONENT_LEN - index * size_of::<u32>();
        let word = u32::from_le_bytes([
            input[end - 1],
            input[end - 2],
            input[end - 3],
            input[end - 4],
        ]);

        // SAFETY: every ECDSA parameter window is at least 32 bytes long and
        // 32-bit aligned. `index` is bounded to its eight P-256 words, and the
        // driver owns the peripheral exclusively for the duration of the write.
        unsafe { base.add(index).write_volatile(word) };
    }
}

fn scalar_is_in_range(value: &[u8; P256_COMPONENT_LEN]) -> bool {
    if value.iter().all(|byte| *byte == 0) {
        return false;
    }

    for (lhs, rhs) in value.iter().zip(P256_ORDER) {
        if *lhs < rhs {
            return true;
        }
        if *lhs > rhs {
            return false;
        }
    }

    false
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn p256_scalar_range_rejects_zero_and_order() {
        assert!(!scalar_is_in_range(&[0; P256_COMPONENT_LEN]));
        assert!(!scalar_is_in_range(&P256_ORDER));

        let mut order_minus_one = P256_ORDER;
        order_minus_one[P256_COMPONENT_LEN - 1] -= 1;
        assert!(scalar_is_in_range(&order_minus_one));
    }
}

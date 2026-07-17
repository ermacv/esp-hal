//! ECDSA signature accelerator.
//!
//! The ESP32-S31 peripheral consumes little-endian curve components. This
//! driver exposes the conventional big-endian SEC1/raw-signature formats and
//! performs the conversion at the hardware boundary.

use crate::{
    peripherals::ECDSA,
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
    /// This initial API waits synchronously for the accelerator. An async
    /// operation handle will use the peripheral interrupts once their exact
    /// S31 stage semantics have been qualified on hardware.
    pub fn verify_p256_prehashed(
        &mut self,
        digest: &P256Digest,
        signature: &P256Signature,
        public_key: &P256PublicKey,
    ) -> bool {
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
        regs.int_ena().write(|w| w);
        clear_interrupts();

        regs.start().write(|w| w.start().set_bit());
        while Self::state() != State::Load {}

        write_reversed(&digest.0, |index, value| {
            regs.z_mem(index).write(|w| unsafe { w.bits(value) });
        });
        write_reversed(&signature.r, |index, value| {
            regs.r_mem(index).write(|w| unsafe { w.bits(value) });
        });
        write_reversed(&signature.s, |index, value| {
            regs.s_mem(index).write(|w| unsafe { w.bits(value) });
        });
        write_reversed(&public_key.x, |index, value| {
            regs.qax_mem(index).write(|w| unsafe { w.bits(value) });
        });
        write_reversed(&public_key.y, |index, value| {
            regs.qay_mem(index).write(|w| unsafe { w.bits(value) });
        });

        regs.start().write(|w| w.load_done().set_bit());
        while Self::state() != State::Idle {}

        let valid = regs.result().read().operation_result().bit_is_set();
        clear_interrupts();
        valid
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

fn write_reversed(input: &[u8; P256_COMPONENT_LEN], mut write: impl FnMut(usize, u8)) {
    for (index, value) in input.iter().rev().copied().enumerate() {
        write(index, value);
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

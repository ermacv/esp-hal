//! Finite polling of one USB IN transfer, without an interrupt or a lock.
use core::task::Poll;

pub(super) trait Control {
    fn clear_completion(&mut self);
    fn publish(&mut self);
    fn complete(&self) -> bool;
    fn ready(&self) -> bool;
}

#[inline(always)]
pub(super) fn poll(pending: &mut bool, control: &mut impl Control) -> Poll<()> {
    if !*pending {
        // IN_EMPTY is sticky: only an event after this publication can
        // complete it. DATA_FREE can still describe the preceding packet.
        control.clear_completion();
        control.publish();
        *pending = true;
        return Poll::Pending;
    }
    if !control.complete() || !control.ready() {
        return Poll::Pending;
    }
    *pending = false;
    Poll::Ready(())
}

#[cfg(test)]
#[path = "flush_tests.rs"]
mod tests;

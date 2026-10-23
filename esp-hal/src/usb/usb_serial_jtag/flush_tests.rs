use super::*;

#[derive(Default)]
struct Port {
    completion: bool,
    ready: bool,
    publications: usize,
}
impl Control for Port {
    fn clear_completion(&mut self) {
        self.completion = false;
    }
    fn publish(&mut self) {
        self.publications += 1;
    }
    fn complete(&self) -> bool {
        self.completion
    }
    fn ready(&self) -> bool {
        self.ready
    }
}

#[test]
fn pending_retries_never_publish_another_transfer() {
    let mut pending = false;
    let mut port = Port::default();
    for _ in 0..100 {
        assert_eq!(poll(&mut pending, &mut port), Poll::Pending);
    }
    assert_eq!(port.publications, 1);
    port.completion = true;
    port.ready = true;
    assert_eq!(poll(&mut pending, &mut port), Poll::Ready(()));
    assert_eq!(port.publications, 1);
}

#[test]
fn a_stale_empty_event_cannot_complete_a_new_flush() {
    let mut pending = false;
    let mut port = Port {
        completion: true,
        ready: true,
        ..Default::default()
    };
    assert_eq!(poll(&mut pending, &mut port), Poll::Pending);
    assert_eq!(poll(&mut pending, &mut port), Poll::Pending);
    assert!(!port.completion);
}

#[test]
fn an_early_empty_event_still_waits_for_fifo_readiness() {
    let mut pending = false;
    let mut port = Port::default();
    assert_eq!(poll(&mut pending, &mut port), Poll::Pending);
    port.completion = true;
    assert_eq!(poll(&mut pending, &mut port), Poll::Pending);
    port.ready = true;
    assert_eq!(poll(&mut pending, &mut port), Poll::Ready(()));
    assert_eq!(port.publications, 1);
}

#[test]
fn the_next_flush_has_its_own_publication_and_completion() {
    let mut pending = false;
    let mut port = Port::default();
    for count in 1..=2 {
        assert_eq!(poll(&mut pending, &mut port), Poll::Pending);
        assert_eq!(port.publications, count);
        assert!(!port.completion);
        port.completion = true;
        port.ready = true;
        assert_eq!(poll(&mut pending, &mut port), Poll::Ready(()));
    }
}

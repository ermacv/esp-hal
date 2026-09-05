use super::contains;

#[test]
fn executable_span_must_belong_entirely_to_the_live_mapping() {
    let mapping = 0x5000_0000..0x5100_0000;
    assert!(contains(mapping.clone(), mapping.start, mapping.len()));
    assert!(contains(mapping.clone(), mapping.start + 7, 41));
    assert!(contains(mapping.clone(), mapping.end, 0));
    assert!(!contains(mapping.clone(), mapping.start - 1, 1));
    assert!(!contains(mapping.clone(), mapping.end - 1, 2));
    assert!(!contains(mapping.clone(), mapping.end, 1));
    assert!(!contains(mapping.clone(), mapping.start, usize::MAX));
}

#[test]
fn no_code_can_be_published_before_a_mapping_is_established() {
    assert!(!contains(0..0, 0, 0));
    assert!(!contains(0..0, 0, 1));
    assert!(!contains(5..4, 5, 0));
}

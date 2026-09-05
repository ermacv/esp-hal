use super::*;

#[test]
fn candidate_reads_stay_within_complete_pages() {
    assert_eq!(
        page_count(0x10000, 0x42010000, 16 * 4096, 16 * 1024 * 1024),
        Some(16)
    );
    assert_eq!(
        page_count(0, 0x42000000, 15 * 4096 - 1, 16 * 1024 * 1024),
        None
    );
    assert_eq!(
        page_count(0, 0x42000000, 15 * 4096 + 128, 16 * 1024 * 1024),
        Some(15)
    );
    assert_eq!(
        page_count(0, 0x42000000, usize::MAX, 16 * 1024 * 1024),
        Some(31)
    );
}

#[test]
fn rejects_out_of_chip_unaligned_and_wrapping_mappings() {
    assert_eq!(page_count(4096, 0x42000000, 15 * 4096, 15 * 4096), None);
    assert_eq!(page_count(1, 0x42000000, 16 * 4096, 16 * 1024 * 1024), None);
    assert_eq!(page_count(0, 0x42000001, 16 * 4096, 16 * 1024 * 1024), None);
    assert_eq!(
        page_count(0, usize::MAX & !4095, 16 * 4096, 16 * 1024 * 1024),
        None
    );
    assert_eq!(
        page_count(u32::MAX & !4095, 0x42000000, 16 * 4096, u32::MAX),
        None
    );
}

#[test]
fn requires_a_stable_window_and_ignores_out_of_table_candidates() {
    assert_eq!(select_window(0), None);
    assert_eq!(select_window(0b11011011011), None);
    assert_eq!(select_window(0xf000), None);
    assert_eq!(select_window(0b111), Some(1));
    assert_eq!(select_window(0b1111 << 8), Some(10));
    assert_eq!(select_window(0b1111_0111), Some(6));
    assert_eq!(select_window(0b111_0111), Some(1));
}

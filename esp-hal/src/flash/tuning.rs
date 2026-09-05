//! Pure bounds and stable-window selection for the destructive timing sweep.

/// Read only complete pages, reserve twelve candidate pages and three final
/// verification pages, and never step outside either the mapping or the chip.
#[inline(always)]
pub(super) fn page_count(
    physical: u32,
    virtual_start: usize,
    size: usize,
    capacity: u32,
) -> Option<usize> {
    const PAGE_BYTES: usize = 4096;
    if !physical.is_multiple_of(PAGE_BYTES as u32) || !virtual_start.is_multiple_of(PAGE_BYTES) {
        return None;
    }
    let pages = (size / PAGE_BYTES).min(31);
    let bytes = pages * PAGE_BYTES;
    if pages < 15 || physical.checked_add(bytes as u32)? > capacity {
        return None;
    }
    virtual_start.checked_add(bytes)?;
    Some(pages)
}

/// Middle of the longest run of at least three passing candidates. Ties keep
/// the first window, matching the controller's existing sweep order.
#[inline(always)]
pub(super) fn select_window(mask: u16) -> Option<usize> {
    let mut longest_start = 0;
    let mut longest_len = 0;
    let mut current_start = 0;
    let mut current_len = 0;
    let mut candidate = 0;
    while candidate < 12 {
        if mask & (1 << candidate) != 0 {
            if current_len == 0 {
                current_start = candidate;
            }
            current_len += 1;
            if current_len > longest_len {
                longest_start = current_start;
                longest_len = current_len;
            }
        } else {
            current_len = 0;
        }
        candidate += 1;
    }
    if longest_len >= 3 {
        Some(longest_start + longest_len / 2)
    } else {
        None
    }
}

#[cfg(test)]
#[path = "tuning_tests.rs"]
mod tests;

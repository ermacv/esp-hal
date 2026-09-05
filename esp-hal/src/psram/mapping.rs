use core::ops::Range;

/// Checks the complete requested span before a cache operation can publish it.
#[inline(always)]
pub(super) fn contains(mapping: Range<usize>, start: usize, size: usize) -> bool {
    mapping.start < mapping.end
        && start >= mapping.start
        && start
            .checked_add(size)
            .is_some_and(|end| end <= mapping.end)
}

#[cfg(test)]
#[path = "mapping_tests.rs"]
mod tests;

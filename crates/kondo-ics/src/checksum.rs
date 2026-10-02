//! Checksum (even parity) calculation.

/// Calculates even parity.
///
/// Counts the set bits in the data and returns the even parity bit.
#[must_use]
pub fn calculate_parity(data: &[u8]) -> u8 {
    let mut parity = 0u8;
    for &byte in data {
        parity ^= byte;
    }
    // Count set bits.
    let ones = parity.count_ones();
    // Even parity: return 1 when the number of set bits is odd.
    u8::from(ones % 2 == 1)
}

/// Validates parity.
#[must_use]
pub fn validate_parity(data: &[u8], expected: u8) -> bool {
    calculate_parity(data) == expected
}

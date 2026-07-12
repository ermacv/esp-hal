//! ESP32-S31 eFuse fields used by the HAL.

use crate::efuse::EfuseField;

// ESP-IDF esp_efuse_table.csv: MAC_FACTORY occupies BLK1 bits 0..47.
pub const MAC0: EfuseField = EfuseField::new(1, 0, 0, 32);
pub const MAC1: EfuseField = EfuseField::new(1, 1, 0, 16);

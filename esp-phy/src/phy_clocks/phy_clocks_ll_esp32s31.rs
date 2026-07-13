use core::ptr::{read_volatile, write_volatile};

const HP_MODEM_CONF: *mut u32 = (0x2058_7000 + 0x1e0) as *mut u32;
const MODEM_SYSCON_CLK_CONF: *mut u32 = (0x2010_9c00 + 0x04) as *mut u32;
const MODEM_SYSCON_CLK_CONF1: *mut u32 = (0x2010_9c00 + 0x14) as *mut u32;
const MODEM_LPCON_CLK_CONF: *mut u32 = (0x2010_f000 + 0x18) as *mut u32;

const PHY_FE_CLOCKS: u32 = (1 << 15) | (1 << 13) | (1 << 14) | (1 << 21) | (1 << 19) | (1 << 20);
const I2C_MASTER_CLOCK: u32 = 1 << 2;
const I2C_MASTER_SELECT_160M: u32 = 1 << 12;

#[inline]
unsafe fn update_bits(register: *mut u32, mask: u32, enable: bool) {
    // SAFETY: callers serialize PHY clock transitions through esp-phy's global lock. The
    // addresses and masks come from ESP-IDF's ESP32-S31 modem register headers.
    let value = unsafe { read_volatile(register) };
    unsafe { write_volatile(register, if enable { value | mask } else { value & !mask }) };
}

pub(crate) fn enable_phy(enable: bool) {
    unsafe {
        // ESP-IDF uses 0x3d while any modem client needs the SoC PLL source, and 0x25 when the
        // final client releases it.
        write_volatile(HP_MODEM_CONF, if enable { 0x3d } else { 0x25 });
        update_bits(MODEM_SYSCON_CLK_CONF1, PHY_FE_CLOCKS, enable);
        update_bits(MODEM_SYSCON_CLK_CONF, I2C_MASTER_SELECT_160M, enable);
        update_bits(MODEM_LPCON_CLK_CONF, I2C_MASTER_CLOCK, enable);
    }
}

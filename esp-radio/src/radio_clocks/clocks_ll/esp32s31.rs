use core::ptr::{read_volatile, write_volatile};

const HP_MODEM_CTRL0: *mut u32 = (0x2058_7000 + 0x40) as *mut u32;
const HP_MODEM_CONF: *mut u32 = (0x2058_7000 + 0x1e0) as *mut u32;
const MODEM_SYSCON_CLK_CONF_POWER_ST: *mut u32 = (0x2010_9c00 + 0x0c) as *mut u32;
const MODEM_SYSCON_RST_CONF: *mut u32 = (0x2010_9c00 + 0x10) as *mut u32;
const MODEM_SYSCON_CLK_CONF1: *mut u32 = (0x2010_9c00 + 0x14) as *mut u32;
const MODEM_LPCON_CLK_CONF: *mut u32 = (0x2010_f000 + 0x18) as *mut u32;
const MODEM_LPCON_CLK_CONF_POWER_ST: *mut u32 = (0x2010_f000 + 0x20) as *mut u32;

const WIFI_CLOCKS: u32 = 0x7ff;
const COEX_CLOCK: u32 = 1 << 1;

#[inline]
unsafe fn update_bits(register: *mut u32, mask: u32, enable: bool) {
    // SAFETY: radio clock changes are serialized by esp-radio. Addresses and masks are copied
    // from the ESP32-S31 modem register headers in ESP-IDF.
    let value = unsafe { read_volatile(register) };
    unsafe { write_volatile(register, if enable { value | mask } else { value & !mask }) };
}

pub(crate) fn enable_wifi(enable: bool) {
    unsafe {
        if enable {
            reset_mask(1 << 8);
        }
        update_bits(MODEM_SYSCON_CLK_CONF1, WIFI_CLOCKS, enable);
        update_bits(MODEM_LPCON_CLK_CONF, COEX_CLOCK, enable);
        write_volatile(HP_MODEM_CONF, if enable { 0x3d } else { 0x25 });
    }
}

pub(crate) fn reset_wifi_mac() {
    unsafe { reset_mask(1 << 9) }
}

pub(crate) fn reset_wifi_subsystem() {
    // ESP32-S31 exposes the Wi-Fi baseband and MAC reset lines as bits 8 and 9
    // of MODEM_SYSCON_MODEM_RST_CONF. The other modem reset fields used by
    // C5/C6 are laid out differently and are not part of the S31 Wi-Fi path.
    unsafe { reset_mask((1 << 8) | (1 << 9)) }
}

unsafe fn reset_mask(mask: u32) {
    unsafe {
        update_bits(MODEM_SYSCON_RST_CONF, mask, true);
        update_bits(MODEM_SYSCON_RST_CONF, mask, false);
    }
}

pub(crate) fn init_clocks() {
    unsafe {
        update_bits(HP_MODEM_CTRL0, 1, true);
        update_bits(MODEM_SYSCON_CLK_CONF_POWER_ST, 0x6464_6400, true);
        update_bits(MODEM_LPCON_CLK_CONF_POWER_ST, 0x6666_0000, true);
    }
}

pub(crate) fn enable_bt(_enable: bool) {}
pub(crate) fn enable_ieee802154(_enable: bool) {}
pub(crate) fn ble_rtc_clk_init() {}
pub(crate) fn reset_rpa() {}

use core::ptr::{read_volatile, write_volatile};

const HP_MODEM_CONF: *mut u32 = (0x2058_7000 + 0x1e0) as *mut u32;
const MODEM_SYSCON_CLK_CONF: *mut u32 = (0x2010_9c00 + 0x04) as *mut u32;
const MODEM_SYSCON_CLK_CONF_POWER_ST: *mut u32 = (0x2010_9c00 + 0x0c) as *mut u32;
const MODEM_SYSCON_MODEM_RST_CONF: *mut u32 = (0x2010_9c00 + 0x10) as *mut u32;
const MODEM_SYSCON_CLK_CONF1: *mut u32 = (0x2010_9c00 + 0x14) as *mut u32;
const MODEM_LPCON_CLK_CONF: *mut u32 = (0x2010_f000 + 0x18) as *mut u32;
const MODEM_LPCON_CLK_CONF_POWER_ST: *mut u32 = (0x2010_f000 + 0x20) as *mut u32;

const PHY_FE_CLOCKS: u32 = (1 << 15) | (1 << 13) | (1 << 14) | (1 << 21) | (1 << 19) | (1 << 20);
// ESP-IDF's PERIPH_PHY_CALIBRATION_MODULE dependencies. Full RF calibration opens both
// Wi-Fi and BT/802.15.4 baseband paths even when Wi-Fi is the only radio being initialized.
const PHY_CALIBRATION_CLOCKS: u32 = 0x17b | (1 << 2) | (1 << 7) | (1 << 10) | (1 << 16) | (1 << 17);
const WIFI_BB_RESET: u32 = 1 << 8;
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
        // Keep every modem domain ungated in HP-active and the shared domains ungated in
        // HP-modem mode. ESP-IDF applies these ICG maps before every modem clock request.
        update_bits(MODEM_SYSCON_CLK_CONF_POWER_ST, 0x6464_6400, true);
        update_bits(MODEM_LPCON_CLK_CONF_POWER_ST, 0x6666_0000, true);

        // ESP-IDF uses 0x3d while any modem client needs the SoC PLL source, and 0x25 when the
        // final client releases it.
        write_volatile(HP_MODEM_CONF, if enable { 0x3d } else { 0x25 });
        if enable {
            // Match modem_clock_wifi_bb_configure(): reset the BB before its clocks are opened.
            update_bits(MODEM_SYSCON_MODEM_RST_CONF, WIFI_BB_RESET, true);
            update_bits(MODEM_SYSCON_MODEM_RST_CONF, WIFI_BB_RESET, false);
        }
        update_bits(MODEM_SYSCON_CLK_CONF1, PHY_CALIBRATION_CLOCKS, enable);
        update_bits(MODEM_SYSCON_CLK_CONF1, PHY_FE_CLOCKS, enable);
        update_bits(MODEM_SYSCON_CLK_CONF, I2C_MASTER_SELECT_160M, enable);
        update_bits(MODEM_LPCON_CLK_CONF, I2C_MASTER_CLOCK, enable);
    }
}

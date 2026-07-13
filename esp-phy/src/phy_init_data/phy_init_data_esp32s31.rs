use crate::sys::include::esp_phy_init_data_t;

const CONFIG_ESP_PHY_MAX_TX_POWER: u8 = 20;

const fn limit(value: u8, high: u8) -> u8 {
    if value > high { high } else { value }
}

const fn default_init_data() -> esp_phy_init_data_t {
    let tx_power = CONFIG_ESP_PHY_MAX_TX_POWER * 4;
    let mut params = [0; 128];

    // Keep this table in lockstep with ESP-IDF's
    // components/esp_phy/esp32s31/phy_init_data.c.
    params[0] = 0x01;
    params[2] = limit(tx_power, 0x54);
    params[3] = limit(tx_power, 0x54);
    params[4] = limit(tx_power, 0x50);
    params[5] = limit(tx_power, 0x50);
    params[6] = limit(tx_power, 0x4c);
    params[7] = limit(tx_power, 0x48);
    params[8] = limit(tx_power, 0x50);
    params[9] = limit(tx_power, 0x50);
    params[10] = limit(tx_power, 0x4c);
    params[11] = limit(tx_power, 0x48);
    params[12] = limit(tx_power, 0x40);
    params[13] = limit(tx_power, 0x3c);
    params[14] = limit(tx_power, 0x3c);
    params[15] = limit(tx_power, 0x3c);
    params[16] = limit(tx_power, 0x4c);
    params[17] = limit(tx_power, 0x4c);
    params[18] = limit(tx_power, 0x48);
    params[19] = limit(tx_power, 0x44);
    params[127] = 0x51;

    esp_phy_init_data_t { params }
}

pub(crate) static PHY_INIT_DATA_DEFAULT: esp_phy_init_data_t = default_init_data();

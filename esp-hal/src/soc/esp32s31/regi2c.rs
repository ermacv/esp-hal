//! Analog register-I2C hooks pending clock-switching support.

pub(crate) fn regi2c_read(_block: u8, _host_id: u8, _register: u8) -> u8 {
    0
}

pub(crate) fn regi2c_write(_block: u8, _host_id: u8, _register: u8, _data: u8) {}

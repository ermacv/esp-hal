use crate::{
    peripherals::{CNNT_SYS, HP_ALIVE_SYS},
    system::{Peripheral, PeripheralClockControl},
};

pub fn hs_enable_device_mode() {
    hs_init();
    connect_hs_pulldowns(false);
}

pub fn hs_enable_host_mode() {
    hs_init();
    connect_hs_pulldowns(true);
}

fn hs_init() {
    PeripheralClockControl::enable(Peripheral::UsbHs);

    CNNT_SYS::regs().sys_usb_otg20_ctrl().modify(|_, w| {
        w.sys_usb_otg20_utmifs_clk_en().set_bit();
        w.sys_usb_otg20_phyref_clk_en().set_bit()
    });

    HP_ALIVE_SYS::regs().usb_otghs_ctrl().modify(|_, w| {
        w.reg_usb_otghs_phy_suspendm_force_en().clear_bit();
        w.reg_usb_otghs_phy_pll_force_en().clear_bit()
    });

    CNNT_SYS::regs().sys_usb_otg20_ctrl().modify(|_, w| {
        w.sys_usb_otg20_ahb_rst_en().set_bit();
        w.sys_usb_otg20_apb_rst_en().set_bit();
        w.sys_usb_otg20_phy_rst_en().set_bit()
    });
    CNNT_SYS::regs()
        .sys_usb_otg20_ctrl()
        .modify(|_, w| w.sys_usb_otg20_phy_rst_en().clear_bit());
    CNNT_SYS::regs().sys_usb_otg20_ctrl().modify(|_, w| {
        w.sys_usb_otg20_ahb_rst_en().clear_bit();
        w.sys_usb_otg20_apb_rst_en().clear_bit()
    });

    HP_ALIVE_SYS::regs()
        .usb_otghs_ctrl()
        .modify(|_, w| w.reg_usb_otghs_phy_otg_suspendm().set_bit());

    // Parallel low-speed mode with keep-alive (ESP-IDF `usb_utmi_ll_configure_ls`).
    // SAFETY: USB_HS ownership provides exclusive access to the UTMI PHY.
    let utmi = unsafe { &*crate::pac::USB_UTMI::ptr() };
    utmi.fc_06().modify(|_, w| {
        w.ls_par_en().set_bit();
        w.ls_kpalv_en().set_bit()
    });
}

fn connect_hs_pulldowns(connect: bool) {
    HP_ALIVE_SYS::regs().usb_ctrl().modify(|_, w| {
        w.usb_otghs_phy_dppulldown().bit(connect);
        w.usb_otghs_phy_dmpulldown().bit(connect)
    });
}

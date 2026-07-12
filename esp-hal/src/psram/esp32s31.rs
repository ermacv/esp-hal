use core::ops::Range;

use crate::peripherals::{
    CACHE, CPU_APM, HP_APM, HP_MEM_APM, HP_SYS_CLKRST, IOMUX_MSPI_PIN, PSRAM_MSPI,
};

use super::{EXTMEM_ORIGIN, PsramSize};

#[derive(Copy, Clone, Debug, Default, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramConfig {
    /// PSRAM size. `AutoDetect` reads MR2 density.
    pub size: PsramSize,
    /// Preserve cache/MMU state established by the second-stage bootloader.
    ///
    /// Set this for applications executing directly from mapped flash.
    pub cache_already_initialized: bool,
}

#[repr(C)]
struct RomSpiCommand {
    command: u16,
    command_bits: u16,
    address: *mut u32,
    address_bits: u32,
    tx_data: *mut u32,
    tx_bits: u32,
    rx_data: *mut u32,
    rx_bits: u32,
    dummy_bits: u32,
}

unsafe extern "C" {
    fn esp_rom_spi_set_op_mode(spi_num: i32, mode: i32);
    fn esp_rom_spi_cmd_config(spi_num: i32, command: *mut RomSpiCommand);
    fn esp_rom_spi_cmd_start(
        spi_num: i32,
        receive: *mut u8,
        receive_length: u16,
        chip_select_mask: u8,
        write_or_erase: bool,
    );
    fn ROM_Boot_Cache_Init();
}

pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    let clocks = HP_SYS_CLKRST::regs();
    clocks.psram_ctrl0().modify(|_, w| {
        w.reg_psram_sys_clk_en().set_bit();
        w.reg_psram_pll_clk_en().set_bit();
        w.reg_psram_core_clk_en().set_bit();
        unsafe {
            w.reg_psram_clk_src_sel().bits(0);
            w.reg_psram_core_clk_div_num().bits(0);
        }
        w
    });
    clocks.psram_ctrl0().modify(|_, w| {
        w.reg_psram_axi_rst_en().set_bit();
        w.reg_psram_apb_rst_en().set_bit()
    });
    clocks.psram_ctrl0().modify(|_, w| {
        w.reg_psram_axi_rst_en().clear_bit();
        w.reg_psram_apb_rst_en().clear_bit()
    });

    let mut address = 0u32;
    let mut mode_registers = 0u32;
    let mut command = RomSpiCommand {
        command: 0x4040,
        command_bits: 16,
        address: &mut address,
        address_bits: 32,
        tx_data: core::ptr::null_mut(),
        tx_bits: 0,
        rx_data: &mut mode_registers,
        rx_bits: 16,
        dummy_bits: 8,
    };

    unsafe {
        esp_rom_spi_set_op_mode(3, 7); // OPI DTR
        esp_rom_spi_cmd_config(3, &mut command);
        esp_rom_spi_cmd_start(
            3,
            (&mut mode_registers as *mut u32).cast(),
            2,
            1 << 1,
            false,
        );
    }
    let vendor = ((mode_registers >> 8) & 0x1f) as u8;
    if vendor != 0x0d && vendor != 0x1a {
        return false;
    }

    address = 2;
    mode_registers = 0;
    command.address = &mut address;
    command.rx_data = &mut mode_registers;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut command);
        esp_rom_spi_cmd_start(
            3,
            (&mut mode_registers as *mut u32).cast(),
            2,
            1 << 1,
            false,
        );
    }
    let detected_size = match mode_registers & 0x07 {
        1 => 4 * 1024 * 1024,
        3 => 8 * 1024 * 1024,
        5 => 16 * 1024 * 1024,
        7 => 32 * 1024 * 1024,
        6 => 64 * 1024 * 1024,
        _ => return false,
    };
    if config.size.is_auto() {
        config.size = PsramSize::Size(detected_size);
    }

    // Fixed read latency 2 and write latency 2 for the conservative clock.
    let mut mr01 = 0u32;
    address = 0;
    command.address = &mut address;
    command.rx_data = &mut mr01;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut command);
        esp_rom_spi_cmd_start(3, (&mut mr01 as *mut u32).cast(), 2, 1 << 1, false);
    }
    mr01 = (mr01 & 0xffff_ff00) | ((mr01 & 0xc0) | 0x28);
    let mut register_write = RomSpiCommand {
        command: 0xc0c0,
        command_bits: 16,
        address: &mut address,
        address_bits: 32,
        tx_data: &mut mr01,
        tx_bits: 16,
        rx_data: core::ptr::null_mut(),
        rx_bits: 0,
        dummy_bits: 0,
    };
    unsafe {
        esp_rom_spi_cmd_config(3, &mut register_write);
        esp_rom_spi_cmd_start(3, core::ptr::null_mut(), 0, 1 << 1, false);
    }

    let mut mr48 = 0u32;
    address = 4;
    command.address = &mut address;
    command.rx_data = &mut mr48;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut command);
        esp_rom_spi_cmd_start(3, (&mut mr48 as *mut u32).cast(), 2, 1 << 1, false);
    }
    mr48 = (mr48 & 0xffff_ff00) | ((mr48 & 0x1f) | (2 << 5));
    register_write.address = &mut address;
    register_write.tx_data = &mut mr48;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut register_write);
        esp_rom_spi_cmd_start(3, core::ptr::null_mut(), 0, 1 << 1, false);
    }
    true
}

pub(crate) fn map_psram(config: PsramConfig) -> Range<usize> {
    let size = config.size.get();
    if size == 0 {
        return 0..0;
    }

    unsafe {
        HP_SYS_CLKRST::regs().cache_ctrl0().modify(|_, w| {
            w.reg_cpu_acache_cpu_clk_force_on().set_bit();
            w.reg_rom_acache_mem_clk_force_on().set_bit();
            w.reg_cpu_cache_cpu_clk_force_on().set_bit();
            w.reg_mspi_cache_sys_clk_force_on().set_bit()
        });
        if !config.cache_already_initialized {
            ROM_Boot_Cache_Init();
        }

        let clocks = HP_SYS_CLKRST::regs();
        clocks.psram_ctrl0().modify(|_, w| {
            w.reg_psram_core_clk_en().set_bit();
            w.reg_psram_sys_clk_en().set_bit();
            w.reg_psram_clk_src_sel().bits(1);
            w.reg_psram_core_clk_div_num().bits(0);
            w
        });

        let psram = PSRAM_MSPI::regs();
        psram
            .sram_clk()
            .write(|w| w.bits((19 << 16) | (9 << 8) | 19));
        psram
            .spi_smem_timing_cali()
            .modify(|_, w| w.spi_smem_dll_timing_cali().set_bit());
        IOMUX_MSPI_PIN::regs()
            .psram_dqs_0_pin0()
            .modify(|_, w| w.reg_psram_dqs_0_xpd().set_bit());

        psram.sram_drd_cmd().write(|w| w.bits(15 << 28));
        psram.sram_dwr_cmd().write(|w| w.bits((15 << 28) | 0x8080));
        psram.cache_sctrl().write(|w| {
            w.bits(
                1 | (1 << 3)
                    | (1 << 4)
                    | (1 << 5)
                    | (17 << 6)
                    | (31 << 14)
                    | (1 << 20)
                    | (1 << 21)
                    | (7 << 22),
            )
        });
        psram.sram_cmd().modify(|r, w| {
            w.bits((r.bits() & !((0xf << 18) | (0x3 << 26))) | (0xf << 18) | (1 << 23))
        });
        psram
            .spi_smem_ddr()
            .modify(|r, w| w.bits((r.bits() & !0xf) | 0x3));
        psram
            .spi_smem_ecc_ctrl()
            .modify(|_, w| w.spi_smem_page_size().bits(3));
        psram
            .spi_smem_ac()
            .write(|w| w.bits(1 | (1 << 1) | (3 << 2) | (3 << 7) | (2 << 25) | (1 << 31)));
        psram.ctrl1().modify(|_, w| {
            w.ar_splice_en().set_bit();
            w.aw_splice_en().set_bit()
        });
        psram.cache_fctrl().modify(|_, w| {
            w.spi_close_axi_inf_en().clear_bit();
            w.axi_req_en().set_bit()
        });
        psram
            .mmu_power_ctrl()
            .modify(|_, w| w.spi_mmu_page_size().bits(0));

        let page_count = size / 0x1_0000;
        for page in 0..page_count {
            psram.mmu_item_index().write(|w| w.bits(page as u32));
            psram
                .mmu_item_content()
                .write(|w| w.bits(page as u32 | (1 << 11) | (1 << 10)));
        }
        CACHE::regs()
            .l1_dcache_ctrl()
            .modify(|_, w| w.l1_dcache_shut_dbus0().clear_bit());
        CPU_APM::regs().region0_attr().write(|w| w.bits(0x7777));
        HP_MEM_APM::regs().region0_attr().write(|w| w.bits(0x7777));
        HP_APM::regs().region0_attr().write(|w| w.bits(0x7777));
    }

    EXTMEM_ORIGIN..EXTMEM_ORIGIN + size
}

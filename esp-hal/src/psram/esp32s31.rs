use core::ops::Range;

use crate::peripherals::{
    CACHE, CPU_APM, HP_APM, HP_MEM_APM, HP_SYS_CLKRST, IOMUX_MSPI_PIN, LP_AON_CLK_RST, PMU,
    PSRAM_MSPI,
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
    /// PSRAM bus timing. ESP-IDF uses 200 MHz as the S31 default.
    pub timing: PsramTiming,
}

/// OPI-DTR PSRAM timing parameters.
#[derive(Copy, Clone, Debug, PartialEq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[instability::unstable]
pub struct PsramTiming {
    /// PSRAM bus clock in MHz.
    pub clock_mhz: u32,
    mr0_read_latency: u8,
    mr4_write_latency: u8,
    read_dummy_bits: u32,
    write_dummy_bits: u32,
    register_dummy_bits: u32,
}

impl PsramTiming {
    /// Conservative 20 MHz timing, mainly useful for diagnostics.
    pub const MHZ_20: Self = Self {
        clock_mhz: 20,
        mr0_read_latency: 2,
        mr4_write_latency: 2,
        read_dummy_bits: 18,
        write_dummy_bits: 8,
        register_dummy_bits: 8,
    };

    /// 100 MHz OPI-DTR mode (400 MHz MPLL / 4).
    pub const MHZ_100: Self = Self {
        clock_mhz: 100,
        mr0_read_latency: 2,
        mr4_write_latency: 2,
        read_dummy_bits: 18,
        write_dummy_bits: 8,
        register_dummy_bits: 8,
    };

    /// ESP32-S31's normal 200 MHz OPI-DTR mode (400 MHz MPLL / 2).
    pub const MHZ_200: Self = Self {
        clock_mhz: 200,
        mr0_read_latency: 4,
        mr4_write_latency: 1,
        read_dummy_bits: 26,
        write_dummy_bits: 12,
        register_dummy_bits: 12,
    };

    /// Maximum 250 MHz OPI-DTR mode (500 MHz MPLL / 2).
    pub const MHZ_250: Self = Self {
        clock_mhz: 250,
        mr0_read_latency: 6,
        mr4_write_latency: 3,
        read_dummy_bits: 34,
        write_dummy_bits: 16,
        register_dummy_bits: 16,
    };
}

impl Default for PsramTiming {
    fn default() -> Self {
        Self::MHZ_200
    }
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

#[crate::ram]
pub(crate) fn init_psram(config: &mut PsramConfig) -> bool {
    let clocks = HP_SYS_CLKRST::regs();
    clocks.psram_ctrl0().modify(|_, w| {
        w.psram_sys_clk_en().set_bit();
        w.psram_pll_clk_en().set_bit();
        w.psram_core_clk_en().set_bit();
        unsafe {
            w.psram_clk_src_sel().bits(0);
            w.psram_core_clk_div_num().bits(0);
        }
        w
    });
    let mpll_mhz = match config.timing.clock_mhz {
        100 | 200 => 400,
        250 => 500,
        20 => 0,
        _ => return false,
    };
    if mpll_mhz != 0 && !configure_mpll(mpll_mhz) {
        return false;
    }
    clocks.psram_ctrl0().modify(|_, w| {
        w.psram_axi_rst_en().set_bit();
        w.psram_apb_rst_en().set_bit()
    });
    clocks.psram_ctrl0().modify(|_, w| {
        w.psram_axi_rst_en().clear_bit();
        w.psram_apb_rst_en().clear_bit()
    });

    if mpll_mhz != 0 {
        // DQS, pad drive, DLL and the MSPI2/MSPI3 clocks are prerequisites
        // for direct commands as well as cached accesses. Warm resets used
        // during bring-up preserved this state and hid the cold-boot ordering
        // requirement. This matches ESP-IDF's esp_psram_impl_enable order.
        prepare_psram_phy();
        configure_psram_clock(mpll_mhz / config.timing.clock_mhz);
    }

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
        dummy_bits: config.timing.register_dummy_bits,
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

    // Program fixed latency while direct commands still run at the ROM-safe
    // clock. MR0 and MR1 are returned as one little-endian pair.
    let mut mr01 = 0u32;
    address = 0;
    command.address = &mut address;
    command.rx_data = &mut mr01;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut command);
        esp_rom_spi_cmd_start(3, (&mut mr01 as *mut u32).cast(), 2, 1 << 1, false);
    }
    mr01 = (mr01 & 0xffff_ff00)
        | ((mr01 & 0xc0) | (1 << 5) | ((config.timing.mr0_read_latency as u32) << 2));
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
    // MR4 contains write latency. Preserve the reserved MR5 byte.
    let mut mr45 = 0u32;
    address = 4;
    command.address = &mut address;
    command.rx_data = &mut mr45;
    command.dummy_bits = config.timing.register_dummy_bits;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut command);
        esp_rom_spi_cmd_start(3, (&mut mr45 as *mut u32).cast(), 2, 1 << 1, false);
    }
    mr45 = (mr45 & 0xffff_ff1f) | ((config.timing.mr4_write_latency as u32) << 5);
    register_write.address = &mut address;
    register_write.tx_data = &mut mr45;
    unsafe {
        esp_rom_spi_cmd_config(3, &mut register_write);
        esp_rom_spi_cmd_start(3, core::ptr::null_mut(), 0, 1 << 1, false);
    }

    if mpll_mhz != 0 {
        let divider = mpll_mhz / config.timing.clock_mhz;
        if !tune_psram(&config.timing, mpll_mhz, divider) {
            return false;
        }
    }
    true
}

fn prepare_psram_phy() {
    let psram = PSRAM_MSPI::regs();
    psram
        .timing_cali()
        .modify(|_, w| w.dll_timing_cali().set_bit());
    psram
        .spi_smem_s_timing_cali()
        .modify(|_, w| w.spi_smem_s_dll_timing_cali().set_bit());
    const IOMUX_BASE: usize = 0x2058_4000;
    unsafe {
        for offset in [0x1c, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x40, 0x44] {
            let register = (IOMUX_BASE + offset) as *mut u32;
            register.write_volatile((register.read_volatile() & !(7 << 12)) | (2 << 12));
        }
        let dqs = (IOMUX_BASE + 0x3c) as *mut u32;
        dqs.write_volatile((dqs.read_volatile() & !(7 << 15)) | (2 << 15) | 1);
    }
}

fn configure_mpll(mpll_mhz: u32) -> bool {
    // ESP32-S31 powers the PSRAM PHY and its MPLL from adjustable LDO
    // channel 1. ESP-IDF reserves that channel at 1.8 V and waits 1 ms for
    // it to settle before enabling the MPLL. A warm reset leaves the rail
    // powered, which made this prerequisite easy to miss during bring-up.
    enable_psram_ldo();

    // MPLL = XTAL * (FB_DIV + 1) / (REF_DIV + 1).
    // Power up the MPLL and its analog-I2C domain before starting calibration.
    // ESP-IDF does this in rtc_clk_mpll_enable(), before
    // rtc_clk_mpll_configure(). Warm resets preserve these bits, so omitting
    // them only failed after a real power cycle.
    PMU::regs()
        .imm_hp_ck_power_1()
        .write(|w| unsafe { w.bits((1 << 22) | (1 << 26) | (1 << 30)) });
    unsafe {
        let hp_clock_control = 0x2058_9000 as *mut u32;
        hp_clock_control.write_volatile(hp_clock_control.read_volatile() | (1 << 31));
    }
    PMU::regs()
        .hp_active_hp_ck_power()
        .modify(|r, w| unsafe { w.bits(r.bits() | (1 << 26) | (1 << 30)) });
    unsafe {
        let analog_i2c_clock = (0x2010_f000 + 0x18) as *mut u32;
        analog_i2c_clock.write_volatile(analog_i2c_clock.read_volatile() | (1 << 2));
    }
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().clear_bit());
    LP_AON_CLK_RST::regs().mspi_div().modify(|_, w| unsafe {
        w.mspi_ref_div().bits(1);
        w.mspi_fb_div().bits((mpll_mhz * 2 / 40 - 1) as u8)
    });
    let mut timeout = 1_000_000;
    while !HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .read()
        .mspi_cal_end()
        .bit()
    {
        timeout -= 1;
        if timeout == 0 {
            return false;
        }
        core::hint::spin_loop();
    }
    HP_SYS_CLKRST::regs()
        .ana_pll_ctrl0()
        .modify(|_, w| w.mspi_cal_stop().set_bit());
    true
}

fn enable_psram_ldo() {
    let pmu = PMU::regs();

    // 1.8 V maps exactly to DREF=8 and MUL=4 according to
    // ldo_ll_voltage_to_dref_mul(). Limit inrush while the rail starts.
    pmu.ext_ldo_ctrl().modify(|_, w| w.ext_cur_lim().set_bit());
    pmu.ext_ldo_ctrl().modify(|_, w| unsafe {
        w.ext_ldo_tie_high().clear_bit();
        w.ext_ldo_dref().bits(8);
        w.ext_ldo_mul().bits(4);
        w.ext_ldo_en_vdet().set_bit()
    });
    pmu.psram_cfg().modify(|_, w| w.psram_xpd().set_bit());
    pmu.ext_ldo_ctrl()
        .modify(|_, w| w.ext_cur_lim().clear_bit());

    crate::rom::ets_delay_us(1_000);
}

fn configure_psram_clock(divider: u32) {
    let value = if divider == 1 {
        1 << 31
    } else {
        ((divider - 1) << 16) | ((divider / 2 - 1) << 8) | (divider - 1)
    };
    unsafe {
        HP_SYS_CLKRST::regs().psram_ctrl0().modify(|_, w| {
            w.psram_core_clk_en().set_bit();
            w.psram_sys_clk_en().set_bit();
            w.psram_clk_src_sel().bits(1);
            w.psram_core_clk_div_num().bits(0);
            w
        });
        PSRAM_MSPI::regs().sram_clk().write(|w| w.bits(value));
        // MSPI3 direct-command CLOCK register.
        (0x2050_3014 as *mut u32).write_volatile(value);
    }
}

const TUNING_WORDS: [u32; 32] = [
    0x7f78_6655,
    0xa5ff_005a,
    0x3f3c_33aa,
    0xa5ff_5a00,
    0x1f1e_9955,
    0xa500_5aff,
    0x0f0f_ccaa,
    0xa55a_00ff,
    0x0787_6655,
    0xffa5_5a00,
    0x03c3_33aa,
    0xff00_a55a,
    0x01e1_9955,
    0xff00_5aa5,
    0x00f0_ccaa,
    0xff5a_00a5,
    0x8078_6655,
    0x00a5_ff5a,
    0xc03c_33aa,
    0x00a5_5aff,
    0xe01e_9355,
    0x00ff_5aa5,
    0xf00f_ccaa,
    0x005a_ffa5,
    0xf887_6655,
    0x5aa5_ff00,
    0xfcc3_33aa,
    0x5aff_a500,
    0xfee1_9955,
    0x5a00_a5ff,
    0x11f0_ccaa,
    0x5a00_ffa5,
];

fn set_tuning(phase: u8, data_delay: u8, dqs_delay: u8) {
    const IOMUX_BASE: usize = 0x2058_4000;
    unsafe {
        // DQ0..DQ7 and CLK/CS use DLC bits 7:4.
        for offset in [0x1c, 0x20, 0x24, 0x28, 0x2c, 0x30, 0x34, 0x38, 0x40, 0x44] {
            let register = (IOMUX_BASE + offset) as *mut u32;
            let value = register.read_volatile();
            register.write_volatile((value & !(0xf << 4)) | ((data_delay as u32) << 4));
        }
        let register = (IOMUX_BASE + 0x3c) as *mut u32;
        let value = register.read_volatile();
        register.write_volatile(
            (value & !((3 << 1) | (0xf << 7) | (0xf << 18)))
                | ((phase as u32) << 1)
                | ((dqs_delay as u32) << 7)
                | ((dqs_delay as u32) << 18),
        );
    }
}

fn direct_read_matches(timing: &PsramTiming) -> bool {
    for chunk in 0..1u32 {
        let mut address = 0x80 + chunk * 64;
        let mut received = [0u32; 16];
        let mut command = RomSpiCommand {
            command: 0,
            command_bits: 16,
            address: &mut address,
            address_bits: 32,
            tx_data: core::ptr::null_mut(),
            tx_bits: 0,
            rx_data: received.as_mut_ptr(),
            rx_bits: 64 * 8,
            dummy_bits: timing.read_dummy_bits,
        };
        unsafe {
            esp_rom_spi_set_op_mode(3, 7);
            esp_rom_spi_cmd_config(3, &mut command);
            esp_rom_spi_cmd_start(3, received.as_mut_ptr().cast(), 64, 1 << 1, false);
        }
        if received != TUNING_WORDS[(chunk as usize) * 16..(chunk as usize + 1) * 16] {
            return false;
        }
    }
    true
}

fn write_tuning_reference(timing: &PsramTiming) {
    for chunk in 0..1u32 {
        let mut address = 0x80 + chunk * 64;
        let mut command = RomSpiCommand {
            command: 0x8080,
            command_bits: 16,
            address: &mut address,
            address_bits: 32,
            tx_data: TUNING_WORDS[(chunk as usize) * 16..].as_ptr() as *mut u32,
            tx_bits: 64 * 8,
            rx_data: core::ptr::null_mut(),
            rx_bits: 0,
            dummy_bits: timing.write_dummy_bits,
        };
        unsafe {
            esp_rom_spi_set_op_mode(3, 7);
            esp_rom_spi_cmd_config(3, &mut command);
            esp_rom_spi_cmd_start(3, core::ptr::null_mut(), 0, 1 << 1, false);
        }
    }
}

fn tune_psram(timing: &PsramTiming, mpll_mhz: u32, target_divider: u32) -> bool {
    // The reference survives the clock switch and is read through MSPI3,
    // bypassing the cache/MMU entirely.
    configure_psram_clock(mpll_mhz / 20);
    write_tuning_reference(timing);
    configure_psram_clock(target_divider);

    let mut best_phase = None;
    for phase in 0..4 {
        set_tuning(phase, 0, 0);
        if direct_read_matches(timing) {
            best_phase = Some(phase);
            break;
        }
    }
    let Some(phase) = best_phase else {
        return false;
    };

    let mut longest_start = 0u8;
    let mut longest_len = 0u8;
    let mut current_start = 0u8;
    let mut current_len = 0u8;
    for index in 0..31u8 {
        let (data, dqs) = if index < 16 {
            (0, 15 - index)
        } else {
            (index - 15, 0)
        };
        set_tuning(phase, data, dqs);
        let passes = direct_read_matches(timing);
        if passes {
            if current_len == 0 {
                current_start = index;
            }
            current_len += 1;
            if current_len > longest_len {
                longest_start = current_start;
                longest_len = current_len;
            }
        } else {
            current_len = 0;
        }
    }
    if longest_len < 2 {
        return false;
    }
    let best = longest_start + longest_len / 2;
    let (data, dqs) = if best < 16 {
        (0, 15 - best)
    } else {
        (best - 15, 0)
    };
    set_tuning(phase, data, dqs);
    direct_read_matches(timing)
}

pub(crate) fn map_psram(config: PsramConfig) -> Range<usize> {
    let size = config.size.get();
    if size == 0 {
        return 0..0;
    }

    unsafe {
        HP_SYS_CLKRST::regs().cache_ctrl0().modify(|_, w| {
            w.cpu_acache_cpu_clk_force_on().set_bit();
            w.rom_acache_mem_clk_force_on().set_bit();
            w.cpu_cache_cpu_clk_force_on().set_bit();
            w.mspi_cache_sys_clk_force_on().set_bit()
        });
        if !config.cache_already_initialized {
            ROM_Boot_Cache_Init();
        }

        let clocks = HP_SYS_CLKRST::regs();
        clocks.psram_ctrl0().modify(|_, w| {
            w.psram_core_clk_en().set_bit();
            w.psram_sys_clk_en().set_bit();
            w.psram_clk_src_sel().bits(1);
            w.psram_core_clk_div_num().bits(0);
            w
        });

        let psram = PSRAM_MSPI::regs();
        let mpll_mhz = if config.timing.clock_mhz == 250 {
            500
        } else {
            400
        };
        let divider = mpll_mhz / config.timing.clock_mhz;
        let clock_value = if divider == 1 {
            1 << 31
        } else {
            ((divider - 1) << 16) | ((divider / 2 - 1) << 8) | (divider - 1)
        };
        psram.sram_clk().write(|w| w.bits(clock_value));
        psram
            .spi_smem_s_timing_cali()
            .modify(|_, w| w.spi_smem_s_dll_timing_cali().set_bit());
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
                    | ((config.timing.read_dummy_bits - 1) << 6)
                    | (31 << 14)
                    | (1 << 20)
                    | (1 << 21)
                    | ((config.timing.write_dummy_bits - 1) << 22),
            )
        });
        psram.sram_cmd().modify(|r, w| {
            w.bits((r.bits() & !((0xf << 18) | (0x3 << 26))) | (0xf << 18) | (1 << 23))
        });
        psram
            .spi_smem_s_ddr()
            .modify(|r, w| w.bits((r.bits() & !0xf) | 0x3));
        psram
            .spi_smem_s_ecc_ctrl()
            .modify(|_, w| w.spi_smem_s_page_size().bits(3));
        psram
            .spi_smem_s_ac()
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

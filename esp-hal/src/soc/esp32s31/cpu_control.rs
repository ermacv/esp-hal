#[cfg(feature = "unstable")]
use crate::system::multi_core;
use crate::{
    peripherals::{HP_SYS_CLKRST, HP_SYSTEM, LP_AON_CLKRST, PMU},
    system::Cpu,
};

pub(crate) unsafe fn internal_park_core(core: Cpu, park: bool) {
    // Values specified by ESP-IDF cpu_utility_ll.h.
    let code: u8 = if park { 0x86 } else { 0xff };
    PMU::regs().cpu_stall_sw().modify(|_, w| unsafe {
        match core {
            Cpu::ProCpu => w.hpcore0_sw_stall_code().bits(code),
            Cpu::AppCpu => w.hpcore1_sw_stall_code().bits(code),
        }
    });

    if !park {
        let status = HP_SYSTEM::regs().hp_cpu_corestalled_st();
        match core {
            Cpu::ProCpu => while status.read().hp_reg_core0_corestalled_st().bit_is_set() {},
            Cpu::AppCpu => while status.read().hp_reg_core1_corestalled_st().bit_is_set() {},
        }
    }
}

#[instability::unstable]
pub fn is_running(core: Cpu) -> bool {
    let stall = PMU::regs().cpu_stall_sw().read();
    let code = match core {
        Cpu::ProCpu => stall.hpcore0_sw_stall_code().bits(),
        Cpu::AppCpu => stall.hpcore1_sw_stall_code().bits(),
    };
    code != 0x86
}

pub(crate) fn pre_system_reset() {
    // Reset Core 1 before retaining its PMU stall state across the system reset.
    LP_AON_CLKRST::regs()
        .lp_aonclkrst_hpcore1_reset_ctrl()
        .modify(|_, w| w.lp_aonclkrst_hpcore1_sw_reset().set_bit());
    unsafe { internal_park_core(Cpu::AppCpu, true) };
    crate::rom::ets_set_appcpu_boot_addr(0);
}

pub(crate) fn disable_core1() {
    HP_SYS_CLKRST::regs().hpcore1_ctrl0().modify(|_, w| {
        w.reg_core1_cpu_clk_en()
            .clear_bit()
            .reg_core1_clic_clk_en()
            .clear_bit()
            .reg_core1_global_rst_en()
            .set_bit()
    });
}

#[cfg(feature = "unstable")]
pub(crate) fn start_core1(entry_point: *const u32) {
    // This is the S31 sequence from IDF's
    // cpu_utility_ll_enable_clock_and_reset_app_cpu().
    HP_SYS_CLKRST::regs().hpcore1_ctrl0().modify(|_, w| {
        w.reg_core1_cpu_clk_en()
            .set_bit()
            .reg_core1_clic_clk_en()
            .set_bit()
            .reg_core1_global_rst_en()
            .clear_bit()
    });
    crate::rom::ets_set_appcpu_boot_addr(entry_point as u32);
}

/// Entry point reached directly from the Core 1 ROM polling loop.
#[unsafe(naked)]
#[cfg(feature = "unstable")]
pub(crate) extern "C" fn start_core1_init<F>() -> !
where
    F: FnOnce(),
{
    core::arch::naked_asm!(
        ".option push",
        ".option norelax",
        "la gp, __global_pointer$",
        ".option pop",
        // Enable and initialise the F extension before regular Rust executes.
        "li t0, 0x6000",
        "csrrs x0, mstatus, t0",
        "fscsr x0",
        "la t0, {stack_top}",
        "lw sp, 0(t0)",
        "j {init}",
        stack_top = sym multi_core::APP_CORE_STACK_TOP,
        init = sym start_core1_init_impl::<F>,
    )
}

#[cfg(feature = "unstable")]
fn start_core1_init_impl<F>() -> !
where
    F: FnOnce(),
{
    crate::soc::enable_branch_predictor();
    crate::soc::enable_external_memory_pma();
    crate::rom::ets_set_appcpu_boot_addr(0);

    unsafe {
        #[cfg(all(feature = "rt", stack_guard_monitoring))]
        {
            let guard =
                multi_core::APP_CORE_STACK_GUARD.load(core::sync::atomic::Ordering::Acquire);
            guard.write_volatile(esp_config::esp_config_int!(
                u32,
                "ESP_HAL_CONFIG_STACK_GUARD_VALUE"
            ));
            crate::debugger::set_stack_watchpoint(guard as usize);
        }
        crate::interrupt::init_vectoring();
        multi_core::CpuControl::start_core1_run::<F>()
    }
}

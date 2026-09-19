#[embedded_test::tests(default_timeout = 3)]
mod tests {
    use esp_hal::{
        Config,
        delay::Delay,
        rtc_cntl::{Rtc, RwdtStage},
        time::Duration,
    };
    #[cfg(timergroup_driver_supported)]
    use esp_hal::{
        clock::CpuClock,
        timer::timg::{MwdtStage, TimerGroup},
    };

    // A feed-only test also passes if the watchdog clock never runs. Observe
    // actual expiry without routing an interrupt to the CPU. This deliberately
    // tests the default XTAL boot profile, not other source selectors or reset.
    #[cfg(esp32s31)]
    fn check_s31_wdt_expiry<T: esp_hal::timer::timg::TimerGroupInstance>(
        group: &mut TimerGroup<'_, T>,
    ) {
        use esp_hal::timer::timg::MwdtStageAction;

        // SAFETY: this test owns the complete timer group and installs no ISR.
        let regs = unsafe { &*T::register_block() };
        let delay = Delay::new();
        regs.int_ena().modify(|_, w| w.wdt().clear_bit());
        regs.int_clr().write(|w| w.wdt().clear_bit_by_one());

        group
            .wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_secs(2));
        group.wdt.enable();
        group
            .wdt
            .set_stage_action(MwdtStage::Stage0, MwdtStageAction::Interrupt);
        group
            .wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_millis(100));
        group.wdt.feed();
        delay.delay(Duration::from_millis(25));
        let premature = regs.int_raw().read().wdt().bit_is_set();
        delay.delay(Duration::from_millis(150));
        let expired = regs.int_raw().read().wdt().bit_is_set();
        // Expiry advances to the disabled next stage. Feed first so a broken
        // disable cannot pass simply because stage 0 already finished.
        group.wdt.feed();
        group.wdt.disable();
        regs.int_clr().write(|w| w.wdt().clear_bit_by_one());
        delay.delay(Duration::from_millis(150));
        let expired_while_disabled = regs.int_raw().read().wdt().bit_is_set();

        // Re-enable must restore the clock and accept a new action/timeout;
        // enable intentionally resets the stage actions to their defaults.
        group
            .wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_secs(2));
        group.wdt.enable();
        group
            .wdt
            .set_stage_action(MwdtStage::Stage0, MwdtStageAction::Interrupt);
        group
            .wdt
            .set_timeout(MwdtStage::Stage0, Duration::from_millis(100));
        group.wdt.feed();
        delay.delay(Duration::from_millis(150));
        let rearmed = regs.int_raw().read().wdt().bit_is_set();
        group.wdt.disable();
        regs.int_clr().write(|w| w.wdt().clear_bit_by_one());

        // Quiesce before any assertion can panic into the debugger.
        assert!(!premature);
        assert!(
            expired,
            "watchdog did not expire after shortening its timeout"
        );
        assert!(!expired_while_disabled);
        assert!(rearmed, "watchdog did not expire after disable/re-enable");
    }

    #[test]
    #[cfg(esp32s31)]
    fn test_s31_timg0_wdt_expiry() {
        let p = esp_hal::init(Config::default());
        let mut group = TimerGroup::new(p.TIMG0);
        check_s31_wdt_expiry(&mut group);
        check_s31_wdt_clock_sources(&mut group);
    }

    #[test]
    #[cfg(esp32s31)]
    fn test_s31_timg1_wdt_expiry() {
        let p = esp_hal::init(Config::default());
        let mut group = TimerGroup::new(p.TIMG1);
        check_s31_wdt_expiry(&mut group);
        check_s31_wdt_clock_sources(&mut group);
    }

    #[cfg(esp32s31)]
    fn check_s31_wdt_clock_sources<T: esp_hal::timer::timg::TimerGroupInstance>(
        group: &mut TimerGroup<'_, T>,
    ) {
        use esp_hal::{
            clock::ll::{ClockTree, TimgWdtClockConfig},
            time::Instant,
            timer::timg::MwdtStageAction,
        };
        // SAFETY: the caller retains the complete timer group; no ISR is bound.
        let regs = unsafe { &*T::register_block() };
        for source in [
            TimgWdtClockConfig::PllF80m,
            TimgWdtClockConfig::RcFastClk,
            TimgWdtClockConfig::XtalClk,
        ] {
            ClockTree::with(|clocks| T::clock_instance().configure_wdt_clock(clocks, source));
            group
                .wdt
                .set_timeout(MwdtStage::Stage0, Duration::from_secs(2));
            group.wdt.enable();
            group
                .wdt
                .set_stage_action(MwdtStage::Stage0, MwdtStageAction::Interrupt);
            group
                .wdt
                .set_timeout(MwdtStage::Stage0, Duration::from_millis(100));
            let start = Instant::now();
            group.wdt.feed();
            while !regs.int_raw().read().wdt().bit_is_set()
                && start.elapsed() < Duration::from_millis(250)
            {
                core::hint::spin_loop();
            }
            let elapsed = start.elapsed().as_micros();
            let expired = regs.int_raw().read().wdt().bit_is_set();
            group.wdt.disable();
            regs.int_clr().write(|w| w.wdt().clear_bit_by_one());
            // RC_FAST is approximate; allow oscillator tolerance but reject a
            // missing or wrong mux selection. Assertions run only after stop.
            assert!(expired);
            assert!((90_000..=110_000).contains(&elapsed));
        }
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt() {
        let p = esp_hal::init(Config::default());

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;

        wdt0.set_timeout(MwdtStage::Stage0, Duration::from_millis(500));
        wdt0.enable();

        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt0.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_wdt0_uses_prescaler() {
        let p = esp_hal::init(Config::default());

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;

        // multiplied by 80 (for the default clock source), then taking the 32 lower bits this is
        // 0x40
        wdt0.set_timeout(MwdtStage::Stage0, Duration::from_millis(53_687_092));
        wdt0.enable();

        let delay = Delay::new();
        delay.delay(Duration::from_millis(250));

        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    #[cfg(timergroup_timg1)]
    fn test_feeding_timg1_wdt() {
        let p = esp_hal::init(Config::default());

        let timg1 = TimerGroup::new(p.TIMG1);
        let mut wdt1 = timg1.wdt;

        wdt1.set_timeout(MwdtStage::Stage0, Duration::from_millis(500));
        wdt1.enable();

        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt1.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt1.disable();
    }

    #[test]
    #[cfg(timergroup_timg0)]
    fn test_feeding_timg0_wdt_max_clock() {
        let p = esp_hal::init(Config::default().with_cpu_clock(CpuClock::max()));

        let timg0 = TimerGroup::new(p.TIMG0);
        let mut wdt0 = timg0.wdt;

        wdt0.set_timeout(MwdtStage::Stage0, Duration::from_millis(500));
        wdt0.enable();

        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            wdt0.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        wdt0.disable();
    }

    #[test]
    fn test_feeding_rtc_wdt() {
        let p = esp_hal::init(Config::default());

        let mut rtc = Rtc::new(p.RTC_TIMER);

        rtc.rwdt
            .set_timeout(RwdtStage::Stage0, Duration::from_millis(500));
        rtc.rwdt.enable();

        let delay = Delay::new();

        // Loop for more than the timeout of the watchdog.
        for _ in 0..6 {
            rtc.rwdt.feed();
            delay.delay(Duration::from_millis(100));
        }
        // Disable the watchdog, to prevent accidentally resetting the MCU while the host is setting
        // up the next test.
        rtc.rwdt.disable();
    }

    #[test]
    fn test_init_disables_watchdogs() {
        esp_hal::init(Config::default());

        let delay = Delay::new();
        delay.delay(Duration::from_millis(1000));
    }
}

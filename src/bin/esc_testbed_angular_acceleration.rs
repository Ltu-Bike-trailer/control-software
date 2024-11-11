//! Defines a simple test example for running the ESCs.
//!
//! This uses 3 pwm generators to drive the tansitors. It uses the drive pattern
//! specified by matlab on some page. It uses the hal effect sensors to detect
//! which state of the switching pattern we are in.
#![allow(
    warnings,
    dead_code,
    unused_variables,
    unreachable_code,
    internal_features
)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait, core_intrinsics)]
#![deny(clippy::all)]
#![deny(warnings)]

// PHASE ORDER: Yellow, blue green.

use controller as _; // global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [TIMER0,TIMER1]
)]
mod app {

    use controller::{
        bldc::{DrivePattern, FloatDuty, Pattern},
        cart,
        RingBuffer,
    };
    use cortex_m::asm;
    //use embedded_hal::digital::InputPin;
    use esc::{self, events::GpioEvents, PinConfig};
    use lib::protocol;
    use nrf52840_hal::{
        gpio::{Input, Pin, PullUp},
        pac::{PWM0, PWM1, PWM2, TIMER3},
        pwm::{self, Pwm},
        time::U32Ext,
        timer::OneShot,
    };
    use rtic_monotonics::{
        fugit::{ExtU32, ExtU64, Instant},
        Monotonic,
    };

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        sender: protocol::sender::Sender<10>,
        duty: f32,
        velocity: f32,
        current: f32,
        dvel: (u64, u64),
        pattern: Pattern,
        angle_acc: f32,
        target_davell: f32,
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
        events: esc::events::Manager,
        current_sense: esc::CurrentManager,
        hal_pins: [Pin<Input<PullUp>>; 3],
        phase1: Pwm<PWM0>,
        phase2: Pwm<PWM1>,
        phase3: Pwm<PWM2>,
        drive_pattern: DrivePattern,
        pattern_sender:
            rtic_sync::channel::Sender<'static, ((bool, bool), (bool, bool), (bool, bool)), 10>,
        pattern_receiver:
            rtic_sync::channel::Receiver<'static, ((bool, bool), (bool, bool), (bool, bool)), 10>,
        control_loop_timer: nrf52840_hal::timer::Timer<TIMER3, OneShot>,
        pid: controller::cart::controllers::MotorPidAlt,
        buffer: RingBuffer<f32, 10>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        nrf52840_hal::Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk();
        Mono::start(cx.device.RTC0);

        let p0 = nrf52840_hal::gpio::p0::Parts::new(cx.device.P0);
        let p1 = nrf52840_hal::gpio::p1::Parts::new(cx.device.P1);
        let ppi = nrf52840_hal::ppi::Parts::new(cx.device.PPI);

        let pins = PinConfig::new(p0, p1, ppi, cx.device.GPIOTE);
        let (pins, p1) = pins.configure_p1();
        let (pins, p2) = pins.configure_p2();
        let (pins, p3) = pins.configure_p3();
        let (pins, current_sense) = pins.configure_adc(cx.device.SAADC);
        //current_sense.start_sample();
        let events = pins.complete();
        let hal_pins = [p1.hal_effect, p2.hal_effect, p3.hal_effect];

        let phase1 = Pwm::new(cx.device.PWM0);
        phase1
            .set_output_pin(pwm::Channel::C0, p1.high_side)
            .set_output_pin(pwm::Channel::C1, p1.low_side)
            .set_period(10u32.khz().into())
            .set_duty(0.1);

        let phase2 = Pwm::new(cx.device.PWM1);
        phase2
            .set_output_pin(pwm::Channel::C0, p2.high_side)
            .set_output_pin(pwm::Channel::C1, p2.low_side)
            .set_period(10u32.khz().into())
            .set_duty(0.1);
        let phase3 = Pwm::new(cx.device.PWM2);
        phase3
            .set_output_pin(pwm::Channel::C0, p3.high_side)
            .set_output_pin(pwm::Channel::C1, p3.low_side)
            .set_period(10u32.khz().into())
            .set_duty(0.1);
        phase1.center_align();
        phase2.center_align();
        phase3.center_align();
        //  The order that we should drive the phases.
        let drive_pattern = DrivePattern::new();

        let sender = protocol::sender::Sender::new();
        debug_assert!(Mono::now().duration_since_epoch().to_micros() != 0);
        let (pattern_sender, pattern_receiver) =
            rtic_sync::make_channel!(((bool, bool), (bool, bool), (bool, bool)), 10);

        /*
        if unsafe { hal_pins[0].is_high().unwrap_unchecked() } {
            drive_pattern.set_a();
        }

        if unsafe { hal_pins[1].is_high().unwrap_unchecked() } {
            drive_pattern.set_b();
        }

        if unsafe { hal_pins[2].is_high().unwrap_unchecked() } {
            drive_pattern.set_c();
        }
        */
        // Spawn the tasks.
        let mut timer = nrf52840_hal::timer::Timer::new(cx.device.TIMER3);
        timer.enable_interrupt();
        let mut control_loop_timer = timer.into_oneshot();

        let mut pid = controller::cart::controllers::MotorPidAlt::new();
        pid.follow(1.);
        control_loop_timer.timeout::<1, { cart::controllers::MOTOR_TIMESCALE as u32 }>(
            cart::controllers::MOTOR_TS.micros().into(),
        );
        motor_driver::spawn().ok().unwrap();
        telemetry::spawn().ok().unwrap();
        //mono_sweep::spawn().ok().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
                sender,
                // ~.1 = MAX speed, ~.7 zero speed
                duty: 0.5,
                velocity: 0.,
                current: 0.,
                dvel: (0, 0),
                pattern: Default::default(),
                angle_acc: 0.,
                target_davell: 0.3,
            },
            Local {
                // Initialization of local resources go here
                events,
                current_sense,
                hal_pins,
                phase1,
                phase2,
                phase3,
                drive_pattern,
                pattern_receiver,
                pattern_sender,
                control_loop_timer,
                pid,
                buffer: RingBuffer::new([0.1, -0.05, 0.3, -0.1, 0.5, -0.15, 0.7, -0.2, 10., -1.]),
            },
        )
    }

    #[task(
        binds=GPIOTE,
        local=[
            events,
            hal_pins,
            t:Option<u64> = None,
            drive_pattern,
            pattern_sender,
            prev: (
                Option<Instant<u64,1,32_768>>,
                Option<Instant<u64,1,32_768>>,
                Option<Instant<u64,1,32_768>>
            ) = (
                None,
                None,
                None
            ),
            previous_avel:f32 = 0.,
            prev_vel_t: Option<Instant<u64,1,32_768>> = None,

        ],
        shared = [
            dvel,
            pattern,
            angle_acc,
        ],
        priority = 5
    )]
    /// Manages gpiote interrupts.
    ///
    /// This is every pin interrupt. So hal effect feedback, can bus messages
    /// and possibly more. the contents of this function will vary with the
    /// board we are using.
    ///
    /// ## Motor driving
    ///
    /// This function also drives the motors. This is the defualt case. The only
    /// other time that the motor should be driven externally is if the PID
    /// controller requests a new velocity.
    ///
    /// This ensures that the motor can be started from standstill.
    fn handle_gpio(mut cx: handle_gpio::Context) {
        let now = Mono::now();
        let mut dt = None;
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1HalRising => {
                    //defmt::info!("P1 hal rising");
                    cx.local.drive_pattern.set_a();
                    cx.local
                        .prev
                        .0
                        .replace(now.clone())
                        .map(|el| dt = Some(now - el));
                }
                GpioEvents::P1HalFalling => {
                    //defmt::info!("P2 hal falling");
                    cx.local.drive_pattern.clear_a();
                }
                GpioEvents::P2HalRising => {
                    //defmt::info!("P2 hal rising");
                    cx.local.drive_pattern.set_b();
                    cx.local
                        .prev
                        .1
                        .replace(now.clone())
                        .map(|el| dt = Some(now - el));
                }
                GpioEvents::P2HalFalling => {
                    cx.local.drive_pattern.clear_b();
                    //defmt::info!("P2 hal falling");
                }
                GpioEvents::P3HalRising => {
                    //defmt::info!("P3 hal rising");
                    cx.local.drive_pattern.set_c();
                    cx.local
                        .prev
                        .2
                        .replace(now.clone())
                        .map(|el| dt = Some(now - el));
                }
                GpioEvents::P3HalFalling => {
                    //defmt::info!("P3 hal falling");
                    cx.local.drive_pattern.clear_c();
                }
                GpioEvents::CAN => todo!("Handle CAN event"),
            }
        }
        //defmt::info!("Event");

        cx.shared
            .pattern
            .lock(|pattern| *pattern = cx.local.drive_pattern.get());

        if let Some(dt) = dt {
            let pt = cx.local.prev_vel_t.replace(now.clone());
            if pt.is_none() {
                return;
            }
            let pt = unsafe { pt.unwrap_unchecked() };

            let dt = dt.to_micros();
            let pt = (now - pt).to_micros();
            cx.shared.dvel.lock(|dvel| *dvel = (dt, pt));

            // TODO! Manage negative velocities.
            //defmt::info!("Angular acc {}", dvel);

            /*let prev = cx.local.previous_avel.clone();
            *cx.local.previous_avel = avel.clone();

            let dvel = (avel - prev) / dt;
            defmt::info!("{}", dvel);
            cx.shared.dvel.lock(|target| {
                *target = ((target.0 + dvel) / 2., now.duration_since_epoch().to_micros())
            });*/
        }
        // Trigger the phases inline since we want to run it as fast as
        // possible.
        //
        // Doing it this way ensures that we never miss a state, given that the
        // app is schedulable.
        //defmt::info!("Got hal effect interrupt!");
    }

    #[task(local = [phase1,phase2,phase3,pattern_receiver],shared = [duty,pattern],priority = 2)]
    /// Drives the phases of the motor.
    ///
    /// This is done by disabling the high/low side of each phase with respect
    /// to the switching pattern.
    ///
    /// This function should only be called by the PID controller.
    ///
    /// This ensures that the velocity can be set without dependance on the rate
    /// of interrupts.
    async fn motor_driver(cx: motor_driver::Context) {
        let (phase1, phase2, phase3) = (cx.local.phase1, cx.local.phase2, cx.local.phase3);
        let mut duty = 0.;
        let ((mut p1h, mut p1l), (mut p2h, mut p2l), (mut p3h, mut p3l)) = Default::default();
        let (mut shared_duty, mut pattern) = (cx.shared.duty, cx.shared.pattern);
        loop {
            let entry = Mono::now();
            loop {
                if Mono::now() - entry > 100u64.millis::<1, 16_000_000>() {
                    ((p1h, p1l), (p2h, p2l), (p3h, p3l)) = Default::default();
                    pattern.lock(|w| *w = Default::default());
                    break;
                }

                let (res, shared_duty) =
                    (&mut pattern, &mut shared_duty).lock(|w, duty| (w.clone(), duty.clone()));
                let res = res.get(shared_duty);
                duty = shared_duty;
                if res != ((p1h, p1l), (p2h, p2l), (p3h, p3l)) {
                    ((p1h, p1l), (p2h, p2l), (p3h, p3l)) = res;
                    break;
                }
                Mono::delay(50u64.micros()).await;
            }

            //(&mut drive_pattern, &mut duty).lock(|pattern, duty| (pattern.get(), *duty));
            /*defmt::info!(
                "Pattern Ah {}, Al {}, Bh {}, Bl {}, Ch {}, Cl {}",
                p1h,
                p1l,
                p2h,
                p2l,
                p3h,
                p3l
            );*/
            match (p1h, p1l) {
                (true, false) => {
                    phase1.disable_channel(pwm::Channel::C1);
                    asm::nop();
                    phase1.enable_channel(pwm::Channel::C0);
                }
                (false, true) => {
                    phase1.disable_channel(pwm::Channel::C0);
                    asm::nop();
                    phase1.enable_channel(pwm::Channel::C1);
                }
                (false, false) => {
                    phase1.disable_channel(pwm::Channel::C0);
                    asm::nop();
                    phase1.disable_channel(pwm::Channel::C1);
                }
                _ => {
                    phase1.disable_channel(pwm::Channel::C0);
                    phase1.disable_channel(pwm::Channel::C1);
                    phase2.disable_channel(pwm::Channel::C0);
                    phase2.disable_channel(pwm::Channel::C1);
                    phase3.disable_channel(pwm::Channel::C0);
                    phase3.disable_channel(pwm::Channel::C1);
                }
            };

            match (p2h, p2l) {
                (true, false) => {
                    phase2.disable_channel(pwm::Channel::C1);
                    asm::nop();
                    phase2.enable_channel(pwm::Channel::C0);
                }
                (false, true) => {
                    phase2.disable_channel(pwm::Channel::C0);
                    asm::nop();
                    phase2.enable_channel(pwm::Channel::C1);
                }
                (false, false) => {
                    phase2.disable_channel(pwm::Channel::C0);
                    asm::nop();
                    phase2.disable_channel(pwm::Channel::C1);
                }
                _ => {
                    phase1.disable_channel(pwm::Channel::C0);
                    phase1.disable_channel(pwm::Channel::C1);
                    phase2.disable_channel(pwm::Channel::C0);
                    phase2.disable_channel(pwm::Channel::C1);
                    phase3.disable_channel(pwm::Channel::C0);
                    phase3.disable_channel(pwm::Channel::C1);
                }
            };

            match (p3h, p3l) {
                (true, false) => {
                    phase3.disable_channel(pwm::Channel::C1);
                    phase3.enable_channel(pwm::Channel::C0);
                }
                (false, true) => {
                    phase3.disable_channel(pwm::Channel::C0);
                    phase3.enable_channel(pwm::Channel::C1);
                }
                (false, false) => {
                    phase3.disable_channel(pwm::Channel::C0);
                    phase3.disable_channel(pwm::Channel::C1);
                }
                _ => {
                    phase1.disable_channel(pwm::Channel::C0);
                    phase1.disable_channel(pwm::Channel::C1);
                    phase2.disable_channel(pwm::Channel::C0);
                    phase2.disable_channel(pwm::Channel::C1);
                    phase3.disable_channel(pwm::Channel::C0);
                    phase3.disable_channel(pwm::Channel::C1);
                }
            };
            // defmt::info!("Setting {}", duty);
            let duty = unsafe { core::intrinsics::fabsf32(duty) };
            phase1.set_duty(duty);
            phase2.set_duty(duty);
            phase3.set_duty(duty);
            /*duty *= 1.005;
            if duty >= 1. {
                duty = 1.;
            }*/
        }
    }

    #[task(shared = [duty], priority = 3)]
    async fn mono_sweep(mut cx: mono_sweep::Context) {
        loop {
            cx.shared.duty.lock(|duty| {
                *duty -= 0.1;
                if *duty <= 0.1 {
                    *duty = 0.1;
                }
                defmt::info!("Using {} duty cycle", *duty);
            });
            Mono::delay(10u32.secs().into()).await;
        }
    }
    #[task(binds = TIMER3,
        local = [
            pid,
            control_loop_timer,
            previous_avel: f32 = 0.,
            previous_dvel: f32 = 0.,
            prev: f32 = 0.,
            integral: f32 = 0.,
        ],
        shared = [
            duty,
            current,
            dvel,angle_acc,
            target_davell
        ],
        priority = 4,
    )]
    fn control_loop(mut cx: control_loop::Context) {
        //defmt::info!("Control loop");
        let start: rtic_monotonics::fugit::Instant<u64, 1, 32_768> = Mono::now();
        let timer = cx.local.control_loop_timer;
        timer.reset_event();
        // Create a MOTOR_TS duration in actual time.
        const DURATION: rtic_monotonics::fugit::Duration<u64, 1, 16000000> =
            rtic_monotonics::fugit::Duration::<
                u64,
                1,
                { controller::cart::controllers::MOTOR_TIMESCALE as u32 },
            >::from_ticks(controller::cart::controllers::MOTOR_TS as u64)
            .convert();
        let target = cx.shared.target_davell.lock(|t| *t);

        /*let current = cx.shared.current.lock(|c| {
            let ret = c.clone();
            *c = f32::NEG_INFINITY;
            ret
        });
        defmt::info!("Current : {} mA", current);*/
        let (dt, pt) = cx.shared.dvel.lock(|t| *t);
        /* defmt::info!("dt : {}, pt : {}",dt,pt); */
        let dt = dt as f32 / 1_000_000.;
        const FACTOR: f32 = core::f32::consts::TAU / 86.;
        let avel = FACTOR / dt;
        let dt = pt as f32 / 1_000_000.;
        let prev = cx.local.previous_avel.clone();
        *cx.local.previous_avel = avel.clone();
        let mut dvel = (avel - prev) / dt;

        //defmt::info!("Dvel {}", dvel);
        //let mut dvel = dvel.clamp(-50., 50.);
        if dvel.is_nan() || dvel.is_infinite() {
            dvel = 0.;
        }
        let dvel = dvel.clamp(-5., 5.);

        const KP: f32 = 100.;
        const KI: f32 = 10.;
        const KD: f32 = 1.;
        let del = dvel - *cx.local.previous_dvel;

        // TODO: Remove this ful-hack if at all possible.
        const RATE_LIMIT: f32 = 0.1;
        let dvel = match (del > RATE_LIMIT, del < RATE_LIMIT) {
            (true, false) => *cx.local.previous_dvel + RATE_LIMIT,
            (false, true) => *cx.local.previous_dvel - RATE_LIMIT,
            _ => dvel,
        };
        *cx.local.previous_dvel = dvel;
        let err = target - dvel;

        let p = (KP * err).clamp(-100., 100.);
        let i = (*cx.local.prev + err / 2.) * (DURATION.to_micros() as f32 / 1_000_000.);
        *cx.local.integral = (*cx.local.integral + i).clamp(-100., 100.);
        let i = KD * *cx.local.integral;
        let d = KD
            * ((err - *cx.local.prev) / (DURATION.to_micros() as f32 / 1_000_000.))
                .clamp(-100., 100.);
        *cx.local.prev = err;

        //defmt::info!("p {}, i {}, d {}", p, i, d);
        let actuation = p + i + d;

        const MIN_DUTY: f32 = -1.0;
        const MAX_DUTY: f32 = 0.95;
        const RANGE: f32 = MAX_DUTY - MIN_DUTY;
        let actuation = (((actuation + 300.) * RANGE) / 600.) + MIN_DUTY;
        let actuation = actuation.clamp(-1., 1.);
        // Target 500 mA
        // 1 Rad per second I guess.
        //cx.local.pid.register_measurement(dvel);
        // This is fine. The only time this throws an error is if the u16 fails to write
        // which it never does.
        //let actuation = unsafe { cx.local.pid.actuate().unwrap_unchecked() };
        /* defmt::info!("Actuation : {} / {}", actuation, 100.,); */
        cx.shared.duty.lock(|duty| *duty = actuation);

        cx.shared.angle_acc.lock(|acc| *acc = dvel);
        const RTC_DURATION: rtic_monotonics::fugit::Duration<u64, 1, 32768> =
            rtic_monotonics::fugit::Duration::<
                u64,
                1,
                { controller::cart::controllers::MOTOR_TIMESCALE as u32 },
            >::from_ticks(controller::cart::controllers::MOTOR_TS as u64)
            .convert();
        timer.timeout(unsafe {
            (start + RTC_DURATION)
                .checked_duration_since(Mono::now())
                .unwrap_unchecked()
        });
    }

    #[task(shared = [angle_acc,duty,target_davell], local=[buffer],priority = 2)]
    /// Drives the phases of the motor.
    ///
    /// This is done by disabling the high/low side of each phase with respect
    /// to the switching pattern.
    ///
    /// This function should only be called by the PID controller.
    ///
    /// This ensures that the velocity can be set without dependance on the rate
    /// of interrupts.
    async fn telemetry(cx: telemetry::Context) {
        let telemetry::SharedResources {
            angle_acc,
            duty,
            mut target_davell,
            __rtic_internal_marker,
        } = cx.shared;
        let mut rec = (angle_acc, duty);
        for reference in cx.local.buffer {
            let (acc, duty) = rec.lock(|acc, duty| (*acc, *duty));
            target_davell.lock(|t: &mut f32| *t = reference);
            defmt::info!("Acceleration : {} {}, following {}", acc, duty, reference);
            Mono::delay(5u64.secs()).await;
        }
    }

    #[task(binds = SAADC, local=[current_sense,n:i32 = 0],shared =[current],priority=1)]
    /// Continuously samples the current.
    ///
    /// If there is not recipient the value is merely averaged with the previous
    /// timestamp.
    fn current_sense(mut cx: current_sense::Context) {
        //defmt::info!("ZAMPLE :/");
        let sample = cx.local.current_sense.complete_sample();
        let mut current = 0.;
        if sample[0] > 0.01 {
            current += sample[0];
        }
        if sample[1] > 0.01 {
            current += sample[1];
        }
        if sample[2] > 0.01 {
            current += sample[2];
        }
        defmt::info!("Current : {:?}", sample);
        //let sample = sample[0].max(sample[1]).max(sample[2]);
        //defmt::info!("Sampled : {:?}",sample);
        cx.shared.current.lock(|target| {
            if *target == f32::NEG_INFINITY {
                *cx.local.n = 0;
                *target = current;
                return;
            }
            *target = target.max(current);
            /* *target = (*target * *cx.local.n as f32) + current;
             *cx.local.n += 1;
             *target /= *cx.local.n as f32; */
        });
        cx.local.current_sense.start_sample();
    }

    /// When ever the system is not running we should sample the current.
    #[idle(shared = [sender,duty])]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");
        loop {}
    }
}

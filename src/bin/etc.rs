//! Defines the firmware for the first iteration o the etc.
//!
//! This supports:
//!
//! - ~constant torque driving.
//! - regenerative breaking.
//! - Telemetry
//!
//! ## Constant torque driving.
//!
//! This comes with a small caveat, it is controlled with the angular
//! acceleration of the wheels rather than the input current to the system.
//! Doing this means that we control the cumulative torque at the surface rather
//! than the added torque from the motors.
//!
//! This has not been tested with a real system yet since we need to turn the
//! cart up side down, there is, however, no real reason to suspect that it
//! should not work.
//!
//! ## Regenerative breaking
//!
//! This works, the PID controller needs to be tuned to achieve reasonable
//! response times when the cart is loaded.
//!
//! ## Telemetry
//!
//! Currently this logs to the rtt but this should be changed to log the CAN bus
//! in the coming revisions.
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
mod etc {

    use controller::{
        bldc::{DrivePattern, FloatDuty, Pattern},
        cart,
        RingBuffer,
    };
    use cortex_m::asm;
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

    #[shared]
    /// Shared resources.
    struct Shared {
        sender: protocol::sender::Sender<10>,
        duty: f32,
        velocity: f32,
        current: f32,
        dvel: (u64, u64),
        pattern: Pattern,
        angle_acc: f32,
        target_davell: f32,
    }

    #[local]
    struct Local {
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

        // Configure phase driving pwms.
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

        // Spawn the tasks.
        let mut timer = nrf52840_hal::timer::Timer::new(cx.device.TIMER3);
        timer.enable_interrupt();
        let mut control_loop_timer = timer.into_oneshot();

        control_loop_timer.timeout::<1, { cart::constants::MOTOR_TIMESCALE as u32 }>(
            cart::constants::MOTOR_TS.micros().into(),
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
                buffer: RingBuffer::new([0.1, -0.05, 0.3, -0.1, 0.5, -0.15, 0.7, -0.2, 10., -1.]),
            },
        )
    }

    #[task(
        binds=GPIOTE,
        local=[
            // The event manager, this allows for easy iteration over the event enum.
            events,
            // The hall effect pins, these are unused but left for ease of use.
            hal_pins,
            // The pattern manager.
            drive_pattern,
            // The latest time measurements, one per hal effect interrupt.
            prev: (
                Option<Instant<u64,1,32_768>>,
                Option<Instant<u64,1,32_768>>,
                Option<Instant<u64,1,32_768>>
            ) = (
                None,
                None,
                None
            ),
            // The latest time for any rising edge interrupt.
            // 
            // This allows for angular acceleration calculations.
            prev_vel_t: Option<Instant<u64,1,32_768>> = None,

        ],
        shared = [
            // The values needed for derivative calculations.
            dvel,
            // The current drive pattern.
            pattern,
        ],
        priority = 5
    )]

    /// Manages gpiote interrupts.
    ///
    /// These interrupts can be
    /// - Hal effect
    /// - Can buss
    ///
    /// ## Hal effect
    ///
    /// If a hal effect interrupt is triggered the function measures all of the
    /// necessary values to be able to compute the angular velocity and sets
    /// the shared variables. Moreover, it gets the latest pattern and updates
    /// the shared pattern.
    ///
    /// ## Can buss
    ///
    /// TODO
    fn handle_gpio(mut cx: handle_gpio::Context) {
        let now = Mono::now();
        let mut dt = None;
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1HalRising => {
                    cx.local.drive_pattern.set_a();
                    cx.local
                        .prev
                        .0
                        .replace(now.clone())
                        .map(|el| dt = Some(now - el));
                }
                GpioEvents::P1HalFalling => {
                    cx.local.drive_pattern.clear_a();
                }
                GpioEvents::P2HalRising => {
                    cx.local.drive_pattern.set_b();
                    cx.local
                        .prev
                        .1
                        .replace(now.clone())
                        .map(|el| dt = Some(now - el));
                }
                GpioEvents::P2HalFalling => {
                    cx.local.drive_pattern.clear_b();
                }
                GpioEvents::P3HalRising => {
                    cx.local.drive_pattern.set_c();
                    cx.local
                        .prev
                        .2
                        .replace(now.clone())
                        .map(|el| dt = Some(now - el));
                }
                GpioEvents::P3HalFalling => {
                    cx.local.drive_pattern.clear_c();
                }
                GpioEvents::CAN => todo!("Handle CAN event"),
            }
        }

        cx.shared
            .pattern
            .lock(|pattern| *pattern = cx.local.drive_pattern.get());

        // Provide all of the needed measurements.
        if let Some(dt) = dt {
            let pt = cx.local.prev_vel_t.replace(now.clone());
            if pt.is_none() {
                return;
            }
            let pt = unsafe { pt.unwrap_unchecked() };

            let dt = dt.to_micros();
            let pt = (now - pt).to_micros();
            cx.shared.dvel.lock(|dvel| *dvel = (dt, pt));
        }
    }

    #[task(local = [phase1,phase2,phase3,pattern_receiver],shared = [duty,pattern],priority = 2)]
    /// Drives the phases of the motor.
    ///
    /// This is done by disabling the high/low side of each phase with respect
    /// to the switching pattern.
    async fn motor_driver(cx: motor_driver::Context) {
        // Initiate the state variables.
        let (phase1, phase2, phase3) = (cx.local.phase1, cx.local.phase2, cx.local.phase3);
        let mut duty = 0.;
        let ((mut p1h, mut p1l), (mut p2h, mut p2l), (mut p3h, mut p3l));
        let (mut shared_duty, mut pattern) = (cx.shared.duty, cx.shared.pattern);
        let mut old_pattern = Pattern::default();
        loop {
            let entry = Mono::now();
            // Block until we get a new write.
            // if nothing happens for 100 millis we simply kill all phases.
            loop {
                // DO NOT REMOVE THIS, IF YOU DO YOU KILL THE MOTORS.
                //
                // This ensures that the motor controller drops any voltage supplied to the
                // motor after 100 millis. If this happens the motor is static,
                // likely locked due to the orientation of the magnetic fields
                // we have nothing else to do aside from dropping the voltages to zero.
                if Mono::now() - entry > 100u64.millis::<1, 16_000_000>() {
                    ((p1h, p1l), (p2h, p2l), (p3h, p3l)) = Default::default();
                    pattern.lock(|w| *w = Default::default());
                    break;
                }

                // Check if we got a new control signal.
                let (pattern, shared_duty) =
                    (&mut pattern, &mut shared_duty).lock(|w, duty| (w.clone(), duty.clone()));

                // If we got a new control signal or if the motor shifted positions apply the
                // control signals again in the new pattern.
                if old_pattern != pattern || shared_duty != duty {
                    ((p1h, p1l), (p2h, p2l), (p3h, p3l)) = pattern.get(shared_duty);
                    duty = shared_duty;
                    old_pattern = pattern;
                    break;
                }
                // NOTE: This could likely be removed, increasing current consumption slightly
                // but improving the performance of the etc.
                Mono::delay(50u64.micros()).await;
            }
            // Apply the switching pattern.
            //
            // If any of the phases tries to kill the system we simply return early.
            'apply: {
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
                        break 'apply;
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
                        break 'apply;
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
                        break 'apply;
                    }
                };
            }
            // This is a bit faster.
            // The sign of the f32 is only relevant when we are setting the direction to
            // rotate, not while setting the duty cycles of the mosfets.
            // Any such control should be done before this.
            let duty = unsafe { core::intrinsics::fabsf32(duty) };
            phase1.set_duty(duty);
            phase2.set_duty(duty);
            phase3.set_duty(duty);
        }
    }

    #[task(shared = [duty], priority = 3)]
    /// NOTE: Left for completeness and testing purposes. This can be
    /// substituted for the control system to apply a specific duty cycle
    /// for a specific amount of time.
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
            // TIMER3 but as value.
            control_loop_timer,
            // Latest angular velocity.
            previous_avel: f32 = 0.,
            // Latest gradient in angular velocity.
            previous_dvel: f32 = 0.,
            // Previous error.
            prev: f32 = 0.,
            // Integral component accumulator.
            integral: f32 = 0.,
            // Counter for wether or not the cart control system should be allowed to run.
            started:u32 = 0
        ],
        shared = [
            // The target duty cycle.
            duty,
            // The latest current measurement.
            // This is unused but it is left for ease of porting.
            current,
            // The components needed for torque control.
            dvel,
            // Current angular acceleration. This allows for non intrusive logging.
            angle_acc,
            // The target gradient in angular velocity.
            target_davell
        ],
        priority = 4,
    )]
    /// Runs the PID control loop for constant torque.
    ///
    /// Due to performance requirements this was moved in to the function to
    /// greatly simplify the optimizations.
    fn control_loop(mut cx: control_loop::Context) {
        // This will miss by a few micro second, it is fine for our applications.
        let start: rtic_monotonics::fugit::Instant<u64, 1, 32_768> = Mono::now();
        let timer = cx.local.control_loop_timer;
        timer.reset_event();

        // Create a MOTOR_TS duration in actual time.
        const DURATION: rtic_monotonics::fugit::Duration<u64, 1, 16000000> =
            rtic_monotonics::fugit::Duration::<
                u64,
                1,
                { controller::cart::constants::MOTOR_TIMESCALE as u32 },
            >::from_ticks(controller::cart::constants::MOTOR_TS as u64)
            .convert();
        let target = cx.shared.target_davell.lock(|t| *t);

        let (dt, pt) = cx.shared.dvel.lock(|t| *t);
        let dt = dt as f32 / 1_000_000.;
        const FACTOR: f32 = core::f32::consts::TAU / 86.;
        let avel = FACTOR / dt;

        // Safe guards.

        // Do not run the PID control loop unless we have hade some velocity for a
        // while.
        {
            // This means that the cyclist will have to start the cart and then once it is
            // rolling we can start the control system.
            if unsafe { core::intrinsics::fabsf32(avel) } > 0.1 && *cx.local.started < 10 {
                *cx.local.started += 1;
            } else if unsafe { core::intrinsics::fabsf32(avel) } <= 0.1 && *cx.local.started >= 3 {
                *cx.local.started -= 1;
            } else {
                *cx.local.started = 0;
            }
            // To mitigate jerking motions we only apply control signals to the system
            if unsafe { core::intrinsics::fabsf32(avel) } < 0.1 || *cx.local.started < 3 {
                *cx.local.previous_avel = avel;
                const RTC_DURATION: rtic_monotonics::fugit::Duration<u64, 1, 32768> =
                    rtic_monotonics::fugit::Duration::<
                        u64,
                        1,
                        { controller::cart::constants::MOTOR_TIMESCALE as u32 },
                    >::from_ticks(controller::cart::constants::MOTOR_TS as u64)
                    .convert();
                // Ensure that we do not apply any control signal while the motor is not
                // rolling.
                cx.shared.duty.lock(|d| *d = 0.);
                timer.timeout(unsafe {
                    (start + RTC_DURATION)
                        .checked_duration_since(Mono::now())
                        .unwrap_unchecked()
                });
                return;
            }
        }

        // Control system.
        let dt = pt as f32 / 1_000_000.;
        let prev = cx.local.previous_avel.clone();
        *cx.local.previous_avel = avel.clone();
        let mut dvel = (avel - prev) / dt;

        // Remove undefined operations from the pid equations.
        // If the value is unbounded simply floor it to zero.
        if dvel.is_nan() || dvel.is_infinite() {
            dvel = 0.;
        }
        // TODO: Remove this once it is not needed any more.
        // When the cart is moving with any form of load this should no
        // longer be needed.
        let dvel = dvel.clamp(-5., 5.);

        // PID constants. These are defined here simply to be more readable.
        const KP: f32 = 100.;
        const KI: f32 = 10.;
        const KD: f32 = 1.;
        // Gradient in velocity.
        // This is a pretty good approximation of acceleration
        // and therefore torque.
        let del = dvel - *cx.local.previous_dvel;

        // TODO: Remove this ful-hack if at all possible.
        // This will likely not be needed once we have the cart right side up.
        const RATE_LIMIT: f32 = 0.1;
        let dvel = match (del > RATE_LIMIT, del < RATE_LIMIT) {
            (true, false) => *cx.local.previous_dvel + RATE_LIMIT,
            (false, true) => *cx.local.previous_dvel - RATE_LIMIT,
            _ => dvel,
        };
        *cx.local.previous_dvel = dvel;

        // Actual PID calculations.
        // These should not need a lot of modifications.
        let err = target - dvel;

        let p = (KP * err).clamp(-100., 100.);
        let i = (*cx.local.prev + err / 2.) * (DURATION.to_micros() as f32 / 1_000_000.);
        *cx.local.integral = (*cx.local.integral + i).clamp(-100., 100.);
        let i = KD * *cx.local.integral;
        let d = KD
            * ((err - *cx.local.prev) / (DURATION.to_micros() as f32 / 1_000_000.))
                .clamp(-100., 100.);
        *cx.local.prev = err;

        let actuation = p + i + d;

        // Ensure that we get a percentage.
        const MIN_DUTY: f32 = -1.0;
        const MAX_DUTY: f32 = 0.95;
        const RANGE: f32 = MAX_DUTY - MIN_DUTY;
        let actuation = (((actuation + 300.) * RANGE) / 600.) + MIN_DUTY;
        let actuation = actuation.clamp(-1., 1.);
        cx.shared.duty.lock(|duty| *duty = actuation);

        // Log the latest angular acceleration.
        cx.shared.angle_acc.lock(|acc| *acc = dvel);

        // Compute this once, no need to spend cycles on this.
        const RTC_DURATION: rtic_monotonics::fugit::Duration<u64, 1, 32768> =
            rtic_monotonics::fugit::Duration::<
                u64,
                1,
                { controller::cart::constants::MOTOR_TIMESCALE as u32 },
            >::from_ticks(controller::cart::constants::MOTOR_TS as u64)
            .convert();

        // Wait until the next control loop iteration.
        timer.timeout(unsafe {
            (start + RTC_DURATION)
                .checked_duration_since(Mono::now())
                .unwrap_unchecked()
        });
    }

    #[task(shared = [angle_acc,duty,target_davell], local=[buffer],priority = 2)]
    /// Sends telemetry data over the rtt channel at a fixed rate.
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

    // TODO: make this have a higher priority if we ever use it. As is, it will
    // never run.
    //
    // TODO: Make current_sense shared and sync the start_sample with the pwm
    // generator. This needs some form of rate limiting, we could disable the
    // pwm interrupt in the ISR and enable it in the motor driver, so if the
    // phase pattern changes we sample it once. for this we need the period of a
    // single pwm wave and ensure that we sample this. Or rather give ourself a
    // bit of a grace period on both edges.
    #[task(binds = SAADC, local=[current_sense,n:i32 = 0],shared =[current],priority=1)]
    /// Continuously samples the current.
    fn current_sense(mut cx: current_sense::Context) {
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
        cx.shared.current.lock(|target| {
            if *target == f32::NEG_INFINITY {
                *cx.local.n = 0;
                *target = current;
                return;
            }
            *target = current;
        });
        cx.local.current_sense.start_sample();
    }
}

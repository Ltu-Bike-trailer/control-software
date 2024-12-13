//! Defines the firmware for the ecc for the cart.
//!
//! This supports:
//!
//! - Logging torque over can
//! - Setting current over can
//! - PID based current control which is good enough.
//! - Current sensing done as non intrusive as possible.
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

use controller as _;
use rtic_monotonics::nrf::timer::prelude::*;
nrf_timer4_monotonic!(Mono, 16_000_000);

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [TIMER0,TIMER1,TIMER2]
)]
mod etc {

    use can_mcp2515::drivers::can::{AcceptanceFilterMask, CanInte, Mcp2515Driver};
    use controller::{
        bldc::{DrivePattern, FloatDuty, Pattern},
        cart::{
            self,
            constants::{DURATION, KD, KI, KP, MIN_DUTY, RANGE},
        },
        RingBuffer,
    };
    use esc::{self, events::GpioEvents, PinConfig};
    use lib::protocol::{self, message::CanMessage, MessageType, WriteType};
    use nrf52840_hal::{
        gpio::{p1::P1_08, Input, Output, Pin, PullUp, PushPull},
        pac::{PWM0, PWM1, PWM2, SPI0, TIMER3},
        pwm::{self, Pwm},
        spi::Spi,
        time::U32Ext,
        timer::OneShot,
    };
    //use paste::paste;
    use rtic_monotonics::{
        fugit::{ExtU32, ExtU64, Instant},
        Monotonic,
    };

    use crate::Mono;

    #[shared]
    /// Shared resources.
    struct Shared {
        sender: protocol::sender::Sender<1>,
        duty: f32,
        velocity: f32,
        current: f32,
        dvel: (u64, u64),
        pattern: Pattern,
        angle_acc: f32,
        target_davell: f32,
        current_sense: esc::CurrentManager,
    }

    #[local]
    struct Local {
        events: esc::events::Manager,
        hal_pins: [Pin<Input<PullUp>>; 3],
        phase1: Pwm<PWM0>,
        phase2: Pwm<PWM1>,
        phase3: Pwm<PWM2>,
        drive_pattern: DrivePattern,
        control_loop_timer: nrf52840_hal::timer::Timer<TIMER3, OneShot>,
        buffer: RingBuffer<f32, 15>,
        can: Mcp2515Driver<Spi<SPI0>, P1_08<Output<PushPull>>, Pin<Input<PullUp>>>,
        can_event_sender: rtic_sync::channel::Sender<'static, Option<CanMessage>, 10>,
        can_event_receiver: rtic_sync::channel::Receiver<'static, Option<CanMessage>, 10>,
        can_receive_sender: rtic_sync::channel::Sender<'static, CanMessage, 10>,
        can_receive_receiver: rtic_sync::channel::Receiver<'static, CanMessage, 10>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        defmt::trace!("Init: Clocks");
        nrf52840_hal::Clocks::new(cx.device.CLOCK)
            // The current board seems to have issues with the external oscillator.
            //.enable_ext_hfosc()
            .start_lfclk();
        defmt::trace!("Init: mono");
        Mono::start(cx.device.TIMER4);

        let p0 = nrf52840_hal::gpio::p0::Parts::new(cx.device.P0);
        let p1 = nrf52840_hal::gpio::p1::Parts::new(cx.device.P1);
        let ppi = nrf52840_hal::ppi::Parts::new(cx.device.PPI);

        defmt::trace!("Init: Pin config");
        let pins = PinConfig::new(p0, p1, ppi, cx.device.GPIOTE, cx.device.SPI0);
        defmt::trace!("Init: phase 1");
        let (pins, p1) = pins.configure_p1();
        defmt::trace!("Init: phase 2");
        let (pins, p2) = pins.configure_p2();
        defmt::trace!("Init: phase 3");
        let (pins, p3) = pins.configure_p3();
        defmt::trace!("Init: Adc");
        let (pins, current_sense) = pins.configure_adc(cx.device.SAADC);

        defmt::trace!("Init: SPI");
        let (pins, (spi, int_pin, cs)) = pins.configure_spi();
        defmt::trace!("Init: CAN driver");
        let can = can_mcp2515::drivers::can::Mcp2515Driver::init(
            spi,
            cs,
            int_pin,
            // Only accept messages for the left motor.
            can_mcp2515::drivers::can::Mcp2515Settings::default()
                .enable_interrupts(&[
                    CanInte::RX0IE,
                    CanInte::TX0IE,
                    CanInte::TX1IE,
                    CanInte::TX2IE,
                ])
                // All bits have to match a left motor message.
                .filter_b0(AcceptanceFilterMask::new(
                    0x7FF,
                    lib::protocol::constants::Message::SetMotorReference as u16,
                ))
                // All bits have to match 0x0
                .filter_b1(AcceptanceFilterMask::new(0x7FF, 0)),
        );
        defmt::trace!("Init: Event manager");
        let events = pins.complete();

        let hal_pins = [p1.hal_effect, p2.hal_effect, p3.hal_effect];

        // Configure phase driving pwms.
        defmt::trace!("Init: PWM controllers");
        let phase1 = Pwm::new(cx.device.PWM0);
        phase1
            .set_output_pin(pwm::Channel::C0, p1.high_side)
            .set_output_pin(pwm::Channel::C1, p1.low_side)
            .set_period(10u32.khz().into())
            .set_duty(0.0);

        let phase2 = Pwm::new(cx.device.PWM1);
        phase2
            .set_output_pin(pwm::Channel::C0, p2.high_side)
            .set_output_pin(pwm::Channel::C1, p2.low_side)
            .set_period(10u32.khz().into())
            .set_duty(0.0);
        let phase3 = Pwm::new(cx.device.PWM2);
        phase3
            .set_output_pin(pwm::Channel::C0, p3.high_side)
            .set_output_pin(pwm::Channel::C1, p3.low_side)
            .set_period(10u32.khz().into())
            .set_duty(0.0);
        phase1.enable_interrupt(pwm::PwmEvent::PwmPeriodEnd);
        phase1.center_align();
        phase2.center_align();
        phase3.center_align();

        //  The order that we should drive the phases.
        let drive_pattern = DrivePattern::new();

        let sender = protocol::sender::Sender::new();
        debug_assert!(Mono::now().duration_since_epoch().to_micros() != 0);

        // Spawn the tasks.

        defmt::trace!("Init: Control loop timer");
        let mut timer = nrf52840_hal::timer::Timer::new(cx.device.TIMER3);
        timer.enable_interrupt();
        let mut control_loop_timer = timer.into_oneshot();

        control_loop_timer.timeout::<1, { cart::constants::MOTOR_TIMESCALE as u32 }>(
            cart::constants::MOTOR_TS.micros().into(),
        );
        motor_driver::spawn().ok().unwrap();

        let (can_event_sender, can_event_receiver) =
            rtic_sync::make_channel!(Option<CanMessage>, 10);
        let (can_receive_sender, can_receive_receiver) = rtic_sync::make_channel!(CanMessage, 10);
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
                current_sense,
            },
            Local {
                // Initialization of local resources go here
                events,
                hal_pins,
                phase1,
                phase2,
                phase3,
                drive_pattern,
                control_loop_timer,
                buffer: RingBuffer::new([
                    0.5, 0.6, 0.7, 0.8, 0.9, 1., 1.5, 2.5, -0.1, -0.2, -2., 1.5, 0.2, 0.3, 0.4,
                ]),
                can,
                can_event_receiver,
                can_event_sender,
                can_receive_sender,
                can_receive_receiver,
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
                Option<Instant<u64,1,16_000_000>>,
                Option<Instant<u64,1,16_000_000>>,
                Option<Instant<u64,1,16_000_000>>
            ) = (
                None,
                None,
                None
            ),
            // The latest time for any rising edge interrupt.
            // 
            // This allows for angular acceleration calculations.
            prev_vel_t: Option<Instant<u64,1,16_000_000>> = None,
            can_event_sender,

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
    fn handle_gpio(mut cx: handle_gpio::Context) {
        let now = Mono::now();
        let mut dt = None;
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1HalRising => {
                    cx.local.drive_pattern.set_a();
                    dt = cx.local.prev.0.replace(now).map(|el| Some(now - el));
                }
                GpioEvents::P1HalFalling => {
                    cx.local.drive_pattern.clear_a();
                }
                GpioEvents::P2HalRising => {
                    cx.local.drive_pattern.set_b();
                    dt = cx.local.prev.1.replace(now).map(|el| Some(now - el));
                }
                GpioEvents::P2HalFalling => {
                    cx.local.drive_pattern.clear_b();
                }
                GpioEvents::P3HalRising => {
                    cx.local.drive_pattern.set_c();
                    dt = cx.local.prev.2.replace(now).map(|el| Some(now - el));
                }
                GpioEvents::P3HalFalling => {
                    cx.local.drive_pattern.clear_c();
                }
                GpioEvents::CAN => {
                    let _ = cx.local.can_event_sender.try_send(None);
                }
            }
        }

        cx.shared
            .pattern
            .lock(|pattern| *pattern = cx.local.drive_pattern.get());

        // Provide all of the needed measurements.
        if let Some(Some(dt)) = dt {
            let pt = cx.local.prev_vel_t.replace(now);
            if pt.is_none() {
                return;
            }
            let pt = unsafe { pt.unwrap_unchecked() };

            let dt = dt.to_micros();
            let pt = (now - pt).to_micros();
            cx.shared.dvel.lock(|dvel| *dvel = (dt, pt));
        }
    }

    #[task(local = [can,can_event_receiver], shared = [target_davell], priority = 3)]
    /// Manages CAN messages.
    async fn can(mut cx: can::Context) {
        let mut queue = lib::protocol::sender::Sender::new();
        while let Ok(message) = cx.local.can_event_receiver.recv().await {
            if message.is_none() {
                let events = cx.local.can.interrupt_manager();
                while let Some(event) = events.next() {
                    if let Some(message) = event.handle() {
                        let de: MessageType = match MessageType::try_from(&message) {
                            Ok(val) => val,
                            Err(_) => continue,
                        };
                        let (deadline, target) = match de {
                            MessageType::Write(WriteType::MotorReference { deadline, target }) => {
                                (Mono::now() + deadline.micros(), target)
                            }
                            _ => continue,
                        };
                        cx.shared.target_davell.lock(|duty| *duty = target);
                    }
                }
                if let Some(msg) = queue.dequeue() {
                    // It is totally fine if this fails here.
                    let _ = cx.local.can.transmit(&msg);
                } else {
                    // Grabs the 10 latest control messages from the buffer. This is safe since we
                    // will never collide with an ongoing write.
                    queue = unsafe { controller::util::DATA.get_n_latest::<10>() };
                }

                continue;
            }
            let message = unsafe { message.unwrap_unchecked() };

            let _ = cx.local.can.transmit(&message);
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
            previous_error: f32 = 0.,
            // Integral component accumulator.
            integral: f32 = 0.,
            // Counter for wether or not the cart control system should be allowed to run.
            started:u32 = 0,
            // The previous current
            current:f32 = 0.,
            kp:f32= 0.,
            ki:Option<f32> = None,
            kd:Option<f32> = None,
            loop_counter:usize = 0,
            buffer,
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
            target_davell,
        ],
        priority = 4,
    )]
    /// Runs the PID control loop for constant torque.
    ///
    /// Due to performance requirements this was moved in to the function to
    /// greatly simplify the optimizations.
    fn control_loop(mut cx: control_loop::Context) {
        // This will miss by a few nano seconds, it is fine for our applications.
        let start: rtic_monotonics::fugit::Instant<u64, 1, 16_000_000> = Mono::now();
        let control_loop_timer = cx.local.control_loop_timer;
        control_loop_timer.reset_event();

        let target = cx.shared.target_davell.lock(|t| *t);

        let current = cx.shared.current.lock(|current| {
            let ret = *current;
            ret
        });

        // TODO: Remove this ful-hack if at all possible.
        *cx.local.current = current;

        // Actual PID calculations.
        // These should not need a lot of modifications.
        let current_error = target - current;

        let p_component = (KP * current_error).clamp(-100., 100.);
        let i_component = (*cx.local.previous_error + current_error / 2.)
            * (DURATION.to_micros() as f32 / 1_000_000.);
        *cx.local.integral = (*cx.local.integral + i_component).clamp(-100., 100.);
        let i_component = KI * *cx.local.integral;
        let d_component = KD
            * ((current_error - *cx.local.previous_error)
                / (DURATION.to_micros() as f32 / 1_000_000.))
                .clamp(-100., 100.);
        *cx.local.previous_error = current_error;

        let actuation = p_component + i_component + d_component;

        // Ensure that we get a percentage.
        let actuation = (((actuation + 150.) * RANGE) / 300.) + MIN_DUTY;
        let actuation = actuation.clamp(-1., 1.);

        cx.shared.duty.lock(|duty| *duty = actuation);

        *cx.local.loop_counter = cx.local.loop_counter.wrapping_add(1);

        let time_to_sleep = (start + DURATION) - Mono::now();

        // Wait until the next control loop iteration.
        control_loop_timer.timeout(time_to_sleep);

        // Compute an intermediate representation of torque. This is no longer needed
        // but will be logger over the CAN bus for ease of debugging.
        let (delta_time, previous_delta_time) = cx.shared.dvel.lock(|t| *t);
        let delta_time = delta_time as f32 / 1_000_000.;
        const FACTOR: f32 = core::f32::consts::TAU / 86.;
        let angular_velocity = FACTOR / delta_time;

        let delta_time = previous_delta_time as f32 / 1_000_000.;
        let prev = *cx.local.previous_avel;
        *cx.local.previous_avel = angular_velocity;
        let angular_acceleration = (angular_velocity - prev) / delta_time;

        if *cx.local.loop_counter % 3 == 0 {
            let element = controller::util::ControlLog::new(
                current,
                angular_velocity,
                angular_acceleration,
                start.duration_since_epoch().to_micros(),
                target,
            );
            unsafe { controller::util::DATA.write(element) };
        }
    }

    /// Starts an adc sample.
    ///
    /// This ensures that we start sampling on start of the pwm signal.
    #[task(binds = PWM0, shared=[current_sense], priority = 3)]
    fn start_sample(mut cx: start_sample::Context) {
        cx.shared.current_sense.lock(|sense| {
            sense.start_sample();
        });

        // Disable interrupts until next time.
        nrf52840_hal::pac::NVIC::mask(nrf52840_hal::pac::interrupt::PWM0);
    }

    /// Samples all of the phases current on the rising edge of the pwm signal.
    ///
    /// This samples all of the currents but returns only the global system
    /// current as this is a smoother signal.
    #[task(binds = SAADC, shared = [current,current_sense], priority=3)]
    /// Continuously samples the current.
    fn current_sense(mut cx: current_sense::Context) {
        let sample = cx
            .shared
            .current_sense
            .lock(|sense| sense.complete_sample());

        cx.shared.current.lock(|current| {
            *current = (sample[3] + *current) / 2.;
        });
    }

    #[task(local = [phase1,phase2,phase3],shared = [duty,pattern],priority = 2)]
    #[inline(never)]
    /// Drives the phases of the motor.
    ///
    /// This is done by disabling the high/low side of each phase with respect
    /// to the switching pattern.
    async fn motor_driver(cx: motor_driver::Context) {
        // Initiate the state variables.
        let (phase1, phase2, phase3) = (cx.local.phase1, cx.local.phase2, cx.local.phase3);
        let mut duty = 0.;
        //let mut drive_pattern;
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
                    pattern.lock(|w| *w = Default::default());
                    break;
                }

                // Check if we got a new control signal.
                let (new_pattern, shared_duty) =
                    (&mut pattern, &mut shared_duty).lock(|w, duty| (*w, *duty));

                // If we got a new control signal or if the motor shifted positions apply the
                // control signals again in the new pattern.
                if old_pattern != new_pattern || shared_duty != duty {
                    //drive_pattern = new_pattern.get_u8(shared_duty);
                    duty = shared_duty;
                    old_pattern = new_pattern;
                    break;
                }
                // NOTE: This could likely be removed, increasing current
                // consumption slightly but improving the
                // performance of the etc.
                Mono::delay(50u64.micros()).await;
            }
            let ((p1h, p1l), (p2h, p2l), (p3h, p3l)) = old_pattern.get(duty);
            // Apply the switching pattern.
            //
            // If any of the phases tries to kill the system we simply return early.
            'apply: {
                match (p1h, p1l) {
                    (true, false) => {
                        phase1.disable_channel(pwm::Channel::C1);
                        phase1.enable_channel(pwm::Channel::C0);
                    }
                    (false, true) => {
                        phase1.disable_channel(pwm::Channel::C0);
                        phase1.enable_channel(pwm::Channel::C1);
                    }
                    (false, false) => {
                        phase1.disable_channel(pwm::Channel::C0);
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
                        phase2.enable_channel(pwm::Channel::C0);
                    }
                    (false, true) => {
                        phase2.disable_channel(pwm::Channel::C0);
                        phase2.enable_channel(pwm::Channel::C1);
                    }
                    (false, false) => {
                        phase2.disable_channel(pwm::Channel::C0);
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
            // Start sampling once the current sequence is done.
            unsafe { nrf52840_hal::pac::NVIC::unmask(nrf52840_hal::pac::interrupt::PWM0) };
            nrf52840_hal::pac::NVIC::unpend(nrf52840_hal::pac::interrupt::PWM0);
            phase1.set_duty(duty);
            phase2.set_duty(duty);
            phase3.set_duty(duty);
        }
    }
}

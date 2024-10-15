//! Defines a simple test example for running the ESCs.
//!
//! This uses 3 pwm generators to drive the tansitors. It uses the drive pattern
//! specified by matlab on some page. It uses the hal effect sensors to detect
//! which state of the switching pattern we are in.
#![allow(warnings, dead_code, unused_variables, unreachable_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

// PHASE ORDER: Yellow, blue green.

use controller as _; // global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [TIMER0]
)]
mod app {

    use controller::bldc::{DrivePattern, FloatDuty};
    use cortex_m::asm;
    //use embedded_hal::digital::InputPin;
    use esc::{self, events::GpioEvents, PinConfig};
    use foc::{pwm::SpaceVector, Foc};
    use lib::protocol;
    use nrf52840_hal::{
        gpio::{Input, Pin, PullUp},
        pac::{PWM0, PWM1, PWM2},
        pwm::{self, Pwm},
        time::U32Ext,
    };
    use rtic_monotonics::Monotonic;
    use rtic_sync::make_channel;

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        drive_pattern: DrivePattern,
        sender: protocol::sender::Sender<10>,
        duty: f32,
        velocity: f32,
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
        events: esc::events::Manager,
        current_sense: esc::CurrentManager,

        current_sender: rtic_sync::channel::Sender<'static, [f32; 3], 100>,
        current_receiver: rtic_sync::channel::Receiver<'static, [f32; 3], 100>,
        hal_pins: [Pin<Input<PullUp>>; 3],
        foc_sender: rtic_sync::channel::Sender<'static, u8, 100>,
        foc_receiver: rtic_sync::channel::Receiver<'static, u8, 100>,
        foc_manager: Foc<SpaceVector, { 1 << 15 }>,
        phase1: Pwm<PWM0>,
        phase2: Pwm<PWM1>,
        phase3: Pwm<PWM2>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        nrf52840_hal::Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk();
        Mono::start(cx.device.RTC0);

        let foc_manager = foc::Foc::<SpaceVector, { 1 << 15 }>::new(
            foc::pid::PIController::new(
                fixed::FixedI32::from_num(10),
                fixed::FixedI32::from_num(2),
            ),
            foc::pid::PIController::new(
                fixed::FixedI32::from_num(10),
                fixed::FixedI32::from_num(2),
            ),
        );
        let p0 = nrf52840_hal::gpio::p0::Parts::new(cx.device.P0);
        let p1 = nrf52840_hal::gpio::p1::Parts::new(cx.device.P1);
        let ppi = nrf52840_hal::ppi::Parts::new(cx.device.PPI);

        let pins = PinConfig::new(p0, p1, ppi, cx.device.GPIOTE);
        let (pins, p1) = pins.configure_p1();
        let (pins, p2) = pins.configure_p2();
        let (pins, p3) = pins.configure_p3();
        let (pins, current_sense) = pins.configure_adc(cx.device.SAADC);
        let events = pins.complete();
        let hal_pins = [p1.hal_effect, p2.hal_effect, p3.hal_effect];

        let phase1 = Pwm::new(cx.device.PWM0);
        phase1
            .set_output_pin(pwm::Channel::C0, p1.high_side)
            .set_output_pin(pwm::Channel::C1, p1.low_side)
            .set_period(100u32.khz().into())
            .set_duty(0.1);
        phase1.center_align();

        let phase2 = Pwm::new(cx.device.PWM1);
        phase2
            .set_output_pin(pwm::Channel::C0, p2.high_side)
            .set_output_pin(pwm::Channel::C1, p2.low_side)
            .set_period(1000u32.khz().into())
            .set_duty(0.1);
        phase2.center_align();
        let phase3 = Pwm::new(cx.device.PWM2);
        phase3
            .set_output_pin(pwm::Channel::C0, p3.high_side)
            .set_output_pin(pwm::Channel::C1, p3.low_side)
            .set_period(100u32.khz().into())
            .set_duty(0.1);
        phase3.center_align();

        //  The order that we should drive the phases.
        let drive_pattern = DrivePattern::new();

        let sender = protocol::sender::Sender::new();
        debug_assert!(Mono::now().duration_since_epoch().to_micros() != 0);

        let (foc_sender, foc_receiver) = make_channel!(u8, 100);
        let (current_sender, current_receiver) = make_channel!([f32; 3], 100);
        // Spawn the tasks.
        motor_driver::spawn().ok().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
                drive_pattern,
                sender,
                duty: 0.5,
                velocity: 0.,
            },
            Local {
                // Initialization of local resources go here
                events,
                current_sense,
                hal_pins,
                foc_receiver,
                foc_sender,
                foc_manager,
                phase1,
                phase2,
                phase3,
                current_receiver,
                current_sender,
            },
        )
    }

    #[task(binds=GPIOTE, local=[events,hal_pins,t:Option<u64> = None,foc_sender,state: [bool;3] = [false;3]],shared = [drive_pattern], priority = 5)]
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
        cx.shared.drive_pattern.lock(|pattern| {
            for event in cx.local.events.events() {
                match event {
                    GpioEvents::P1HalRising => pattern.set_a(),
                    GpioEvents::P1HalFalling => pattern.clear_a(),
                    GpioEvents::P2HalRising => pattern.set_b(),
                    GpioEvents::P2HalFalling => pattern.clear_b(),
                    GpioEvents::P3HalRising => pattern.set_c(),
                    GpioEvents::P3HalFalling => pattern.clear_c(),
                    GpioEvents::CAN => todo!("Handle CAN event"),
                }
            }

            // We are the only sender so we will always acquire the lock.
            let _ = cx.local.foc_sender.try_send(pattern.get_state());
        });
    }

    #[task(local = [foc_manager,phase1,phase2,phase3,foc_receiver,current_receiver,current_sense],priority = 2)]
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
        let foc = cx.local.foc_manager;
        let (p1, p2, p3) = (cx.local.phase1, cx.local.phase2, cx.local.phase3);
        let mut prev_time = Mono::now();
        let mut angle: fixed::types::I16F16 = fixed::types::I16F16::default();
        loop {
            if let Ok(val) = cx.local.foc_receiver.try_recv() {
                defmt::info!("SPINNING {}", val);
                let intermediate: fixed::types::I16F16 =
                    fixed::types::I16F16::from_num(32. * 360. / core::f32::consts::PI);
                angle = match val {
                    0b110 => fixed::types::I16F16::from_num(32. * 180. / core::f32::consts::PI),
                    0b100 => fixed::types::I16F16::from_num(96. * 180. / core::f32::consts::PI),
                    0b101 => fixed::types::I16F16::from_num(160. * 180. / core::f32::consts::PI),
                    0b001 => fixed::types::I16F16::from_num(224. * 180. / core::f32::consts::PI),
                    0b011 => fixed::types::I16F16::from_num(288. * 180. / core::f32::consts::PI),
                    0b010 => fixed::types::I16F16::from_num(352. * 180. / core::f32::consts::PI),
                    _val => fixed::types::I16F16::max(
                        fixed::types::I16F16::TAU,
                        angle + fixed::types::I16F16::from_num(0.1),
                    ),
                };
            } else {
                angle = fixed::types::I16F16::max(
                    fixed::types::I16F16::TAU,
                    angle + fixed::types::I16F16::from_num(0.1),
                );
            }
            // Make this async to allow other tasks to urn in the mean time.
            let current = match cx.local.current_sense.sample().await {
                Ok([c1, c2, _c3]) => [
                    fixed::types::I16F16::from_num(c1),
                    fixed::types::I16F16::from_num(c2),
                ],
                Err(_) => continue,
            };
            let time = Mono::now();

            let [pa, pb, pc] = foc.update(
                current,
                angle,
                // The power target in amperes.
                fixed::types::I16F16::from_num(3),
                fixed::types::I16F16::from_num(
                    unsafe { time.checked_duration_since(prev_time).unwrap_unchecked() }
                        .to_micros(),
                ),
            );
            prev_time = time;

            const AVG: u16 = u16::MAX;
            if pa > AVG {
                p1.enable_channel(pwm::Channel::C0);
                p1.disable_channel(pwm::Channel::C1);
                let pa = pa - AVG;
            } else {
                let pa = AVG - pa;
                p1.enable_channel(pwm::Channel::C1);
                p1.disable_channel(pwm::Channel::C0);
            }

            if pb > AVG {
                p2.enable_channel(pwm::Channel::C0);
                p2.disable_channel(pwm::Channel::C1);
                let pb = pb - AVG;
            } else {
                let pb = AVG - pb;
                p2.enable_channel(pwm::Channel::C1);
                p2.disable_channel(pwm::Channel::C0);
            }

            if pc > AVG {
                p3.enable_channel(pwm::Channel::C0);
                p3.disable_channel(pwm::Channel::C1);
                let pc = pc - AVG;
            } else {
                let pc = AVG - pc;
                p3.enable_channel(pwm::Channel::C1);
                p3.disable_channel(pwm::Channel::C0);
            }

            // Apply changes.
            p1.enable();
            p1.set_duty(pa as f32 / AVG as f32);
            p2.enable();
            p2.set_duty(pb as f32 / AVG as f32);
            p3.enable();
            p3.set_duty(pc as f32 / AVG as f32);
        }
    }

    /// When ever the system is not running we should sample the current.
    #[idle(shared = [sender])]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");
        loop {
            asm::wfi();
        }
    }
}

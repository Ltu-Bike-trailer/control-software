//! Defines a simple test example for running the ESCs.
//!
//! This uses 3 pwm generators to drive the tansitors. It uses the drive pattern
//! specified by matlab on some page. It uses the hal effect sensors to detect
//! which state of the switching pattern we are in.

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [TIMER0]
)]
mod app {

    use controller::bldc::{DrivePattern, FloatDuty};
    use embedded_hal::digital::InputPin;
    use esc::{self, events::GpioEvents, PinConfig};
    use lib::protocol;
    use nrf52840_hal::{
        gpio::{Input, Pin, PullUp},
        pac::{PWM0, PWM1, PWM2},
        pwm::{self, Pwm},
        time::U32Ext,
    };
    use rtic_monotonics::{fugit::ExtU64, Monotonic};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        drive_pattern: DrivePattern,
        sender: protocol::sender::Sender<10>,
        duty: f32,
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
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        Mono::start(cx.device.RTC0);

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
            .set_period(1u32.mhz().into())
            .set_duty(0.1);

        let phase2 = Pwm::new(cx.device.PWM1);
        phase2
            .set_output_pin(pwm::Channel::C0, p2.high_side)
            .set_output_pin(pwm::Channel::C1, p2.low_side)
            .set_period(1u32.mhz().into())
            .set_duty(0.1);
        let phase3 = Pwm::new(cx.device.PWM2);
        phase3
            .set_output_pin(pwm::Channel::C0, p3.high_side)
            .set_output_pin(pwm::Channel::C1, p3.low_side)
            .set_period(1u32.mhz().into())
            .set_duty(0.1);

        //  The order that we should drive the phases.
        let drive_pattern = DrivePattern::new();

        let sender = protocol::sender::Sender::new();

        // Spawn the tasks.
        sense::spawn().ok().unwrap();
        motor_driver::spawn().ok().unwrap();
        (
            Shared {
                // Initialization of shared resources go here
                drive_pattern,
                sender,
                duty: 0.2,
            },
            Local {
                // Initialization of local resources go here
                events,
                current_sense,
                hal_pins,
                phase1,
                phase2,
                phase3,
            },
        )
    }

    #[task(binds=GPIOTE, local=[events,hal_pins],shared = [drive_pattern], priority = 5)]
    /// Manages gpiote interrupts.
    ///
    /// This is every pin interrupt. So hal effect feedback, can bus messages
    /// and possibly more. the contents of this function will vary with the
    /// board we are using.
    fn handle_gpio(mut cx: handle_gpio::Context) {
        // Docs says that it is infallible.
        let edge =
            |pin: &mut Pin<Input<PullUp>>| -> bool { unsafe { pin.is_high().unwrap_unchecked() } };
        let mut set_phase = false;
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1Hal => {
                    cx.shared
                        .drive_pattern
                        .lock(|pattern| match edge(&mut cx.local.hal_pins[0]) {
                            true => pattern.set_a(),
                            false => pattern.clear_a(),
                        });
                    set_phase |= true;
                }
                GpioEvents::P2Hal => {
                    cx.shared
                        .drive_pattern
                        .lock(|pattern| match edge(&mut cx.local.hal_pins[0]) {
                            true => pattern.set_b(),
                            false => pattern.clear_b(),
                        });
                    set_phase |= true;
                }
                GpioEvents::P3Hal => {
                    cx.shared
                        .drive_pattern
                        .lock(|pattern| match edge(&mut cx.local.hal_pins[0]) {
                            true => pattern.set_c(),
                            false => pattern.clear_c(),
                        });
                    set_phase |= true;
                }
                GpioEvents::CAN => todo!("Handle CAN event"),
            }
        }

        if set_phase {
            defmt::info!("Got hal effect interrupt!");
        }
    }

    #[task(local=[current_sense],shared = [sender],priority = 2)]
    /// Measures the current consumption at the current timestamp.
    ///
    /// TODO: Rewrite the hal implementation of this to not be blocking so we
    /// can sample faster.
    async fn sense(mut cx: sense::Context) {
        loop {
            let t = Mono::now();
            let measurement = match cx.local.current_sense.sample() {
                Ok(val) => val,
                Err(_) => continue,
            };
            let _ = cx.shared.sender.lock(|sender| {
                sender.set_current_sense_left((t.duration_since_epoch().to_millis(), measurement))
            });
            // Poll each 10 ms.
            Mono::delay_until(t + 10u64.millis()).await;
        }
    }

    #[task(shared = [drive_pattern,duty],local = [phase1,phase2,phase3],priority = 2)]
    /// Drives the phases of the motor.
    ///
    /// This is done by disabling the high/low side of each phase with respect
    /// to the switching pattern.
    async fn motor_driver(cx: motor_driver::Context) {
        let (phase1, phase2, phase3) = (cx.local.phase1, cx.local.phase2, cx.local.phase3);
        let (mut drive_pattern, mut duty) = (cx.shared.drive_pattern, cx.shared.duty);
        loop {
            let t = Mono::now();

            let ((p1, p2, p3), duty) =
                (&mut drive_pattern, &mut duty).lock(|pattern, duty| (pattern.get(), *duty));
            defmt::trace!("Pattern A {}, B {}, C {}", p1, p2, p3);
            match p1 {
                true => phase1
                    .enable_channel(pwm::Channel::C0)
                    .disable_channel(pwm::Channel::C1),
                false => phase1
                    .enable_channel(pwm::Channel::C1)
                    .disable_channel(pwm::Channel::C0),
            };
            match p2 {
                true => phase2
                    .enable_channel(pwm::Channel::C0)
                    .disable_channel(pwm::Channel::C1),
                false => phase2
                    .enable_channel(pwm::Channel::C1)
                    .disable_channel(pwm::Channel::C0),
            };
            match p3 {
                true => phase3
                    .enable_channel(pwm::Channel::C0)
                    .disable_channel(pwm::Channel::C1),
                false => phase3
                    .enable_channel(pwm::Channel::C1)
                    .disable_channel(pwm::Channel::C0),
            };
            phase1.set_duty(duty);
            phase2.set_duty(duty);
            phase3.set_duty(duty);

            // Poll each 10 us.
            Mono::delay_until(t + 10u64.micros()).await;
        }
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
}

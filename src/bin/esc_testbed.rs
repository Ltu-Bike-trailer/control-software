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

use controller as _; // global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);







#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [TIMER0]
)]
mod app {

    use controller::{
        bldc::{DrivePattern, FloatDuty},
        cart::constants::CURRENT_SAMPLE_RATE,
    };
    use cortex_m::asm;
    //use embedded_hal::digital::InputPin;
    use esc::{self, events::GpioEvents, PinConfig};
    use lib::protocol;
    use nrf52840_hal::{
        gpio::{Input, Pin, PullUp},
        pac::{PWM0, PWM1, PWM2},
        pwm::{self, Pwm},
        time::U32Ext,
    };
    use rtic_monotonics::Monotonic;

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        drive_pattern: DrivePattern,
        sender: protocol::sender::Sender<10>,
        duty: f32,
        phase1: Pwm<PWM0>,
        phase2: Pwm<PWM1>,
        phase3: Pwm<PWM2>,
        velocity: f32,
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
        events: esc::events::Manager,
        current_sense: esc::CurrentManager,
        hal_pins: [Pin<Input<PullUp>>; 3],
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
        let events = pins.complete();
        let hal_pins = [p1.hal_effect, p2.hal_effect, p3.hal_effect];

        let phase1 = Pwm::new(cx.device.PWM0);
        phase1
            .set_output_pin(pwm::Channel::C0, p1.high_side)
            .set_output_pin(pwm::Channel::C1, p1.low_side)
            .set_period(100u32.khz().into())
            .set_duty(0.1);

        let phase2 = Pwm::new(cx.device.PWM1);
        phase2
            .set_output_pin(pwm::Channel::C0, p2.high_side)
            .set_output_pin(pwm::Channel::C1, p2.low_side)
            .set_period(1000u32.khz().into())
            .set_duty(0.1);
        let phase3 = Pwm::new(cx.device.PWM2);
        phase3
            .set_output_pin(pwm::Channel::C0, p3.high_side)
            .set_output_pin(pwm::Channel::C1, p3.low_side)
            .set_period(100u32.khz().into())
            .set_duty(0.1);
        //  The order that we should drive the phases.
        let drive_pattern = DrivePattern::new();

        let sender = protocol::sender::Sender::new();
        debug_assert!(Mono::now().duration_since_epoch().to_micros() != 0);
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
        motor_driver::spawn().ok().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
                drive_pattern,
                sender,
                duty: 0.5,
                phase1,
                phase2,
                phase3,
                velocity: 0.,
            },
            Local {
                // Initialization of local resources go here
                events,
                current_sense,
                hal_pins,
            },
        )
    }

    #[task(binds=GPIOTE, local=[events,hal_pins,t:Option<u64> = None],shared = [drive_pattern,phase1,phase2,phase3,duty,velocity], priority = 5)]
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
        let t = Mono::now();
        let mut set_phase = false;
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1HalRising => {
                    defmt::info!("P1 hal rising");
                    cx.shared.drive_pattern.lock(|pattern| pattern.set_a());
                    set_phase = true;
                }
                GpioEvents::P1HalFalling => {
                    defmt::info!("P2 hal falling");
                    cx.shared.drive_pattern.lock(|pattern| pattern.clear_a());
                    set_phase = true;
                }
                GpioEvents::P2HalRising => {
                    defmt::info!("P2 hal rising");
                    cx.shared.drive_pattern.lock(|pattern| pattern.set_b());
                    set_phase = true;
                }
                GpioEvents::P2HalFalling => {
                    defmt::info!("P2 hal falling");
                    cx.shared.drive_pattern.lock(|pattern| pattern.clear_b());
                    set_phase = true;
                }
                GpioEvents::P3HalRising => {
                    defmt::info!("P3 hal rising");
                    cx.shared.drive_pattern.lock(|pattern| pattern.set_c());
                    set_phase = true;
                }
                GpioEvents::P3HalFalling => {
                    defmt::info!("P3 hal falling");
                    cx.shared.drive_pattern.lock(|pattern| pattern.clear_c());
                    set_phase = true;
                }
                GpioEvents::CAN => todo!("Handle CAN event"),
            }
        }

        if !set_phase {
            return;
        }

        // Trigger the phases inline since we want to run it as fast as possible.
        //
        // Doing it this way ensures that we never miss a state, given that the app i
        // schedulable.
        defmt::info!("Got hal effect interrupt!");
        let (mut drive_pattern, mut duty) = (cx.shared.drive_pattern, cx.shared.duty);

        let (((p1h, p1l), (p2h, p2l), (p3h, p3l)), duty) =
            (&mut drive_pattern, &mut duty).lock(|pattern, duty| (pattern.get(), *duty));
        defmt::info!(
            "Pattern Ah {}, Al {}, Bh {}, Bl {}, Ch {}, Cl {}",
            p1h,
            p1l,
            p2h,
            p2l,
            p3h,
            p3l
        );
        debug_assert!(p1h != p1l || !p1h);
        debug_assert!(p2h != p2l || !p2h);
        debug_assert!(p3h != p3l || !p3h);
        (cx.shared.phase1, cx.shared.phase2, cx.shared.phase3).lock(|phase1, phase2, phase3| {
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
                _ => panic!("Tried to kill the system on phase 1 :("),
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
                _ => panic!("Tried to kill the system on phase 2 :("),
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
                _ => panic!("Tried to kill the system on phase 3 :("),
            };
            phase1.set_duty(duty);
            phase2.set_duty(duty);
            phase3.set_duty(duty);
        });
    }

    #[task(shared = [drive_pattern,duty,phase1,phase2,phase3],priority = 2)]
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
        let (mut drive_pattern, mut duty) = (cx.shared.drive_pattern, cx.shared.duty);

        let (((p1h, p1l), (p2h, p2l), (p3h, p3l)), duty) =
            (&mut drive_pattern, &mut duty).lock(|pattern, duty| (pattern.get(), *duty));
        defmt::info!(
            "Pattern Ah {}, Al {}, Bh {}, Bl {}, Ch {}, Cl {}",
            p1h,
            p1l,
            p2h,
            p2l,
            p3h,
            p3l
        );
        assert!(p2h != p2l || !p2h);
        (cx.shared.phase1, cx.shared.phase2, cx.shared.phase3).lock(|phase1, phase2, phase3| {
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
                _ => panic!("Tried to kill the system on phase 1 :("),
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
                _ => panic!("Tried to kill the system on phase 2 :("),
            };

            match (p3h, p3l) {
                (true, false) => {
                    phase3.disable_channel(pwm::Channel::C1);
                    asm::nop();
                    phase3.enable_channel(pwm::Channel::C0);
                }
                (false, true) => {
                    phase3.disable_channel(pwm::Channel::C0);
                    asm::nop();
                    phase3.enable_channel(pwm::Channel::C1);
                }
                (false, false) => {
                    phase3.disable_channel(pwm::Channel::C0);
                    asm::nop();
                    phase3.disable_channel(pwm::Channel::C1);
                }
                _ => panic!("Tried to kill the system on phase 3 :("),
            };
            phase1.set_duty(duty);
            phase2.set_duty(duty);
            phase3.set_duty(duty);
        });
    }

    /// When ever the system is not running we should sample the current.
    #[idle(local=[current_sense],shared = [sender])]
    fn idle(mut cx: idle::Context) -> ! {
        defmt::info!("idle");

        let mut count = 0;
        let (mut avg_p1, mut avg_p2, mut avg_p3) = (0., 0., 0.);
        let mut prev = Mono::now();
        loop {
            let t = Mono::now();
            let [p1, p2, p3] = match cx.local.current_sense.sample() {
                Ok(val) => val,
                Err(_) => continue,
            };

            avg_p1 += p1;
            avg_p2 += p2;
            avg_p3 += p3;
            count += 1;

            // Send the average over the latest measurement period.
            if t.checked_duration_since(prev)
                .is_some_and(|diff| diff.to_micros() > CURRENT_SAMPLE_RATE)
            {
                prev = t;
                let avg = [
                    avg_p1 / count as f32,
                    avg_p2 / count as f32,
                    avg_p3 / count as f32,
                ];
                //defmt::info!("Sending current average {:?}", avg);
                let _ = cx.shared.sender.lock(|sender| {
                    sender.set_current_sense_left((t.duration_since_epoch().to_millis(), avg))
                });
            }
        }
    }
}

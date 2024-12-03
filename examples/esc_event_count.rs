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
    dispatchers = [TIMER0,TIMER1]
)]
mod app {

    use controller::bldc::{DrivePattern, FloatDuty};
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
    use rtic_monotonics::{fugit::ExtU32, Monotonic};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
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
        hal_pins: [Pin<Input<PullUp>>; 3],
        phase1: Pwm<PWM0>,
        phase2: Pwm<PWM1>,
        phase3: Pwm<PWM2>,
        drive_pattern: DrivePattern,
        pattern_sender:
            rtic_sync::channel::Sender<'static, ((bool, bool), (bool, bool), (bool, bool)), 10>,
        pattern_receiver:
            rtic_sync::channel::Receiver<'static, ((bool, bool), (bool, bool), (bool, bool)), 10>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        nrf52840_hal::Clocks::new(cx.device.CLOCK).start_lfclk();
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
        //motor_driver::spawn().ok().unwrap();
        //mono_sweep::spawn().ok().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
                sender,
                // ~.1 = MAX speed, ~.7 zero speed
                duty: 0.75,
                velocity: 0.,
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
            },
        )
    }

    #[task(binds=GPIOTE, local=[
        events,
        hal_pins,
        t:Option<u64> = None,
        drive_pattern,
        pattern_sender,
        edge_counter:(usize,usize,usize) = (0,0,0)
        ], priority = 5)]
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
    fn handle_gpio(cx: handle_gpio::Context) {
        let edges = cx.local.edge_counter;
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1HalRising => {
                    //defmt::info!("P1 hal rising");
                    cx.local.drive_pattern.set_a();
                    edges.0 += 1;
                }
                GpioEvents::P1HalFalling => {
                    //defmt::info!("P2 hal falling");
                    cx.local.drive_pattern.clear_a();
                }
                GpioEvents::P2HalRising => {
                    //defmt::info!("P2 hal rising");
                    cx.local.drive_pattern.set_b();
                    edges.1 += 1;
                }
                GpioEvents::P2HalFalling => {
                    cx.local.drive_pattern.clear_b();
                    //defmt::info!("P2 hal falling");
                }
                GpioEvents::P3HalRising => {
                    //defmt::info!("P3 hal rising");
                    cx.local.drive_pattern.set_c();
                    edges.2 += 1;
                }
                GpioEvents::P3HalFalling => {
                    //defmt::info!("P3 hal falling");
                    cx.local.drive_pattern.clear_c();
                }
                GpioEvents::CAN => todo!("Handle CAN event"),
            }
        }
        defmt::info!("Edges {:?}", edges);
        // Trigger the phases inline since we want to run it as fast as
        // possible.
        //
        // Doing it this way ensures that we never miss a state, given that the
        // app i schedulable.
        //defmt::info!("Got hal effect interrupt!");
    }

    #[task(local = [phase1,phase2,phase3,pattern_receiver],shared = [duty],priority = 2)]
    /// Drives the phases of the motor.
    ///
    /// This is done by disabling the high/low side of each phase with respect
    /// to the switching pattern.
    ///
    /// This function should only be called by the PID controller.
    ///
    /// This ensures that the velocity can be set without dependance on the rate
    /// of interrupts.
    async fn motor_driver(mut cx: motor_driver::Context) {
        let (phase1, phase2, phase3) = (cx.local.phase1, cx.local.phase2, cx.local.phase3);
        let mut duty = 0.;
        let ((mut p1h, mut p1l), (mut p2h, mut p2l), (mut p3h, mut p3l)) = Default::default();
        loop {
            if let Ok(mut val) = cx.local.pattern_receiver.try_recv() {
                while let Ok(newval) = cx.local.pattern_receiver.try_recv() {
                    val = newval;
                }
                ((p1h, p1l), (p2h, p2l), (p3h, p3l)) = val;
                duty = cx.shared.duty.lock(|duty| duty.clone());
                //defmt::info!("ZOOOM");
            } else {
                /*if duty > 50 {
                    duty -= 1;
                } else {
                    duty = 0;
                }*/
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
            debug_assert!(p1h != p1l || !p1h);
            debug_assert!(p2h != p2l || !p2h);
            debug_assert!(p3h != p3l || !p3h);
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
            duty *= 1.005;
            if duty >= 1. {
                duty = 1.;
            }
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

    /// When ever the system is not running we should sample the current.
    #[idle(local=[current_sense],shared = [sender,duty])]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");
        loop {}
    }
}

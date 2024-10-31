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
    //use embedded_hal::digital::InputPin;
    use esc::{self, events::GpioEvents, PinConfig};
    use lib::protocol;
    use nrf52840_hal::{
        gpio::{Input, Pin, PullUp},
        pac::{PWM0, PWM1, PWM2, TIMER2, TIMER3},
        pwm::{self, Pwm},
        time::U32Ext,
        timer::{Instance, OneShot},
    };
    use rtic_monotonics::{fugit::{Duration, ExtU32}, Monotonic};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        sender: protocol::sender::Sender<10>,
        duty: f32,
        velocity: f32,
        pattern: u8,
        time_difference_hal_effect: rtic_monotonics::fugit::Duration<u64, 1, 32768>
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
        motor_control_timer: nrf52840_hal::timer::Timer<TIMER2, OneShot>,
        control_loop_timer: nrf52840_hal::timer::Timer<TIMER3, OneShot>,
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

        let mut timer = nrf52840_hal::timer::Timer::new(cx.device.TIMER2);
        timer.enable_interrupt();
        let motor_control_timer = timer.into_oneshot();

        let mut timer = nrf52840_hal::timer::Timer::new(cx.device.TIMER3);
        timer.enable_interrupt();
        let control_loop_timer = timer.into_oneshot();

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
        let time_difference_hal_effect:rtic_monotonics::fugit::Duration<u64, 1, 32768> =  Duration::<u64,1,32768>::from_ticks(0);
        (
            Shared {
                // Initialization of shared resources go here
                sender,
                // ~.1 = MAX speed, ~.7 zero speed
                duty: 0.75,
                velocity: 0.,
                pattern: Default::default(),
                time_difference_hal_effect
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
                motor_control_timer,
                control_loop_timer
            },
        )
    }

    #[task(binds=GPIOTE, local=[
        events,
        hal_pins,
        t:Option<rtic_monotonics::fugit::Instant<u64, 1, 32768>> = None,
        drive_pattern,
        ], 
        shared = [
            pattern
        ],
        priority = 5)]
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
        let new_t: rtic_monotonics::fugit::Instant<u64, 1, 32768> = Mono::now();
        let t = cx.local.t.replace(new_t.clone());
        for event in cx.local.events.events() {
            match event {
                GpioEvents::P1HalRising => {
                    //defmt::info!("P1 hal rising");
                    cx.local.drive_pattern.set_a();
                }
                GpioEvents::P1HalFalling => {
                    //defmt::info!("P2 hal falling");
                    cx.local.drive_pattern.clear_a();
                }
                GpioEvents::P2HalRising => {
                    //defmt::info!("P2 hal rising");
                    cx.local.drive_pattern.set_b();
                }
                GpioEvents::P2HalFalling => {
                    cx.local.drive_pattern.clear_b();
                    //defmt::info!("P2 hal falling");
                }
                GpioEvents::P3HalRising => {
                    //defmt::info!("P3 hal rising");
                    cx.local.drive_pattern.set_c();
                }
                GpioEvents::P3HalFalling => {
                    //defmt::info!("P3 hal falling");
                    cx.local.drive_pattern.clear_c();
                }
                GpioEvents::CAN => todo!("Handle CAN event"),
            }
        }
        //defmt::info!("Event");

        cx
            .shared.pattern.lock(|pattern| *pattern = cx.local.drive_pattern.get_state());
        // This might be bad practice but it is a neat way to wake the motor controller.
        cortex_m::peripheral::NVIC::pend(TIMER2::INTERRUPT);

        // Trigger the phases inline since we want to run it as fast as
        // possible.
        //
        // Doing it this way ensures that we never miss a state, given that the
        // app i schedulable.
        //defmt::info!("Got hal effect interrupt!");
    }

    #[task(binds=TIMER2,local = [
        phase1,
        phase2,
        phase3,
        motor_control_timer,
        ],shared = [duty,pattern,time_difference_hal_effect])]
    fn motor_sync(cx: motor_sync::Context) {

        // 1. Reset event so we do not get issues.
        cx.local.motor_control_timer.reset_event();
        // 2. Get the latest state. We need all of these at once so we might as well lock all.
        let (
                duty,
                pattern,
                dt
         ) = (cx.shared.duty,cx.shared.pattern,cx.shared.time_difference_hal_effect).lock(|a,b,c| (*a,b.clone(),c.clone()));
        
        // 3. Set a timeout for when we expect the next hall effect event to come in.
        cx.local.motor_control_timer.timeout(dt);

        // 4. Apply latest control signals.
        let (phase1,phase2,phase3) = (cx.local.phase1,cx.local.phase2,cx.local.phase3);

        // Burns a bit of memory but we get a single jump instruction in to 8 single write instructions.
        match pattern {
            0b00_00_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_00_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_00_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_01_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_01_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_01_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_10_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_10_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b00_10_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b00,2>();
            }
            0b01_00_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_00_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_00_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_01_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_01_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_01_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_10_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_10_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b01_10_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b01,2>();
            }
            0b10_00_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_00_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_00_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b00,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_01_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_01_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_01_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b01,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_10_00 => {
                phase1.modify_channels::<0b00,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_10_01 => {
                phase1.modify_channels::<0b01,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b10,2>();
            }
            0b10_10_10 => {
                phase1.modify_channels::<0b10,2>();
                phase2.modify_channels::<0b10,2>();
                phase3.modify_channels::<0b10,2>();
            },
            _ =>{
                phase1.disable();
                phase2.disable();
                phase3.disable();
                panic!("Tried to kill the system by shorting two phases.")
            }
        }


        phase1.set_duty(duty);
        phase2.set_duty(duty);
        phase3.set_duty(duty);
        // DONE :) Now return ensuring that we allow other tasks to run.
    }

    #[task(binds = TIMER3)]
    fn control_loop(_cx:control_loop::Context) {}


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

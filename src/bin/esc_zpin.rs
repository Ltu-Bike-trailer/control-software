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

use controller as _; 
// global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::timer::prelude::*;
nrf_timer4_monotonic!(Mono,16_000_000);


#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,TIMER1]
)]
mod app {

    use controller::{bldc::{DrivePattern, FloatDuty}, cart};
    //use embedded_hal::digital::InputPin;
    use esc::{self, events::GpioEvents, PinConfig};
    use lib::protocol;
    use nrf52840_hal::{
        gpio::{Input, Pin, PullUp},
        pac::{PWM0, PWM1, PWM2, TIMER0, TIMER2, TIMER3},
        pwm::{self, Pwm},
        time::U32Ext,
        timer::{  OneShot},
    };
    use rtic_monotonics::{fugit::{Duration,ExtU32, ExtU64}, Monotonic};

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        sender: protocol::sender::Sender<10>,
        duty: u16,
        velocity: f32,
        pattern: u8,
        time_difference_hal_effect: rtic_monotonics::fugit::Duration<u64, 1, 32768>,
        current: [f32;3],
        debounce: nrf52840_hal::timer::Timer<TIMER0, OneShot>,
        motor_control_timer: nrf52840_hal::timer::Timer<TIMER2, OneShot>,
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
        control_loop_timer: nrf52840_hal::timer::Timer<TIMER3, OneShot>,
        pid:controller::cart::controllers::MotorPid,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        nrf52840_hal::Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        defmt::info!("Clock config done.");
        Mono::start(cx.device.TIMER4);

        let p0 = nrf52840_hal::gpio::p0::Parts::new(cx.device.P0);
        let p1 = nrf52840_hal::gpio::p1::Parts::new(cx.device.P1);
        let ppi = nrf52840_hal::ppi::Parts::new(cx.device.PPI);

        let pins = PinConfig::new(p0, p1, ppi, cx.device.GPIOTE);
        let (pins, p1) = pins.configure_p1();
        let (pins, p2) = pins.configure_p2();
        let (pins, p3) = pins.configure_p3();
        let (pins, mut current_sense) = pins.configure_adc(cx.device.SAADC);
        let events = pins.complete();
        let hal_pins = [p1.hal_effect, p2.hal_effect, p3.hal_effect];

        current_sense.start_sample();
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
        phase1.enable();
        phase2.enable();
        phase3.enable();




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
        let mut control_loop_timer = timer.into_oneshot();
        control_loop_timer.enable_interrupt();
        control_loop_timer.timeout::<1,{cart::controllers::MOTOR_TIMESCALE as u32}>(cart::controllers::MOTOR_TS.micros().into());
        let pid = controller::cart::controllers::MotorPid::new(0);
        let mut timer = nrf52840_hal::timer::Timer::new(cx.device.TIMER0);
        timer.enable_interrupt();
        let debounce = timer.into_oneshot();
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
        phase1.enable();
        phase2.enable();
        phase3.enable();
        // Spawn the tasks.
        //motor_driver::spawn().ok().unwrap();
        //mono_sweep::spawn().ok().unwrap();
        let time_difference_hal_effect:rtic_monotonics::fugit::Duration<u64, 1, 32768> =  Duration::<u64,1,32768>::from_ticks(0);
        defmt::info!("Ztarted");
        (
            Shared {
                // Initialization of shared resources go here
                sender,
                // ~.1 = MAX speed, ~.7 zero speed
                duty: (cart::constants::PWM_MAX/2) as u16,
                velocity: 0.,
                pattern: Default::default(),
                time_difference_hal_effect,
                current: [0.;3],
                motor_control_timer,
                debounce
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
                control_loop_timer,
                pid,
            },
        )
    }

    #[task(binds=GPIOTE, local=[
        hal_pins,
        t:Option<rtic_monotonics::fugit::Instant<u64, 1, 16000000>> = None,
        drive_pattern,
        events,
        ], 
        shared = [
            debounce,
            pattern,
            motor_control_timer,
        ],
        priority = 7
    )]
    /// Manages gpiote interrupts.
    ///
    /// This is every pin interrupt. So hal effect feedback, can bus messages
    /// and possibly more. the contents of this function will vary with the
    /// board we are using.
    ///
    /// ## Motor driving
    ///
    /// This function also drives the motors. This is the default case. The only
    /// other time that the motor should be driven externally is if the PID
    /// controller requests a new velocity.
    ///
    /// This ensures that the motor can be started from standstill.
    fn handle_gpio(mut cx: handle_gpio::Context) {
        let new_t: rtic_monotonics::fugit::Instant<u64, 1, 16000000> = Mono::now();
        let t: Option<rtic_monotonics::fugit::Instant<u64, 1, 16000000>> = cx.local.t.replace(new_t.clone());
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
            .shared.pattern.lock(|pattern| *pattern = cx.local.drive_pattern.get_pattern_u8());
        cortex_m::peripheral::NVIC::mask(nrf52840_hal::pac::Interrupt::GPIOTE);
        // This might be bad practice but it is a neat way to wake the motor controller.
        cx.shared.debounce.lock(|d|{
            d.timeout::<1, 16000000>(1u64.micros().into());
            d.enable_interrupt();
        });
        //defmt::info!("State : {}",cx.local.drive_pattern.get_state());
        //cortex_m::peripheral::NVIC::pend(TIMER2::INTERRUPT);
       // cx.shared.motor_control_timer.lock(|timer| { timer.timeout::<1, 16000000>(100u64.micros()) });

        let _ = phase_driver::spawn();
        // Trigger the phases inline since we want to run it as fast as
        // possible.
        //
        // Doing it this way ensures that we never miss a state, given that the
        // app i schedulable.


    }

    #[task(
        binds=TIMER0,
        shared = [
            debounce,],
        priority = 8
    )]
    /// Runs the phases. This should be blazingly fast now.
    /// 
    /// 1. Single write.
    /// 2. write base pri, copy resources, write base pri. Just a few instructions
    /// 3. set timeout. This should be a single write.
    /// 4. Table branch, 12 writes for en/dis, 3 writes for pwm.
    /// 
    /// It takes some 1.2KiB data, which is fine since we want it to branch far rather than often.
    fn debounce(mut cx: debounce::Context) {
        cx.shared.debounce.lock(|debounce|debounce.reset_event());
        unsafe { cortex_m::peripheral::NVIC::unmask(nrf52840_hal::pac::Interrupt::GPIOTE) };
        //defmt::info!("Bounced");
        
    }

    #[task(
        //binds=TIMER2,
        local = [
            phase1,
            phase2,
            phase3,
            prev_duty:u16 = 0,
            prev_step:u8 = 0,
        ],
        shared = [
            duty,
            pattern,
            time_difference_hal_effect,
            motor_control_timer,
        ],
        priority = 5
    )]
    /// Runs the phases. This should be blazingly fast now.
    /// 
    /// 1. Single write.
    /// 2. write base pri, copy resources, write base pri. Just a few instructions
    /// 3. set timeout. This should be a single write.
    /// 4. Table branch, 12 writes for en/dis, 3 writes for pwm.
    /// 
    /// It takes some 1.2KiB data, which is fine since we want it to branch far rather than often.
    async fn phase_driver(cx: phase_driver::Context) {
        let now = Mono::now();
        Mono::delay(10u64.micros()).await;
        defmt::info!("Phaze!");
        //cx.shared.motor_control_timer.lock(|t| t.reset_event());
        //defmt::info!("Phase driving");

        // 1. Reset event so we do not get issues.
        //cx.local.motor_control_timer.reset_event();
        // 2. Get the latest state. We need all of these at once so we might as well lock all.
        let (
                duty,
                mut pattern,
                dt
        ) = (cx.shared.duty,cx.shared.pattern,cx.shared.time_difference_hal_effect).lock(|a,b,c| ({
            let ret = *a;
            //let new = *a as u32 *100;
            //*a = (new /110) as u16;
            //ret
            ret        
            },b.clone(),c.clone()));

        //NVIC::unpend(TIMER2::INTERRUPT);
        // 3. Set a timeout for when we expect the next hall effect event to come in.

        // 4. Apply latest control signals.
        let (phase1,phase2,phase3) = (cx.local.phase1,cx.local.phase2,cx.local.phase3);
        if pattern == 0 {
            pattern = *cx.local.prev_step;
        }
        defmt::info!("Supplied {:b}",pattern.clone());
        
        // Burns a bit of memory but we get a single jump instruction in to 8 single write instructions.
        // While this is a bit expensive we expect it to be ran every time. So nothing smart to do here.

        /*
        match pattern {
            0b00_00_00 => {
                phase1.disable_channel(pwm::Channel::C0);
                phase1.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
            }
            0b00_00_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_00_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_01_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_01_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_01_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_10_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_10_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b00_10_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b01_00_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_00_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_00_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_01_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_01_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_01_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_10_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_10_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b01_10_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            }
            0b10_00_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_00_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_00_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_01_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_01_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_01_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_10_00 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_10_01 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            0b10_10_10 => {
                phase1.enable_channel(pwm::Channel::C1);
                phase1.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
                phase2.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
                phase3.disable_channel(pwm::Channel::C0);
            }
            val =>{
                phase1.disable();
                phase2.disable();
                phase3.disable();
                panic!("Tried to kill the system by shorting two phases.")
            }
        }*/

        match pattern & 0b11 {
            0b10 => {
                phase1.disable_channel(pwm::Channel::C1);
                phase1.enable_channel(pwm::Channel::C0);
            },
            0b01 => {
                phase1.disable_channel(pwm::Channel::C0);
                phase1.enable_channel(pwm::Channel::C1);
            }
            0b00 => {
                phase1.disable_channel(pwm::Channel::C0);
                phase1.disable_channel(pwm::Channel::C1);
            }
            _ => {
                panic!("Invalid state");
            }
        }

        match pattern & 0b1100 {
            0b10_00 => {
                phase2.disable_channel(pwm::Channel::C1);
                phase2.enable_channel(pwm::Channel::C0);
            },
            0b01_00 => {
                phase2.disable_channel(pwm::Channel::C0);
                phase2.enable_channel(pwm::Channel::C1);
            }
            0b00_00 => {
                phase2.disable_channel(pwm::Channel::C0);
                phase2.disable_channel(pwm::Channel::C1);
            }
            _ => {
                panic!("Invalid state");
            }
        }

        match pattern & 0b110000 {
            0b10_00_00 => {
                phase3.disable_channel(pwm::Channel::C1);
                phase3.enable_channel(pwm::Channel::C0);
            },
            0b01_00_00 => {
                phase3.disable_channel(pwm::Channel::C0);
                phase3.enable_channel(pwm::Channel::C1);
            }
            0b00_00_00 => {
                phase3.disable_channel(pwm::Channel::C0);
                phase3.disable_channel(pwm::Channel::C1);
            }
            _ => {
                panic!("Invalid state");
            }
        }

         
         /*  
        if *cx.local.prev_step == pattern {
            cx.shared.motor_control_timer.lock(|w| {
                w.timeout::<1,16_000_000>(100u64.millis().into());
                w.enable_interrupt();
            });
        }*/

        // This is semi expensive.
        // So if we do not need to do it we should not.
        /*if duty == *cx.local.prev_duty {
            return;
        }*/
        let fduty = duty as f32 / cart::constants::PWM_MAX as f32;
        let fduty = fduty.clamp(0.1, 0.9);
        phase1.set_duty(fduty);
        phase2.set_duty(fduty);
        phase3.set_duty(fduty);
        /* 
        phase1.set_duty_on_common(cart::constants::PWM_MAX as u16- duty);
        phase2.set_duty_on_common(cart::constants::PWM_MAX as u16-duty);
        phase3.set_duty_on_common(cart::constants::PWM_MAX as u16-duty);
           */ 
        *cx.local.prev_duty = duty;
        //defmt::info!("Phase driving exit");
        // DONE :) Now return ensuring that we allow other tasks to run.
    }

    

    /* 
    #[task(binds = TIMER3,
        local = [
            pid,
            control_loop_timer
        ],
        shared = [
            duty,
            current,
        ]
    )]
    fn control_loop(mut cx:control_loop::Context) {
        defmt::info!("Control loop");
        let start: rtic_monotonics::fugit::Instant<u64, 1, 16000000> = Mono::now();
        let timer = cx.local.control_loop_timer;
        timer.reset_event();
        const DURATION:rtic_monotonics::fugit::Duration<u64, 1, 16000000> = rtic_monotonics::fugit::Duration::<u64, 1, { controller::cart::controllers::MOTOR_TS }>::from_ticks(controller::cart::controllers::MOTOR_TS as u64).convert();
        
        let [c1,c2,c3] = cx.shared.current.lock(|c| {
            c.clone()
        });
        
        let current = c1.max(c2).max(c3);
        defmt::info!("Current : {} mA",current);
        
        // Target 500 mA
        cx.local.pid.follow(500.);
        cx.local.pid.register_measurement(current, controller::cart::controllers::MOTOR_TS);
        // This is fine. The only time this throws an error is if the u16 fails to write which it never does.
        let _actuation = unsafe { cx.local.pid.actuate().unwrap_unchecked() };
        //cx.shared.duty.lock(|duty| *duty = actuation.actuation as u16);
        timer.timeout(unsafe { (start+DURATION).checked_duration_since(Mono::now()).unwrap_unchecked() });
    }
    */

    #[task(binds = SAADC, local=[current_sense,n:i32 = 0],shared =[current])]
    /// Continuously samples the current.
    /// 
    /// If there is not recipient the value is merely averaged with the previous timestamp.
    fn current_sense(mut cx:current_sense::Context) {
        //defmt::info!("ZAMPLE :/");
        let sample = cx.local.current_sense.complete_sample();
        //defmt::info!("Sampled : {:?}",sample);
        cx.shared.current.lock(|target|  *target = sample);
        cx.local.current_sense.start_sample();
    }


    /* 
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
    */

    /// When ever the system is not running we should sample the current.
    #[idle(shared = [sender,duty])]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");
        loop {}
    }
}

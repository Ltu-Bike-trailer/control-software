//! Defines the main app that runs on our cart

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = nrf52840_hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [TIMER0]
)]
mod app {
    use bsp::CurrentManager;

    //use rtic_monotonics::{fugit::ExtU32, Monotonic};
    use super::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
        adc: CurrentManager,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        nrf52840_hal::Clocks::new(cx.device.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk();

        Mono::start(cx.device.RTC0);
        let pa = nrf52840_hal::gpio::p0::Parts::new(cx.device.P0);
        let pb = nrf52840_hal::gpio::p1::Parts::new(cx.device.P1);
        let board = bsp::PinConfig::new(pa, pb);
        let (_board, mut adc) = board.configure_adc(cx.device.SAADC);
        adc.start_sample();
        // TODO setup monotonic if used
        // let sysclk = { /* clock setup + returning sysclk as an u32 */ };
        // let token = rtic_monotonics::create_systick_token!();
        // rtic_monotonics::systick::Systick::new(cx.core.SYST, sysclk, token);
        //task1::spawn().ok().unwrap();
        defmt::info!("Out of init");
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                // Initialization of local resources go here
                adc,
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
    #[task(local=[adc], binds = SAADC)]
    fn sample_current(cx: sample_current::Context) {
        let read = cx.local.adc.complete_sample();
        defmt::info!("Read : {} A", read[0]);
        cx.local.adc.start_sample();
    }

    /*
    // TODO: Add tasks
    #[task(priority = 1,local=[adc])]
    async fn task1(cx: task1::Context) {
        defmt::info!("Hello from task1!");
        loop {
            let sample = cx.local.adc.sample().await;
            defmt::info!("Current : {}A", sample);
            Mono::delay(10u32.millis().into()).await;
        }
    }*/
}

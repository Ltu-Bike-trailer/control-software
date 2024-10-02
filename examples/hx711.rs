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
    device = nrf52840_hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [RTC1]
)]
mod app {

    use hx711::Hx711;
    use nrf52840_hal::{self as hal, gpio::{self, Level}, saadc::{SaadcConfig, Time}, Delay};
    
    use prelude::{ExtU64, Monotonic};
    use rtic_monotonics::nrf::rtc::*;

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
    }

    // Local resources go here
    #[local]
    struct Local {
        driver:  Hx711<Delay, gpio::Pin<gpio::Input<gpio::Floating>>, gpio::Pin<gpio::Output<gpio::PushPull>>>
    }

    #[init]
    #[allow(dead_code, unreachable_code, unused_variables)]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let device = cx.device;

        hal::clocks::Clocks::new(device.CLOCK).start_lfclk();
        // Initialize Monotonic
        Mono::start(device.RTC0);
        let delay = Delay::new(unsafe {core::mem::transmute(())});

        let port0 = gpio::p0::Parts::new(device.P0);
        //let port1 = gpio::p1::Parts::new(device.P1);

        let sck = port0.p0_05.into_push_pull_output(Level::Low).degrade();
        let miso= port0.p0_07.into_floating_input().degrade();


        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_40US;
        // TODO: Change this to a DMA version as to not block for 3us.
        //let mut adc = Saadc::new(device.SAADC, cfg);
        // let mut p2cs = port0.p0_30.into_floating_input();
        // let mut p1cs = port0.p0_31.into_floating_input();
        // loop {
        //     let val = adc.read_channel(&mut p2cs).unwrap() -  adc.read_channel(&mut p1cs).unwrap();
        //     defmt::info!("Value :) {:?}", val);
        //     for _ in 0..1000000 {
        //         cortex_m::asm::nop();
        //     }
        // }




        //let spi = Spi::new(device.SPI1, pins, Frequency::M1, MODE_1);
        let driver = hx711::Hx711::new(delay, miso, sck).ok().unwrap();
        blast::spawn().ok().unwrap();
        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                driver, // Initialization of local resources go here
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {}
    }
    #[task(local = [driver], priority = 1)]
    async fn blast(cx: blast::Context) -> () {
        loop {
            let t1 = Mono::now();
            match cx.local.driver.retrieve() {
                Ok(val) => defmt::info!("Value :) {}", val),
                Err(_e) => defmt::info!("Error :/"),
            }

            Mono::delay_until(t1 + 100u64.millis()).await;
        }
    }
}

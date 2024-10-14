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
    dispatchers = [RTC1]
)]
mod app {

    use hx711_spi::Hx711;
    use nrf52840_hal::{self as hal, gpio::{self, Level}, spi::{Frequency, Spi}};
    use nrf52840_hal::spi;
    
    use pac::SPI1;
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
        driver: Hx711<Spi<SPI1>>
    }

    #[init]
    #[allow(dead_code, unreachable_code, unused_variables)]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let device = cx.device;

        // Initialize Monotonic
        hal::clocks::Clocks::new(device.CLOCK).start_lfclk();
        Mono::start(device.RTC0);

        let port0 = gpio::p0::Parts::new(device.P0);

        let sck = port0.p0_05.into_push_pull_output(Level::Low).degrade();
        let miso= port0.p0_07.into_floating_input().degrade();

        defmt::info!("pre-driver");
  
        let pins = spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: None,
        };
       
        let spi = Spi::new(device.SPI1, pins, Frequency::M1, embedded_hal::spi::MODE_1);
        let driver: Hx711<Spi<SPI1>>  = hx711_spi::Hx711::new(spi);
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

        loop {
            blast::spawn().ok();
        }
    }
    #[task(local = [driver], priority = 1)]
    async fn blast(cx: blast::Context) -> () {
        defmt::info!("blast");
        let t1 = Mono::now();
        match cx.local.driver.retrieve() {
            Ok(val) => defmt::info!("Value :) {}", val),
            Err(_e) => defmt::info!("Error :/"),
        }

        Mono::delay_until(t1 + 10000u64.millis()).await;
    }
}

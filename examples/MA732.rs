//! Defines the main app that runs on our cart

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]
#![allow(unused_imports)]

use controller as _;
use rtic_monotonics::nrf_rtc0_monotonic; // global logger + panicking-behavior + memory layout
nrf_rtc0_monotonic!(Mono);

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC1]
)]
mod app {
    use cortex_m::asm::delay;
    use embedded_hal::{digital::{InputPin, OutputPin}, spi::SpiBus};
    use nrf52840_hal::{self as hal, self, gpio::{p0::Parts, Floating, Input, Level, Output, Pin, PushPull}, spi::{self, Frequency, Spi}};

    use nrf52840_hal::pac::{p0, SPI1};
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
        spim: Spi<SPI1>,
        cs: Pin<Output<PushPull>>
    }

    #[init]
    #[allow(dead_code, unreachable_code, unused_variables)]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let device = cx.device;

        // Initialize Monotonic
        hal::clocks::Clocks::new(device.CLOCK).start_lfclk().enable_ext_hfosc();
        Mono::start(device.RTC0);

        let port0 = Parts::new(device.P0);

        let sck: Pin<Output<PushPull>> = port0.p0_07.into_push_pull_output(Level::Low).degrade();
        let miso: Pin<Input<Floating>>= port0.p0_08.into_floating_input().degrade();
        let cs: Pin<Output<PushPull>>  = port0.p0_05.into_push_pull_output(Level::High).degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::Low).degrade();
        
        defmt::info!("pre-driver");
  
        let pins = spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: Some(mosi)
        };
       
        let spim: Spi<SPI1> = Spi::new(device.SPI1, pins, Frequency::K250, embedded_hal::spi::MODE_0);

        blast::spawn().ok().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                spim,
                cs
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


    #[task(local = [spim, cs], priority = 1)]
    async fn blast(cx: blast::Context) -> () {
        let blast::LocalResources {cs, spim, .. } = cx.local;

        let mut angle: [u8; 2] = [0; 2];
        cs.set_low().ok();
        //spim.transfer(cs, &mut angle).ok();
        spim.transfer_in_place(&mut angle).ok();
        cs.set_high().ok();

        defmt::info!("angle:{:?}",angle);
        delay(10000000);
    }

}

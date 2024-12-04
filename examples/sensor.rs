//! Defines the main app that runs on our cart

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait, generic_arg_infer)]
#![deny(clippy::all)]
#![deny(warnings)]
#![allow(unused_imports)]

use controller as _;
use rtic_monotonics::nrf_rtc0_monotonic; // global logger + panicking-behavior + memory layout
nrf_rtc0_monotonic!(Mono);
use controller::drivers::*;

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC1]
)]

mod app {
    use core::{arch::asm, f32::NAN};

    use controller::drivers::MA732::{self, Driver, Register};
    use cortex_m::asm::{self, wfi};
    use embedded_hal::delay::DelayNs;
    use nrf52840_hal::{
        gpio::{p0::Parts, Floating, Input, Level, Output, Pin, PushPull},
        pac::SPI1,
        spi::{self, Frequency, Spi},
        Clocks,
    };

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {
        spi_angle: Spi<SPI1>,
        driver1: Driver<Pin<Output<PushPull>>>,
        driver2: Driver<Pin<Output<PushPull>>>,
    }

    #[init]
    #[allow(dead_code, unreachable_code, unused_variables)]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let device = cx.device;

        Clocks::new(device.CLOCK).start_lfclk().enable_ext_hfosc();
        Mono::start(device.RTC0);

        let port0 = Parts::new(device.P0);

        let sck: Pin<Output<PushPull>> = port0.p0_28.into_push_pull_output(Level::Low).degrade();
        let miso: Pin<Input<Floating>> = port0.p0_08.into_floating_input().degrade();
        let cs1: Pin<Output<PushPull>> = port0.p0_05.into_push_pull_output(Level::High).degrade();
        let cs2: Pin<Output<PushPull>> = port0.p0_07.into_push_pull_output(Level::High).degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::Low).degrade();

        defmt::info!("pre-driver");

        let pins = spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: Some(mosi),
        };

        let spi_angle: Spi<SPI1> = Spi::new(
            device.SPI1,
            pins,
            Frequency::K250,
            embedded_hal::spi::MODE_0,
        );

        let driver1 = MA732::Driver::new(cs1);
        let driver2 = MA732::Driver::new(cs2);

        let _ = read_angle::spawn();

        (Shared {}, Local {
            spi_angle,
            driver1,
            driver2,
        })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            wfi();
        }
    }

    #[task(local = [spi_angle, driver1, driver2],priority=2,)]
    async fn read_angle(cx: read_angle::Context) {
        let driver1 = cx.local.driver1;
        let driver2 = cx.local.driver2;
        let spi_angle = cx.local.spi_angle;

        let mut angle1 = 0.;
        let mut angle2 = 0.;
        loop {
            let angle1_new = driver1.read_angle(spi_angle);
            let angle2_new = driver2.read_angle(spi_angle);

            let Δ1 = angle1 - angle1_new;
            let Δ2 = angle2 - angle2_new;

            //if Δ is >= treshold
            //if diff small dont update
            if (Δ1 >= 0.01 || 0.01 <= Δ1) && (Δ2 >= 0.01 || 0.01 <= Δ2) {
                angle1 = angle1_new;
                angle2 = angle2_new;

                //TODO: SEND UPDATE MESSAGE VIA CAN
            }

            //let the processor sleep/do other things
            Mono::delay_ms(&mut Mono, 100);
        }
    }
}

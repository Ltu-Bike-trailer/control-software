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
use controller::drivers::MA732::*;

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC1]
)]

mod app {
    use core::arch::asm;

    use controller::drivers::MA732::{Driver, Register};
    use cortex_m::asm;
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
    struct Shared {

    }

    // Local resources go here
    #[local]
    struct Local {
        spim: Spi<SPI1>,
        driver: Driver<Pin<Output<PushPull>>>,
    }

    #[init]
    #[allow(dead_code, unreachable_code, unused_variables)]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let device = cx.device;

        // Initialize Monotonic
        Clocks::new(device.CLOCK).start_lfclk().enable_ext_hfosc();
        Mono::start(device.RTC0);

        let port0 = Parts::new(device.P0);

        let sck: Pin<Output<PushPull>> = port0.p0_28.into_push_pull_output(Level::Low).degrade();
        let miso: Pin<Input<Floating>> = port0.p0_08.into_floating_input().degrade();
        let cs: Pin<Output<PushPull>> = port0.p0_05.into_push_pull_output(Level::High).degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::Low).degrade();

        defmt::info!("pre-driver");

        let pins = spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: Some(mosi),
        };

        let spim: Spi<SPI1> = Spi::new(
            device.SPI1,
            pins,
            Frequency::K250,
            embedded_hal::spi::MODE_0,
        );

        let driver = Driver::new(cs);

        worker::spawn().unwrap();

        (
            Shared {

            },
            Local {
                spim,
                driver,
            },
        )
    }

    #[task(local = [driver,spim],priority=2)]
    async fn worker(cx: worker::Context) {
        let (driver, mut spim) = (cx.local.driver, cx.local.spim);

        defmt::info!("Angle pre setting {:?}", driver.read_angle(&mut spim));
        defmt::info!(
            "Read pre update {:?}",
            driver
                .read_register::<Mono, _, _, _>(Register::ZeroSetting1, &mut spim)
                .await
        );
        driver
            .write_register::<Mono, _, _, _>(Register::ZeroSetting1, &mut spim, 100)
            .await;
        defmt::info!(
            "Read post update {:?}",
            driver
                .read_register::<Mono, _, _, _>(Register::ZeroSetting1, &mut spim)
                .await
        );
        defmt::info!("Angle post setting {:?}", driver.read_angle(&mut spim));


        loop{
            defmt::info!("Angle {:?}", driver.read_angle(&mut spim));
            Mono::delay_ms(&mut Mono,500);
        }
    }

}

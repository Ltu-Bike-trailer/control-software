//! Defines the main app that runs on our cart

#![no_main]
#![no_std]
#![feature(type_alias_impl_trait,generic_arg_infer)]
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
    use controller::drivers::{MA732::Driver, MA732::Register};
    use nrf52840_hal::{gpio::{p0::Parts, Floating, Input, Level, Output, Pin, PushPull}, pac::SPI1, spi::{self, Frequency, Spi}, Clocks};

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
        
        driver:Driver<Pin<Output<PushPull>>>
        //cs: Pin<Output<PushPull>>
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
        let miso: Pin<Input<Floating>>= port0.p0_08.into_floating_input().degrade();
        let cs: Pin<Output<PushPull>>  = port0.p0_05.into_push_pull_output(Level::High).degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::Low).degrade();
        
        defmt::info!("pre-driver");
  
        let pins = spi::Pins {
            sck:  Some(sck),
            miso: Some(miso),
            mosi: Some(mosi)
        };
       
        let spim: Spi<SPI1> = Spi::new(device.SPI1, pins, Frequency::K250, embedded_hal::spi::MODE_0);

        let driver = Driver::new(cs);

        worker::spawn().unwrap();

        //blast::spawn().ok().unwrap();

        (
            Shared {
                // Initialization of shared resources go here
            },
            Local {
                spim,
                driver
               //cs
            },
        )
    }

    #[task(local = [driver,spim],priority=2)]
    async fn worker(cx:worker::Context) {
        
        let (driver, mut spim) = (cx.local.driver,cx.local.spim);

        defmt::info!("Angle pre setting {:?}",  driver.read_angle(&mut spim));
        defmt::info!("Read pre update {:?}",           driver.read_register::<Mono,_,_,_>(Register::ZeroSetting1,&mut spim).await);
                                                       driver.write_register::<Mono,_,_,_>(Register::ZeroSetting1,&mut spim, 100).await;
        defmt::info!("Read post update {:?}",          driver.read_register::<Mono,_,_,_>(Register::ZeroSetting1,&mut spim).await);
        defmt::info!("Angle post setting {:?}", driver.read_angle(&mut spim));

    }


    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            //blast::spawn().ok();
        }
    }

    /*
    #[task(local = [spim, cs], priority = 2)]
    async fn blast(cx: blast::Context) -> () {
        defmt::info!("blast");
        let blast::LocalResources {cs, spim, .. } = cx.local;

        let mut angle: [u8; 2] = [0; 2];
        cs.set_low().ok();
        //spim.read(cs, &mut angle).ok();
        spim.read(&mut angle).ok();
        cs.set_high().ok();
        //let tmp: u16 = (angle[0] as u16) + (angle[1] as u16) <<8;
        defmt::info!("angle:{:?}",angle);
        defmt::info!("angle:{:?}",u16::from_be_bytes(angle));
        defmt::info!("angle:{:?}", (u16::from_be_bytes(angle) as f32 / u16::MAX as f32) * f32::consts::TAU); // Tau radians

        
        delay(10000000);
    }
     */
}

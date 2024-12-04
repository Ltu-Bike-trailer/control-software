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

    use controller::drivers::{
        can::{Mcp2515Driver, Mcp2515Settings},
        MA732::{self, Driver, Register},
    };
    use cortex_m::asm::{self, wfi};
    use embedded_can::{blocking::Can, Frame, StandardId};
    use embedded_hal::delay::DelayNs;
    use lib::protocol::sender::Sender;
    use nrf52840_hal::{
        gpio::{p0::Parts, Floating, Input, Level, Output, Pin, PullUp, PushPull},
        gpiote::Gpiote,
        pac::SPI0,
        spi::{self, Frequency, Spi},
        Clocks,
    };

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    // Local resources go here
    #[local]
    struct Local {
        spi_angle: Spi<SPI0>,
        driver1: Driver<Pin<Output<PushPull>>>,
        driver2: Driver<Pin<Output<PushPull>>>,
        can_driver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        sender: Sender<10>,
    }

    #[init]
    #[allow(dead_code, unreachable_code, unused_variables)]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        let device = cx.device;

        Clocks::new(device.CLOCK).start_lfclk().enable_ext_hfosc();
        Mono::start(device.RTC0);

        let port0 = Parts::new(device.P0);

        //ANGLE PINS
        let cs1: Pin<Output<PushPull>> = port0.p0_05.into_push_pull_output(Level::High).degrade();
        let cs2: Pin<Output<PushPull>> = port0.p0_07.into_push_pull_output(Level::High).degrade();

        //CAN PINS
        let can_interrupt = port0.p0_20.into_pullup_input().degrade();
        let cs0: Pin<Output<PushPull>> = port0.p0_23.into_push_pull_output(Level::High).degrade();

        //SHARED
        let sck: Pin<Output<PushPull>> = port0.p0_28.into_push_pull_output(Level::Low).degrade();
        let miso: Pin<Input<Floating>> = port0.p0_08.into_floating_input().degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::Low).degrade();

        let pins = spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: Some(mosi),
        };

        let spi_angle: Spi<SPI0> = Spi::new(
            device.SPI0,
            pins,
            Frequency::K250,
            embedded_hal::spi::MODE_0,
        );

        let driver1 = MA732::Driver::new(cs1);
        let driver2 = MA732::Driver::new(cs2);

        // CAN stuff
        let spi_can: Spi<SPI0> = unsafe { (0x0 as *mut Spi<SPI0>).read() };
        let settings = Mcp2515Settings::default();
        let can_driver = Mcp2515Driver::init(spi_can, cs0, can_interrupt, settings);
        let gpiote = Gpiote::new(device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        let sender: Sender<_> = Sender::new();

        let _ = read_angle::spawn();

        (Shared { gpiote }, Local {
            spi_angle,
            driver1,
            driver2,
            can_driver,
            sender,
        })
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        loop {
            wfi();
        }
    }

    #[task(local = [spi_angle, driver1, driver2, can_driver, sender],priority=2,)]
    async fn read_angle(cx: read_angle::Context) {
        let driver1 = cx.local.driver1;
        let driver2 = cx.local.driver2;
        let can_driver = cx.local.can_driver;
        let spi_angle = cx.local.spi_angle;
        let sender = cx.local.sender;

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

                sender.set_alpha(angle1).unwrap();
                sender.set_theta(angle2).unwrap();

                //transmit first angle
                let mut msg = sender.dequeue().unwrap();
                can_driver.transmit(&msg).unwrap();
                msg.print_frame();

                //transmit 2nd angle
                msg = sender.dequeue().unwrap();
                msg.print_frame();
                can_driver.transmit(&msg).unwrap();
            }

            //let the processor sleep/do other things
            Mono::delay_ms(&mut Mono, 100);
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote])]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        //Do nothing this board has a shared SPI bus so its dangerous to read whenever
        //also it has no reason to read any messages of the can bus.
        let _ = cx.shared.gpiote.lock(|gpiote| {
            gpiote.reset_events();
        });
    }
}

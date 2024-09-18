#![no_std]
#![no_main]
#![allow(unused)]

use controller::drivers::can::{CanDriver, CanModule, CanSettings, CanMessage};
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use cortex_m::asm as _;
use defmt_rtt as _;
use panic_probe as _;
use cortex_m_rt::entry;
use rtic::app;

use controller as _;



#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC0])]
mod app {
    use controller::drivers::can::{CanDriver, CanModule, CanSettings, CanMessage, OperationTypes};
    use cortex_m::asm;
    use embedded_hal::digital::OutputPin;
    use nrf52840_hal::{pac::SPI0, spi::Spi, spi::Frequency, spim::*};
    use nrf52840_hal::gpio::{self, Level, Port, Pin, Output, PushPull};
    
    
    #[shared]
    struct Shared{}

    #[local]
    struct Local {
        candriver: CanDriver<Spi<SPI0>, Pin<Output<PushPull>>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local){
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
         
        defmt::println!("Initialize the SPI instance, and CanDriver");
        let pins = nrf52840_hal::spi::Pins{
            sck: Some(port0.p0_21.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_22.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_23.into_floating_input().degrade()),
        };

        let mut spi = Spi::new(device.SPI0, pins, Frequency::K500, MODE_0);
        let cs_pin = port0.p0_24.into_push_pull_output(Level::High).degrade(); 
        
        let can_settings = CanSettings{
            mode: OperationTypes::Loopback,
            can_clk: 0,
            can_bitrate: 0, 

        };

        let mut can_driver = CanDriver::init(spi, cs_pin, can_settings);
        
        defmt::println!("After initializing Spi<SPI0>..."); 

        (
            Shared {},
            Local {candriver: can_driver},
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi(); 
        }
    }
}

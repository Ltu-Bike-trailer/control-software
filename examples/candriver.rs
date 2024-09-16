#![no_std]
#![no_main]
#![allow(unused)]

use controller::drivers::can::CanDriver;
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use cortex_m::asm as _;
use defmt_rtt as _;
use panic_probe as _;
use cortex_m_rt::entry;

use controller as _;



#[entry]
fn main() -> ! {
    defmt::info!("init");
    defmt::println!("...");

    let periperal = nrf52840_hal::pac::Peripherals::take().unwrap();
    let port0 = nrf52840_hal::gpio::p0::Parts::new(periperal.P0);
    
    defmt::println!("Initialize the SPI instance, and CanDriver");

    let pins = nrf52840_hal::spi::Pins{
        sck: Some(port0.p0_21.into_push_pull_output(Level::Low).degrade()),
        mosi: Some(port0.p0_22.into_push_pull_output(Level::Low).degrade()),
        miso: Some(port0.p0_23.into_floating_input().degrade()),
    };

    let mut spi = nrf52840_hal::spi::Spi::new(periperal.SPI0, pins, nrf52840_hal::spi::Frequency::K500, nrf52840_hal::spim::MODE_0);

    let candriver = CanDriver::init(spi);
     

    loop {
        
    }
}

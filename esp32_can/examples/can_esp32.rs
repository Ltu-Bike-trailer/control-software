#![no_main]

use esp32_can as _;
use esp32_can::drivers::{
    can::{Mcp2515Driver, Mcp2515Settings,AcceptanceFilterMask, Bitrate, McpClock, OperationTypes, ReceiveBufferMode, SettingsCanCtrl, CLKPRE},
    message::CanMessage};

use embedded_hal::digital::{OutputPin, *};
use esp_println::*;

use esp_idf_svc::hal::{
    gpio::{InputPin, Level, Pin, PinDriver, Pins, Pull}, 
    peripherals::Peripherals, 
    spi::*,
    prelude::*};

use log::{info, debug, warn};

fn main() -> anyhow::Result<()>{
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
    
    let periph = Peripherals::take()?;
    let spi = periph.spi2;

    // PIN mapping for interfacing with the SpiDriver and SpiBusDriver
    let sck = periph.pins.gpio6;
    let sdo = periph.pins.gpio7;
    let sdi = periph.pins.gpio2; 
    
    let mut cs_pin = PinDriver::output(periph.pins.gpio18)?;
    cs_pin.set_state(PinState::High)?;
    
    let mut can_interrupt = PinDriver::input(periph.pins.gpio17)?;
    can_interrupt.set_pull(Pull::Up)?;
    // -----------------------------------------------------------------

    let spi_driver = SpiDriver::new::<SPI2>(
        spi, 
        sck, 
        sdo, 
        Some(sdi), 
        &SpiDriverConfig::new())?;
 
    const K125: Hertz = Hertz(125000);

    let bus_config = config::Config::new()
        .baudrate(K125)
        .data_mode(config::MODE_0);
    
    
    let spi_bus = SpiBusDriver::new(spi_driver, &bus_config)?;
   
    

    const CLKEN: bool = true;
    const OSM: bool = false;
    const ABAT: bool = false;

    const MASK_RXN: u16 = 0b1111_1111_1110_0000;
    const FILTER_RX0: u16 = 0x0;
    const FILTER_RX1: u16 = 0x1;
    const DEFAULT_FILTER_MASK: u16 = Mcp2515Settings::DEFAULT_FILTER_MASK;

    let canctrl_settings = SettingsCanCtrl::new(
        OperationTypes::Configuration,
        CLKEN,
        CLKPRE::DIV1,
        ABAT,
        OSM,
    );

    let can_settings = Mcp2515Settings::new(
        canctrl_settings,
        McpClock::MCP8,
        Bitrate::CAN125,
        0u8,
        ReceiveBufferMode::OnlyStandardId,
        AcceptanceFilterMask::new(DEFAULT_FILTER_MASK, DEFAULT_FILTER_MASK),
        AcceptanceFilterMask::new(DEFAULT_FILTER_MASK, DEFAULT_FILTER_MASK),
    );

    log::info!("Initializing MCP2515 Driver!");
    esp_println::println!("Initializing MCP2515 Driver!");
    let mut can_driver = Mcp2515Driver::init(spi_bus, cs_pin, can_interrupt, can_settings);

    Ok(())
}

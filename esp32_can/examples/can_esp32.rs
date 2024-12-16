use can_mcp2515::drivers::{can::*, message::*};
use lib::*;
use protocol::{sender::Sender, CurrentMeasurement, FixedLogType, MotorSubSystem, VelocityInfo, WriteType};

use embedded_hal::digital::{InputPin, OutputPin, *};
use embedded_can::{blocking::Can, StandardId};

use esp_idf_hal::delay::FreeRtos;
use esp_idf_hal::gpio::InterruptType;
use esp_idf_svc::hal::{
    gpio::{Level, PinDriver, Pull}, 
    peripherals::Peripherals, 
    spi::{config::DriverConfig, config::Config, Dma, SpiDriver, SpiBusDriver, config::MODE_0},
    prelude::*};

use log::{info, debug, warn, error};

fn main() -> anyhow::Result<(), anyhow::Error>{
    // It is necessary to call this function once. Otherwise some patches to the runtime
    // implemented by esp-idf-sys might not link properly. See https://github.com/esp-rs/esp-idf-template/issues/71
    esp_idf_svc::sys::link_patches();

    // Bind the log crate to the ESP Logging facilities
    esp_idf_svc::log::EspLogger::initialize_default();
   
    let periph = Peripherals::take()?;

    log::info!("Assigning GPIO peripheral pins now!");
    let spi = periph.spi2;

    // PIN mapping for interfacing with the SpiDriver and SpiBusDriver
    let sck = periph.pins.gpio6;
    let sdo = periph.pins.gpio7; // SDO = MOSI
    let sdi = periph.pins.gpio2; // SDI = MISO
    
    log::info!("Trying to initialize PinDriver for cs and interrupt pin for CAN driver!");
    
    let mut cs_pin = PinDriver::output(periph.pins.gpio10)?;
    cs_pin.set_level(Level::High)?;
    //cs_pin.set_state(PinState::High)?;
    
    
    let mut can_interrupt = PinDriver::input(periph.pins.gpio1)?;
    can_interrupt.set_pull(Pull::Up)?;
    can_interrupt.set_interrupt_type(InterruptType::NegEdge)?;
    
    //can_interrupt.enable_interrupt()?;
    // -----------------------------------------------------------------
    
    log::info!("Trying to initialize SpiDriver!");
    
    let spi_driver = SpiDriver::new(
        spi, 
        sck, 
        sdo, 
        Some(sdi), 
        &DriverConfig::default())?;
 
    //const K125: KiloHertz = 125.kHz();
    let bus_config = Config::new()
        //.baudrate(500u32.kHz().into())
        .baudrate(4.MHz().into())
        .data_mode(MODE_0);
    
    log::info!("Trying to initialize SpiBusDriver!");
    let spi_bus = SpiBusDriver::new(spi_driver, &bus_config)?;

    const CLKEN: bool = false;
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
    let mut can_driver = Mcp2515Driver::init(spi_bus, cs_pin, can_interrupt, can_settings);
    log::info!("MCP2515 Driver was successfully created!");

    //Ok(())
    let mut sender: Sender<10> = Sender::new();
    let dummy_id = StandardId::new(0x1).unwrap();
    let mut frame = CanMessage::new(embedded_can::Id::Standard(dummy_id), &[0x01, 0x02,
        0x03]).unwrap();
    sender.set_left_motor(1.1).unwrap();
    sender.set_left_motor(2.3).unwrap();
    let frme = sender.dequeue().unwrap();
    let frmeee = sender.dequeue().unwrap();
    //frame.print_frame();
    //let _ = can_driver.transmit(&frame);


     loop {
        
        let _ = can_driver.transmit(&frme);
        let _ = can_driver.transmit(&frmeee);

        //esp_idf_svc::hal::delay::FreeRtos::delay_ms(1000); // Send message every 1000ms
        /*
        if can_driver.interrupt_pin.is_low() {
            info!("GOT INTERRUPT!!!");
            while !can_driver.interrupt_is_cleared(){
                let interrupt_type = can_driver.interrupt_decode().unwrap();
                info!("type: {:?}", interrupt_type);
                if let Some(frame) = can_driver.handle_interrupt(interrupt_type) {
                    let msg_type = protocol::MessageType::try_from(&frame).unwrap();
                    info!("Can: {:?}", msg_type);
                }
            }
            if can_driver.interrupt_is_cleared() {
                log::info!("All interrupt is cleared!");
            } else {
                let interrupt_type = can_driver.interrupt_decode().unwrap();
                can_driver.handle_interrupt(interrupt_type);
            }
            //can_driver.interrupt_pin.enable_interrupt()?;
        } else {
            info!("Got nothing");
            FreeRtos::delay_ms(100);
        }
        */
        FreeRtos::delay_ms(500);

    }


}


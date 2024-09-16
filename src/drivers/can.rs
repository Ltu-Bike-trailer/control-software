#![no_main]
#![no_std]
#![allow(unused)]

use embedded_can::StandardId;
use nrf52840_hal::comp::OperationMode;
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::*;
use embedded_can::{Error, Frame, nb::Can};
use nb;

pub struct CanDriver<SPI: spi::SpiBus>{
    pub spi: SPI,
    //can: CanModule<>,
    //buffer: [u8; 8],
}

/*Instruction commands specific to the MCP2515 via SPI*/
#[repr(u8)]
enum InstructionCommand {
    Reset = 0xC0,
    Read = 0x03,
    Write = 0x02,
    Status = 0xB0,
    Modify = 0x05,
    LoadTX = 0x40,
    RTS = 0x87,
    ReadRx = 0x90,
}

/// The MCP2515 modes of operation. 
enum OperationTypes {
    Configuration,
    Normal,
    Sleep,
    ListenOnly,
    Loopback,
}

enum CanBuffer {
    TXB0, //TXB0 - TXB3 = transmit buffers for MCP2515 module-
    TXB1,
    TXB2,
    RXB0, // RXB0 and RXB1 = receive buffers for the MCP2515 module.
    RXB1,
}

enum RXMN {
    RXM0, //Acceptans mask 0
    RXM1, // Acceptans mask 1
}

enum RXFN {
    RXF0, // Acceptance Filter N, n = 0, 1..., 5
    RXF1,
    RXF2,
    RXF3,
    RXF4,
    RXF5,
}

#[derive(Debug)]
pub enum CanDriverError{
    SpiError,
    FrameError,
}

/*
 * Configure the MCP2515 Registers - by writing to its configuration registers via SPI.
 *  - Setup bit timing, filtering, and masks. 
 *  - Initialize and configure TX and RX buffers.
 *  - Enable or disable features such as interrupts as needed on the MCP2515.
 *
 *  The CAN Protocol Engine (reference MCP2515 Data-sheet): 
 *
 *  "The heart of the engine is the Finite State Machine
    (FSM). The FSM is a sequencer that controls the
    sequential data stream between the TX/RX shift
    register, the CRC register and the bus line. The FSM
    also controls the Error Management Logic (EML) and
    the parallel data stream between the TX/RX shift
    registers and the buffers. The FSM ensures that the
    processes of reception, arbitration, transmission and
    error-signaling are performed according to the CAN
    protocol. The automatic retransmission of messages
    on the bus line is also handled by the FSM".
 */
pub struct CanModule<CAN: Can>{
    engine: CAN,
    tx_buf: [u8; 8],
    rx_buf: [u8; 8],
}

pub struct CanMessage{
}

pub struct CanSettings{
    mode: OperationTypes,
    can_clk: u8,
    can_bitrate: u8,

}


impl Frame for CanMessage {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        Some(CanMessage::new())
    }

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
        None
    }

    fn is_extended(&self) -> bool {
        false
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> embedded_can::Id {
        embedded_can::Id::Standard((StandardId::ZERO))
    }

    fn dlc(&self) -> usize {
       1 
    }

    fn data(&self) -> &[u8] {
        &[]
    }
}

impl<CAN> Can for CanModule<CAN>
where 
    CAN: Can,
    CAN::Frame: Frame,
{
    type Frame = CanMessage;
    type Error = CAN::Error;
    
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        //TODO: - Need to implement the driver logic for transmit (logic in impl block for CanModule).
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        //TODO: - Need to implement the driver logic for receive (logic in impl block for CanModule). 
        Ok(CanMessage::new()) 
    }

}

impl<CAN: Can> CanModule<CAN>{
    //TODO: - Logic for driver goes here...
    fn setup_filtermask(&mut self){

    }

    fn set_mode(&mut self, mode: OperationTypes){
        match mode {
            OperationTypes::Configuration =>{
                /*
                 * Configuration mode is automatically selected
                 * after power-up, a reset or can be entered from any
                 * other mode by setting the CANTRL.REQOP bits to ‘100’.
                 */
            }
            OperationTypes::Normal =>{

            }
            OperationTypes::Loopback =>{

            }
            OperationTypes::ListenOnly =>{

            }
            OperationTypes::Sleep => {

            }
        }
    }

    /// The 'Bit Timing Logic' (BTL) - monitors the bus line input and handle the bus-related,
    /// bit timing according to the CAN protocol.
    fn setup_bit_timing(&mut self){
        //1. set CAN CLK
        //2. set CAN bitrate
    }

    /// Logic for transferring messages to the RXBn buffers only if acceptance filter criteria is
    /// met. Where the messages is handled in the Message Assembly Buffer (MAB).
    fn message_acceptance(){}
}

impl CanMessage {
    pub fn new() -> Self {
        CanMessage {  }
    }
}

/// This would e.g., take the `nrf52840_hal::spi::Spi` struct that uses the trait spi::SpiBus.
/// The SPI logic itself relies on the SPI implementation provided by another struct or module that already implements the SpiBus trait. In this case the `nrf52840_hal::spi::Spi`. 
impl<SPI: spi::SpiBus> CanDriver<SPI>{

    pub fn init(spi: SPI) -> Self{
        Self { spi }
    }

    //TODO: - Should I return: Result<[u8; 2], SPI::Error>>, or return buffer in struct directly?
    fn read(&mut self, data: &mut [u8]) {
        self.spi.read(data);
    }

    fn write(&mut self, data: &[u8]){
        /*example: self.spi.write(&[0b01000101,*byte]); */
        self.spi.write(data);
    }

    fn transfer(&mut self, read: &mut [u8], write: &[u8]){
        self.spi.transfer(read, write);
    }

    fn transfer_inplace(&mut self, data: &mut [u8]){
        self.spi.transfer_in_place(data);
    }

    fn flush(&mut self){
        self.spi.flush();
    }

}

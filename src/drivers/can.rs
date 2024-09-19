#![no_main]
#![no_std]
#![allow(unused)]

use digital::ErrorKind;
use embedded_can::StandardId;
use nrf52840_hal::comp::OperationMode;
use nrf52840_hal::{self as _, spi};
use nrf52840_hal::gpio::{Level, Port};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::*;
use embedded_hal::digital::OutputPin;
use embedded_hal::spi::SpiBus;
use embedded_can::{Error, Frame, nb::Can};
use nb;

//TODO: - Add CanModule struct type field to the CanDriver. 

pub struct CanDriver<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin>{
    pub spi: SPI,
    pub cs: PIN, 
}

/*Instruction commands specific to the MCP2515 via SPI*/
#[repr(u8)]
enum InstructionCommand {
    Reset = 0xC0,
    Read = 0x03,
    Write = 0x02,
    Status = 0xA0,
    Modify = 0x05,
    LoadTX = 0x40,
    RTS = 0x87,
    ReadRx = 0x90,
    RxStatus = 0xB0
}

/// The MCP2515 modes of operation.
/* REQOP: Request Operation Mode bits <2:0>
    000 = Set Normal Operation mode
    001 = Set Sleep mode
    010 = Set Loopback mode
    011 = Set Listen-only mode
    100 = Set Configuration mode
*/
#[repr(u8)]
pub enum OperationTypes {
    Configuration = 0b100,
    Normal = 0b000,
    Sleep = 0b001,
    ListenOnly = 0b011,
    Loopback = 0b010,
}

enum TXBN {
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

#[repr(u8)]
pub enum CLKPRE{
    DIV1 = 0b00, // System Clock/1
    DIV2 = 0b01, // System Clock/2
    DIV4 = 0b10, // System Clock/4
    DIV8 = 0b11, // System Clock/8
}


#[repr(u8)]
enum MCP2515Register{
    CANSTAT = 0x0E,    
    CANCTRL = 0x0F,
    CANINTE = 0x2B,
    CANINTF = 0x2C,

    BFPCTRL = 0x0C,
    TXRTSCTRL = 0x0D,
    TEC = 0x1C,
    REC = 0x1D,
    CNF1 = 0x2A,
    CNF2 = 0x29,
    CNF3 = 0x28,
    EFLG = 0x2D,

    TXB0CTRL = 0x30,
    TXB1CTRL = 0x40,
    TXB2CTRL = 0x50,

    RXB0CTRL = 0x60,
    RXB1CTRL = 0x70,
}


#[derive(Debug)]
pub enum CanDriverError{
    SpiError,
    FrameError,
}


pub struct CanMessage{
}

pub struct CanControllSettings{
    mode: OperationTypes,
    clken: bool,
    clkpre: CLKPRE,
    abat: bool, // ABAT: Abort All Pending Transmission bit
    osm: bool, // OSM: One-Shot Mode (1 = Enabled, 0 = disabled)
}

pub struct CanSettings{
    pub canctrl: CanControllSettings, 
    pub can_clk: u8,
    pub can_bitrate: u8,
}

impl CanControllSettings {
    pub fn new(mode: OperationTypes, clken: bool, clkpre: CLKPRE, abat: bool, osm: bool) -> Self{
        Self{
            mode, clken, clkpre, abat, osm
        }
    }

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


#[derive(Debug)]
pub enum CanError  {
 
}

impl embedded_can::Error for CanError{
    fn kind(&self) -> embedded_can::ErrorKind {
        embedded_can::ErrorKind::Other
    }
}

/// required trait methods goes here...
impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin> Can for CanDriver<SPI, PIN> {

    type Frame = CanMessage;
    type Error = CanError;
    
    fn transmit(&mut self, frame: &Self::Frame) -> nb::Result<Option<Self::Frame>, Self::Error> {
        //TODO: - Need to implement the driver logic for transmit (logic in impl block for CanModule).
        self.transmit_can(frame); // This is the custom driver logic for transmitting on the CAN bus. 
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        //TODO: - Need to implement the driver logic for receive (logic in impl block for CanModule). 
        self.receive_can(); // This is the custom driver logic for transmitting on the CAN bus. 
        Ok(CanMessage::new()) 
    }

}


/// Here goes custom logic for the CanMessage struct type
impl CanMessage {
    pub fn new() -> Self {
        CanMessage {  }
    }

    pub fn parse_frame(buffer: &[u8]) -> Self {
        //TODO: - Needs to parse the bit ranges and map each part to its correct message type 
        CanMessage{}
    }
}

/// This would e.g., take the `nrf52840_hal::spi::Spi` struct that uses the trait spi::SpiBus.
/// The SPI logic itself relies on the SPI implementation provided by another struct or module that already implements the SpiBus trait. In this case the `nrf52840_hal::spi::Spi`. 
impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin> CanDriver<SPI, PIN>{

    pub fn init(spi: SPI, cs: PIN, can_settings: CanSettings) -> Self{
        let mut driver = Self{spi, cs};
        driver.setup_configuration(can_settings);
        driver    
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

    fn setup_filtermask(&mut self){
        
    }

    fn read_register(&mut self, reg: MCP2515Register) -> Result<u8, SPI::Error>{
        let mut read_buf: [u8; 2] = [0; 2];
        let read_msg: [u8; 2] = [InstructionCommand::Read as u8, reg as u8];
        
        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &read_msg);
        self.cs.set_high();
        Ok(read_buf[1])
    }

    fn write_register(&mut self, reg: MCP2515Register, data: u8) -> Result<(), SPI::Error>{
        let reg_address: u8 = reg as u8;
        let maskbit: u8; 
        // Expected format: [u8; N] --> [Instruction byte, Address byte, Data byte]
        let byte_msg: [u8; 3] = [InstructionCommand::Write as u8, reg_address, data];

        self.cs.set_low();
        self.spi.write(&byte_msg);
        self.cs.set_high();
        Ok(())
    }

    pub fn set_canctrl_register(&mut self, canctrl_settings: CanControllSettings) -> u8 {
        let mut canctrl_byte = 0u8; // (1): data_byte |= (reg_value << reg_pos)
        let mode_value: u8 = canctrl_settings.mode as u8; //REQOP[2:0] (bits 7-5)
        let prescale: u8 = canctrl_settings.clkpre as u8; //CLKPRE[1:0] (bits 1-0)
        let clk_enabled = canctrl_settings.clken;
       
        defmt::println!("1. canctrl_byte: 0b{:08b}", canctrl_byte);
        canctrl_byte |= mode_value << 5; //(2)
        defmt::println!("2. canctrl_byte after mode select: 0b{:08b}", canctrl_byte);

        if (canctrl_settings.abat == true){
            canctrl_byte |= 1 << 4;
        }

        if (canctrl_settings.osm == true){
            canctrl_byte |= 1 << 3;
        }

        if (clk_enabled == true){
            canctrl_byte |= 1 << 2;
        }

        canctrl_byte |= prescale; //(3)
        defmt::println!("3. canctrl_byte after prescale select: 0b{:08b}", canctrl_byte);

        // 1. canctrl_byte = 0b0000_0000
        // 2. canctrl_byte = 0b0000_0000 OR (0b0000_0xxx << 5) = 0bxxx0_0000
        // 3. 0bxxx0_0000 OR 0b0000_00zz = 0bxxx0_00zz

        return canctrl_byte;
    }

    /// This would apply and configure the Can controller settings for the module.
    pub fn setup_configuration(&mut self, can_settings: CanSettings){
        let canctrl_byte = self.set_canctrl_register(can_settings.canctrl);
        
        defmt::info!("CANCTRL register settings value: 0b{:08b}", canctrl_byte);
        self.write_register(MCP2515Register::CANCTRL, canctrl_byte);
    
        //TODO: - Write other relevant register settings below... 
    }


    fn transmit_can(&mut self, can_msg: &CanMessage){
        //Pull the CS pin LOW.
        self.cs.set_low().map_err(|_| CanDriverError::SpiError);
        
        let instruction = InstructionCommand::Write as u8;
        self.spi.write(&[instruction]);

        self.spi.write(can_msg.data());

        // Deselect MCP2515 module by pulling CS pin high.
        self.cs.set_high();
    }

    fn receive_can(&mut self){

    }

    /// The 'Bit Timing Logic' (BTL) - monitors the bus line input and handle the bus-related,
    /// bit timing according to the CAN protocol.
    fn setup_bit_timing(&mut self){
        //1. set CAN CLK
        //2. set CAN bitrate
    }

    /// Logic for transferring messages to the RXBn buffers only if acceptance filter criteria is
    /// met. Where the messages is handled in the Message Assembly Buffer (MAB).
    fn message_acceptance(){

    }

    fn initiate_transmission(&mut self, instruction: InstructionCommand, address_byte: u8){
        // 1. Need to set the TXBnCTRL.TXREQ bit for each buffer
        // 2. Configure using the TXRTSCTRL register (need to be in 'configuration mode').
        // Format: [u8; N] --> [Instruction byte, Address byte, Data byte]
        
        //self.cs.set_low();
        //let byte_msg = self.create_instruction(instruction, address_byte, 0b0100_0100);
        //self.spi.write(&byte_msg);
        
    }


}

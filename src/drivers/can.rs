//! MCP2515 driver - for writing CAN frame messages over the CAN bus. 
//!
//!
//! This driver, is still under progress...
//!
//! ## Usage
//!
//! ```ignore
//!
//! ```
#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]

use core::{borrow::Borrow, str, usize};
use cortex_m::asm as _;
use digital::ErrorKind;
use embedded_can::{Id, StandardId};
use nrf52840_hal::comp::OperationMode;
use nrf52840_hal::{self as _, spi};
use nrf52840_hal::gpio::{Level, Port};
use nrf52840_hal::gpiote::{Gpiote, GpioteInputPin};
use nrf52840_hal::pac::nfct::framestatus::RX;

use cortex_m_rt::entry;
use defmt::{unwrap, Format};
use defmt_rtt as _;

use embedded_hal::*;
use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal::spi::SpiBus;
use embedded_can::{Error, Frame, blocking::Can};
use nb;

use super::message::CanMessage;


/// The MCP2515 driver struct. 
pub struct CanDriver<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> {
    /// Uses the SpiBus trait from embedded_hal::spi::SpiBus
    ///
    /// This field, can write, read over the Spi bus. 
    pub spi: SPI,
    /// Chip-Select Pin, for driving the pin from high to low, during SPI transactions to the
    /// MCP2515.  
    pub cs: PIN,
    /// Interrupt input pin, that would generate a GPIOTE event on falling edge. 
    /// Going from high to low. 
    pub interrupt_pin: PININT,
    /// This is MCP2515 specific settings, for applying settings such as: 
    /// - CANCTRL register bit settings. 
    /// - MCP clock speed.
    /// - CAN bitrate.
    /// - Enabling of interrupts.
    /// - Receive buffer mode (e.g., turn mask/filters off, Only StandardID, etc...).
    pub can_settings: CanSettings,
}

///Instruction commands specific to the MCP2515 via SPI.
#[repr(u8)]
enum InstructionCommand {
    /// Resets the internal registers to the default state. 
    Reset = 0xC0,
    /// Reads data from the register beginning at selected address. 
    Read = 0x03,
    /// Writes data to the register beginning at the selected address.
    Write = 0x02,
    /// Quick polling command that reads several status bits for transmit and receive functions. 
    Status = 0xA0, //1010_0000
    /// Allow the user to set or clear individual bits in a particular register. 
    Modify = 0x05,
    /// Loading a transmit buffer, this instruction reduce the overheaad of a normal WRITE command. 
    LoadTX = 0x40,   
    /// Request-To-Send - begin message transmission sequence for any of the transmit buffers. 
    RTS = 0x80,      // 10000nnn
    /// Reading the specific receive buffer 0.
    ReadRxb0 = 0x90, // 0b10010000
    /// Reading the specific receive buffer 1. 
    ReadRxb1 = 0x94, // 0b10010100
    /// Quick polling command that indicates filter match and message type of receive message. 
    RxStatus = 0xB0,
}

/// The MCP2515 modes of operation.
///REQOP: Request Operation Mode bits <2:0>
///    000 = Set Normal Operation mode
///    001 = Set Sleep mode
///    010 = Set Loopback mode
///    011 = Set Listen-only mode
///    100 = Set Configuration mode
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OperationTypes {
    Configuration = 0b100,
    Normal = 0b000,
    Sleep = 0b001,
    ListenOnly = 0b011,
    Loopback = 0b010,
}

/// Start address for the Transmit buffer N. 
#[repr(u8)]
enum TXBN {
    TXB0 = 0x30, 
    TXB1 = 0x40,
    TXB2 = 0x50,
}

/// Denotes the valid RX buffers.
#[repr(u8)]
pub enum RXBN {
    RXB0 = 0x60, 
    RXB1 = 0x70,
}

/// Settings related to register RXBnCTRL.RXM bits.
/// For setting up, receive buffer operating mode. 
#[derive(Clone, Copy)]
pub enum ReceiveBufferMode{
    FilterOffReceiveAny,
    OnlyExtendedId,
    OnlyStandardId,
    ReceiveAny,
}

/// Receive Buffer Mask N.
enum RXMN {
    RXM0, 
    RXM1,
}

/// Receive Buffer Filter N.
#[repr(u8)]
enum RXFN {
    RXF0 = 0b0001_0000,
    RXF1,
    RXF2,
    RXF3,
    RXF4,
    RXF5,
}

/// Bits for setting the clock prescale. 
#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CLKPRE {
    /// System Clock/1
    DIV1 = 0b000,
    ///System Clock/2
    DIV2 = 0b001, 
    /// System Clock/4
    DIV4 = 0b010,
    /// System Clock/8
    DIV8 = 0b011,
}

/// MCP2515 module register upper address. 
#[repr(u8)]
enum MCP2515Register {
    /// CAN Status Register. 
    CANSTAT = 0x0E,
    /// CAN Control Register. 
    CANCTRL = 0x0F,
    /// CAN Interrupt Enable Register.
    CANINTE = 0x2B,
    /// CAN Interrupt Flag Register. 
    CANINTF = 0x2C, 

    /// RXnBF PIN control and status register. 
    BFPCTRL = 0x0C,
    /// TXnRTS PIN control and status register. 
    TXRTSCTRL = 0x0D,
    /// Transmit Error Counter Register. 
    TEC = 0x1C,
    /// Receive Error Counter Register. 
    REC = 0x1D,
    /// Configuration register 1.
    CNF1 = 0x2A,
    /// Configuration register 2.
    CNF2 = 0x29,
    /// Configuration register 3
    CNF3 = 0x28,
    /// Error Flag Register. 
    EFLG = 0x2D,

    /// Transmit buffer 0 Control Register. 
    TXB0CTRL = 0x30,
    /// Transmit buffer 1 Control Register. 
    TXB1CTRL = 0x40,
    /// Transmit buffer 2 Control Register. 
    TXB2CTRL = 0x50, 

    /// Receive buffer 0 control register.
    RXB0CTRL = 0x06,
    /// Receive buffer 1 control register.
    RXB1CTRL = 0x07,

    /// Mask 0 - Standard Identifier Register High.
    RXM0SIDH = 0x20,
    /// Mask 0 - Standard Identifier Register Low.
    RXM0SIDL = 0x21,
    /// Mask 1 - Standard Identifier Register High.
    RXM1SIDH = 0x24,
    /// Mask 1 - Standard Identifier Register Low.
    RXM1SIDL = 0x25,

    /// Filter 0 - Standard Identifier Register High. 
    RXF0SIDH = 0x00,
    /// Filter 0 - Standard Identifier Register Low. 
    RXF0SIDL = 0x01,

    /// Filter 1 - Standard Identifier Register High. 
    RXF1SIDH = 0x04,
    /// Filter 1 - Standard Identifier Register Low. 
    RXF1SIDL = 0x05,

    /// Filter 2 - Standard Identifier Register High. 
    RXF2SIDH = 0x08,
    /// Filter 2 - Standard Identifier Register Low. 
    RXF2SIDL = 0x09,

    /// Filter 3 - Standard Identifier Register High. 
    RXF3SIDH = 0x10,
    /// Filter 3 - Standard Identifier Register Low. 
    RXF3SIDL = 0x11,
    /// Filter 4 - Standard Identifier Register High. 
    RXF4SIDH = 0x14,
    /// Filter 4 - Standard Identifier Register Low. 
    RXF4SIDL = 0x15,
    /// Filter 5 - Standard Identifier Register High. 
    RXF5SIDH = 0x18,
    /// Filter 5 - Standard Identifier Register Low. 
    RXF5SIDL = 0x19,
}

/// MCP2515 driver specific error types. 
#[derive(Debug)]
pub enum CanDriverError {
    /// Error related to the SPI bus
    SpiError,
    
    /// CAN Frame related error. 
    FrameError,
}

/// Associated registers to a specific TX/RX buffer. Going from high to low addresses offsets. 
#[repr(u8)]
pub enum TxRxbn {
    
    /// TXBNCTRL/RXBNCTRL
    CTRL = 0x0,
    /// TXBNSIDH/RXBNSIDH
    SIDH = 0x1,
    /// TXBNSIDL/RXBNSIDL
    SIDL = 0x2,
    /// TXBNEID8/RXBNEID8 - Extended ID
    EID8 = 0x3,
    /// TXBNEID0/RXBNEID0 - Extended ID
    EID0 = 0x4,
    /// TXBNDLC/RXBNDLC - Data Length Code
    DLC = 0x5,
    /// TXBND0/RXBND0 - Data byte 0
    D0 = 0x6,
    /// TXBND1/RXBND1 - Data byte 1
    D1 = 0x7,
    /// TXBND2/RXBND2 - Data byte 2
    D2 = 0x8,
    /// TXBND3/RXBND3 - Data byte 3
    D3 = 0x9,
    /// TXBND4/RXBND4 - Data byte 4
    D4 = 0x10,
    /// TXBND5/RXBND5 - Data byte 5
    D5 = 0x11,
    /// TXBND6/RXBND6 - Data byte 6
    D6 = 0x12,
    /// TXBND7/RXBND7 - Data byte 7
    D7 = 0x13,
}

/// MCP2515 CANCTRL register setting bits.  
#[derive(Clone, Copy)]
pub struct CanControllSettings {
    mode: OperationTypes,
    clken: bool,
    clkpre: CLKPRE,
    abat: bool, // ABAT: Abort All Pending Transmission bit
    osm: bool,  // OSM: One-Shot Mode (1 = Enabled, 0 = disabled)
}

/// MCP2515 register CANINTE.
#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CanInte {
    /// Message Error Interrupt bit - during message reception/transmission.
    MERRE = 0b1000_0000,
    
    /// Wake-up Interrupt bit - on CAN bus activity. 
    WAKIE = 0b0100_0000,
    
    /// Error Interrupt bit - on EFLG error condition change. 
    ERRIE = 0b0010_0000,
    
    /// Transmit buffer 2 empty interrupt bit - when TXB2 becomes empty (sent message).
    TX2IE = 0b0001_0000,
    
    /// Transmit buffer 1 empty interrupt bit - when TXB1 becomes empty (sent message).
    TX1IE = 0b0000_1000,
    
    /// Transmit buffer 0 empty interrupt bit - when TXB0 becomes empty (sent message).
    TX0IE = 0b0000_0100,
    
    /// Receive buffer 1 full interrupt bit - message received in RXB1.
    RX1IE = 0b0000_0010,

    /// Receive buffer 0 full interrupt bit - message received in RXB0.
    RX0IE = 0b0000_0001,
}

/// MCP2515 register CANSTAT.ICOD - Interrupt Flag Code bits.
#[repr(u8)]
#[derive(Debug, Format)]
pub enum InterruptFlagCode {
    NoInterrupt = 0b000,
    ErrorInterrupt = 0b001,
    WakeUpInterrupt = 0b010,
    TXB0Interrupt = 0b011,
    TXB1Interrupt = 0b100,
    TXB2Interrupt = 0b101,
    RXB0Interrupt = 0b110,
    RXB1Interrupt = 0b111,
}

/// MCP2515 Clock speed in Mega-hertz.
#[derive(Clone, Copy)]
pub enum McpClock {
    /// 16 MHz.
    MCP16,
    
    /// 8 MHz.
    MCP8,
}

/// Transmission rate in kBPS. 
#[derive(Clone, Copy)]
pub enum CanBitrate {
    /// 125 kBPS
    CAN125,
}

#[derive(Clone, Copy)]
/// Acceptable frame ID that the RX0 or RX1 buffer should accept on the CAN bus.
/// This would determine if a message should be loaded into either of the receive
/// buffers.
///
/// - rx_mask - the mask determine which bits in the identifier that should be examined 
/// with the filter. 
///
/// - acceptance filter - are compared with identifier fields of the message. 
pub struct AcceptanceFilterMask{
    rx_mask: u16,
    acceptance_filter: u16,
}

/// Struct for neccessary settings for setting up the MCP2515 CAN module. 
#[derive(Clone, Copy)]
pub struct CanSettings {
    pub canctrl: CanControllSettings,
    pub mcp_clk: McpClock, // Clock Frequency for MCP2515.
    pub can_bitrate: CanBitrate,
    pub interrupts: u8, 
    pub rxm_mode: ReceiveBufferMode,
    pub rx0_filtermask: AcceptanceFilterMask,
    pub rx1_filtermask: AcceptanceFilterMask,
}

impl AcceptanceFilterMask{
    /// Creates a neew Acceptance Filter and mask, for constraining acceptable frame IDs, 
    /// that should be accepted/ignored. 
    pub fn new(mask: u16, filter: u16) -> Self{
        //TODO: - Parse the RXM mode to define the correct filter (Standard vs Extended ID).
        let acceptance_id = (StandardId::new(filter).unwrap().as_raw() << 5);
        let rx_mask_example = 0b1111_1111_1110_0000 as u16; // All 11 bit StandardID.

        defmt::info!("Creating new message acceptance filter({:016b}) and mask({:016b})", acceptance_id, mask);
        Self { rx_mask: mask, acceptance_filter: acceptance_id }
    }
}

impl CanControllSettings {
    /// Creates a new MCP2515 module specific settings instance struct. 
    ///
    /// ## NOTE 
    ///
    /// This will set: 
    /// - Operation mode (Configuration, Normal, Loopback, etc...). 
    /// - 'Abort All Pending Transmission' (ABAT), 'One-Shot Mode' (transmit one time only).
    /// - CLKOUT pin enable/disable (CLKEN).
    /// - CLKOUT Pin Prescler (CLKPRE).
    pub fn new(mode: OperationTypes, clken: bool, clkpre: CLKPRE, abat: bool, osm: bool) -> Self {
        Self {
            mode,
            clken,
            clkpre,
            abat,
            osm,
        }
    }
}

impl Default for CanSettings {
    fn default() -> CanSettings {
        let canctrl_settings = CanControllSettings::new(OperationTypes::Configuration, true, CLKPRE::DIV1, false, false);
        const DEFAULT_FILTER_MASK: u16 = 0u16;
        Self {
            canctrl: canctrl_settings,
            mcp_clk: McpClock::MCP8,
            can_bitrate: CanBitrate::CAN125,
            interrupts: 0u8,
            rxm_mode: ReceiveBufferMode::OnlyStandardId,
            rx0_filtermask: AcceptanceFilterMask{rx_mask: DEFAULT_FILTER_MASK, acceptance_filter: DEFAULT_FILTER_MASK},
            rx1_filtermask: AcceptanceFilterMask { rx_mask: DEFAULT_FILTER_MASK, acceptance_filter: DEFAULT_FILTER_MASK},
        }
    }
}

impl CanSettings {
    /// Enable interrupt in the MCP2515.CANINTE register.
    /// This works by creating an interrupt mask by setting the 
    /// associated CANINTE bit position associated with the specific interrupt. 
    fn enable_interrupt(mut self, interrupt_type: CanInte) -> Self {
        self.interrupts |= interrupt_type as u8;
        self
    }

    /// Enables interrupt(s) in the MCP2515.CANINTE register.
    /// This works by creating an interrupt mask by setting the 
    /// associated CANINTE bit(s) associated with the specific interrupt.
    fn enable_interrupts(mut self, interrupt_types: &[CanInte]) -> Self {
        interrupt_types
            .iter()
            .for_each(|el| self.interrupts |= *el as u8);
        self
    }
}

impl TryFrom<u8> for InterruptFlagCode {
    type Error = &'static str;

    fn try_from(value: u8) -> Result<Self, Self::Error> {
        match value {
            0b000 => Ok(InterruptFlagCode::NoInterrupt),
            0b001 => Ok(InterruptFlagCode::ErrorInterrupt),
            0b010 => Ok(InterruptFlagCode::WakeUpInterrupt),
            0b011 => Ok(InterruptFlagCode::TXB0Interrupt),
            0b100 => Ok(InterruptFlagCode::TXB1Interrupt),
            0b101 => Ok(InterruptFlagCode::TXB2Interrupt),
            0b110 => Ok(InterruptFlagCode::RXB0Interrupt),
            0b111 => Ok(InterruptFlagCode::RXB1Interrupt),
            _ => Err("No such interrupt flag code exist!"),
        }
    }
}

#[repr(u8)]
#[derive(Clone, Copy, Debug, Format)]
pub enum EFLG{
    /// Receive buffer 1 overflow flag.  
    RX1OVR = 7,
    
    /// Receive buffer 0 overflow flag.  
    RX0OVR = 6,
    
    /// Bus-Off Error Flag bit (TEC = 255)
    TXBO = 5,
    
    /// Transmit Error-Passive Flag bit (TEC >= 128)
    TXEP = 4,
    
    /// Receive Error-Passive Flag bit (REX >= 128)
    RXEP = 3,
    
    /// Transmit Error Warning Flag bit (TEC >= 96)
    TXWAR = 2,
    
    /// Receive Error Warning Flag bit (REC >= 96)
    RXWAR = 1,
    
    /// Error Warning Flag bit (TXWAR or RXWAR = 1)
    EWARN = 0,
    
    /// Just for debugging purpose. No known EFLG type. 
    UNKNOWN = 0xFF,
}

#[derive(Debug)]
/// Specific Can Error types. Still under progress!
pub enum CanError{
    /// Whenever a Transmission error occurs. 
    TransmissionError,
    
    ///Receive buffer 0 overflow error.  
    RX0Overflow,
   
    /// Receive buffer 1 overflow error. 
    RX1Overflow,
    
    /// Message error interrupt, when a can frame is faulty. 
    MessageErrorInterrupt,
    
    /// Can module receive error. 
    ReceiveError,
    
    /// Custom error type for whenever decoding a can frame isn't successful. 
    DecodeError,
}


impl EFLG{
    /// Return the appropriate EFLG type based on provided u8 value. 
    fn from_u8(value: u8) -> Self {
        match value {
            0b000 => Self::EWARN,            
            0b001 => Self::RXWAR,
            0b010 => Self::TXWAR,
            0b011 => Self::RXEP,
            0b100 => Self::TXEP,
            0b101 => Self::TXBO,
            0b110 => Self::RX0OVR,
            0b111 => Self::RX1OVR,
            _ => unreachable!(),
        }
    }
}

impl embedded_can::Error for CanError {
    /// Returns the ErrorKind (CAN error) types.  
    fn kind(&self) -> embedded_can::ErrorKind {
        match self {
            CanError::TransmissionError => embedded_can::ErrorKind::Other, // or an appropriate kind
            CanError::RX0Overflow => embedded_can::ErrorKind::Other,
            CanError::RX1Overflow => embedded_can::ErrorKind::Other,
            CanError::MessageErrorInterrupt => embedded_can::ErrorKind::Other,
            CanError::ReceiveError => embedded_can::ErrorKind::Other,
            CanError::DecodeError => embedded_can::ErrorKind::Other,

        }
    }
}


impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> Can
    for CanDriver<SPI, PIN, PININT>
{
    type Error = CanError;
    type Frame = CanMessage;
    
    /// Calls the driver transmit_can method for transmitting data to the TX buffer. 
    ///
    /// ## NOTE
    ///
    /// This method is required for the embedded_can::blocking::Can trait.  
    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        //defmt::println!("Executing 'self.transmit_can(frame)");
        //self.read_status();
        self.transmit_can(frame); // This is the custom driver logic for transmitting on the CAN bus. 
        Ok(())
    }

    /// Calls the driver receive_can method for reading appropriate RX buffer.
    /// ## NOTE
    ///
    /// This method is required for the embedded_can::blocking::Can trait.
    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        let frame = self.receive_can(); 
        Ok(frame)
    }
}

impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> CanDriver<SPI, PIN, PININT> {
    
    /// Initialize and creates a new driver instance. Based on provided CanSettings argument.
    ///
    /// ## PARAM: 
    ///
    /// - SPI: SpiBus trait for exclusive ownership - SPI bus pins.
    /// - PIN: embedded_hal::digital::OutputPin - Chipselect (CS) pin. 
    /// - PININT: embedded_hal::digital::InputPin - for MCP2515 INT signals.  
    pub fn init(spi: SPI, cs: PIN, interrupt_pin: PININT, can_settings: CanSettings) -> Self {
        let initial_settings = can_settings.borrow();
        let mut driver = Self {
            spi,
            cs,
            interrupt_pin,
            can_settings,
        };

        driver
            .setup_configuration(initial_settings) // Setup CANCTRL register.
            .setup_bitrate() // Setup bitrate and clk freq, in registers: CNF1, CNF2, & CNF3.
            .setup_interrupt(initial_settings) // Setup interrupts in CANINTE register.
            .tx_pending(false) // CLEAR the TXREQ bits indicating the tx buffer is not pending before writing.
            .set_rxm_mode(initial_settings.rxm_mode)
            .filter_message_id(RXBN::RXB0, initial_settings.rx0_filtermask.rx_mask, initial_settings.rx0_filtermask.acceptance_filter)
            .filter_message_id(RXBN::RXB1, initial_settings.rx1_filtermask.rx_mask, initial_settings.rx1_filtermask.acceptance_filter);

        let received_reg = driver.read_register(MCP2515Register::TXB0CTRL, 0x00).unwrap();
        let rts_reg = driver.read_register(MCP2515Register::TXRTSCTRL, 0x00).unwrap();
        
        defmt::println!("TXREQ register: {:08b}", received_reg);
        defmt::println!("TXRTSCTRL register: {:08b}", rts_reg);
     
        driver.activate_canbus();

        if (driver.can_settings.canctrl.mode != OperationTypes::Normal){
            defmt::panic!("Operational Mode, has to be in Normal Mode, before using CAN Bus!");
        }
        
        driver    
    }

    /// SPI read. 
    fn read(&mut self, data: &mut [u8]) {
        self.spi.read(data);
    }

    /// SPI write. 
    fn write(&mut self, data: &[u8]) {
        self.spi.write(data);
    }

    /// Perform spi transfer by writing to MOSI and received on MISO. 
    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) {
        self.cs.set_low();
        self.spi.transfer(read, write);
        self.cs.set_high();
    }

    /// reading a MCP2515 register address + register offset (optional).
    ///
    /// #NOTE 
    ///
    /// Perform spi transfer by writing to MOSI and received on MISO. 
    fn read_register(&mut self, reg: MCP2515Register, reg_offset: u8) -> Result<u8, SPI::Error> {
        const DONTCARES: u8 = 0u8;
        let mut read_buf: [u8; 3] = [0; 3];
        let register: u8 = reg as u8 + reg_offset;
        let mut read_msg: [u8; 3] = [InstructionCommand::Read as u8, register as u8, DONTCARES];

        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &read_msg);
        self.cs.set_high();

        //defmt::info!("Read instruction, Sent (MOSI): {:08b}, Received (MISO):
        // {:08b}", read_msg, read_buf);
        Ok(read_buf[2])
    }

    /// Write u8 data, to a specific MCP2515 register address. 
    ///
    /// ## NOTE
    ///
    /// Expected format: [u8; N] --> [Instruction byte, Address byte, Data
    /// byte]. For writing to a specific register
    fn write_register(&mut self, reg: MCP2515Register, data: u8) -> Result<(), SPI::Error> {
        let reg_address: u8 = reg as u8;
        let mut byte_msg: [u8; 3] = [InstructionCommand::Write as u8, reg_address, data];

        self.cs.set_low();
        self.spi.write(&byte_msg);
        self.cs.set_high();
        //defmt::info!("Write instruction, Sent (MOSI): {:08b}, MISO = High-Impedance",
        // byte_msg);
        Ok(())
    }

    /// Changing settings, is mostly used for changing the operating mode of the MCP2515. 
    /// E.g., changing from 'Configuration' to 'Normal' mode. 
    fn change_settings(&mut self, settings: CanSettings) {
        self.can_settings.canctrl = settings.canctrl;

        let bitmask_canctrl = self.get_canctrl_mask(self.can_settings.canctrl);
        self.write_register(MCP2515Register::CANCTRL, bitmask_canctrl);
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        
        //defmt::info!("Bitmask for changing settings: {:08b}", bitmask_canctrl);         
        //defmt::info!("After change settings in CANSTAT: {:08b}", canstat);
    }

    /// This is just for debugging and testing the SPI, for testing read and write for MCP2515.
    fn transfer_test(&mut self, register: MCP2515Register) {
        let mut read_buf: [u8; 3] = [0; 3];
        let write_buf: [u8; 3] = [InstructionCommand::Read as u8, register as u8, 0x00];
        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &write_buf);
        self.cs.set_high();
        defmt::info!("Sent: {:08b}, Received: {:08b}", write_buf, read_buf);
    }

    /// Loopback test, that sets the mode to loopback, and LOAD TX and READ RX
    /// buffer.
    /// -------------------------------------------------------------------------*
    /// (1). Change mode by writing to CANCTRL register with the settings
    /// bitmask. (2). Create a new CAN frame and convert to byte array.
    /// (3). Perform 'Load TX buffer' instruction, loading the CAN byte frame.
    /// (4). Initiate transmission by setting the TXREQ bit (TXBnCTRL[3])
    /// for each buffer. (5). Poll the target RX buffer, checking if message
    /// has been received. Then read RX!
    /// -------------------------------------------------------------------------*    
    /// Example format of byte message: ...
    /// ... let byte_msg: [u8; 3] = [InstructionCommand::Write as u8,
    /// MCP2515Register::TXB0CTRL as u8, data];
    pub fn loopback_test(&mut self, can_msg: CanMessage) {
        defmt::info!("Loopback Test:");
        let mut can_settings = CanSettings::default();
        let canctrl_settings =
            CanControllSettings::new(OperationTypes::Loopback, false, CLKPRE::DIV1, false, false);
        can_settings.canctrl = canctrl_settings;

        self.change_settings(can_settings);

        let dummy_data = "dummy".as_bytes();
        defmt::println!("Dummy data length: {:?}", dummy_data.len());

        let dummy_id = StandardId::new(0x1).unwrap();
        let mut frame = CanMessage::new(embedded_can::Id::Standard(dummy_id), &dummy_data).unwrap();
        //let mut frame = CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO),
        // dummy_data).unwrap();

        //self.read_status();
        self.load_tx_buffer(TXBN::TXB0, can_msg);
        //self.load_tx_buffer(TXBN::TXB0, frame);
        //self.poll_rx(RXBN::RXB0);
        //self.rx_status();
    }

    /// The 'poll_rx' method was for reading the RX buffers for a loopback test without interrupts. 
    fn poll_rx(&mut self, rx: RXBN) {
        loop {
            let rx_status = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
            match rx {
                RXBN::RXB0 => {
                    if (rx_status & (1 << 0) != 0) {
                        defmt::info!("Frame received in RXB0");
                        let received_frame = self.read_rx_buffer(RXBN::RXB0).unwrap();
                        defmt::info!("Received frame: {:08b}", received_frame);
                        break;
                    }
                }
                RXBN::RXB1 => {
                    //0b0000_0010 AND 0b0000_0100
                    if (rx_status & (1 << 1) != 0) {
                        defmt::info!("Frame received in RXB1");
                        let received_frame = self.read_rx_buffer(RXBN::RXB1).unwrap();
                        defmt::info!("Received frame: {:08b}", received_frame);
                        break;
                    }
                }
            }
        }
    }

    /// Method for applying the CAN controller settings by getting the bitmask.
    /// Format for setting bit: N = N | (1<<k).
    pub fn get_canctrl_mask(&mut self, canctrl_settings: CanControllSettings) -> u8 {
        let mut canctrl_byte = 0u8; // (1): data_byte |= (reg_value << reg_pos)
        let mode_bits: u8 = (canctrl_settings.mode as u8) << 5; //REQOP[2:0] (bits 7-5)
        let prescale: u8 = canctrl_settings.clkpre as u8; //CLKPRE[1:0] (bits 1-0)
        let clk_enabled = canctrl_settings.clken;

        canctrl_byte |= mode_bits; //(2)
        defmt::println!("Mode bitmask: 0b{:08b}", mode_bits);
        defmt::println!(
            "Mode bitmask applied to CANCTRL register: 0b{:08b}",
            canctrl_byte
        );

        if (canctrl_settings.abat == true) {
            canctrl_byte |= 1 << 4;
        }

        if (canctrl_settings.osm == true) {
            canctrl_byte |= 1 << 3;
        }

        if (clk_enabled == true) {
            canctrl_byte |= 1 << 2;
        } else {
            canctrl_byte &= !(1 << 2);
        }

        canctrl_byte |= prescale; //(3)
        defmt::println!(
            "3. CANCTRL bitmask after applying settings: 0b{:08b}",
            canctrl_byte
        );

        return canctrl_byte;
    }

    /// Read Status Instruction. 
    ///
    /// ## NOTE 
    ///
    /// Will show:
    ///
    /// RX0IF (CANINTF[0]) - bit 0
    /// RX1IF (CANINTF[1]) - bit 1
    /// TXREQ (TXB0CNTRL[3]) - bit 2
    /// TX0IF (CANINTF[2]) - bit 3
    /// TXREQ (TXB1CNTRL[3]) - bit 4
    /// TX1IF (CANINTF[3]) - bit 5
    /// TXREQ (TXB2CNTRL[3]) - bit 6
    /// TX2IF (CANINTF[4]) - bit 7
    fn read_status(&mut self) {
        let instruction_msg: [u8; 3] = [InstructionCommand::Status as u8, 0x00, 0x00];
        let mut data_out: [u8; 3] = [0; 3];
        self.cs.set_low();
        self.spi.transfer(&mut data_out, &instruction_msg);
        self.cs.set_high();
        defmt::info!("Read Status: {:08b}", data_out);
    }

    /// RX Status Instruction. 
    ///
    /// ## NOTE 
    ///
    /// "Quick polling command that indicates filter match and message
    ///  type (standard, extended and/or remote) of received message"
    fn rx_status(&mut self) {
        let instruction_msg: [u8; 3] = [InstructionCommand::RxStatus as u8, 0x00, 0x00];
        let mut data_out: [u8; 3] = [0; 3];
        self.cs.set_low();
        self.spi.transfer(&mut data_out, &instruction_msg);
        self.cs.set_high();
        defmt::info!(
            "RX status, Sent (MOSI): {:08b}, Received (MISO): {:08b}",
            instruction_msg,
            data_out
        );
    }

    /// RTS - Request-To-Send instruction. 
    /// This is called after a Load TX instruction has been runned.
    /// It will work, when TXREQ bits are set to 1, indicating TX is pending.  
    fn request_to_send(&mut self, buffer: TXBN) {
        let rts_instruction: u8 = match buffer {
            TXBN::TXB0 => (InstructionCommand::RTS as u8 | 0x01),
            TXBN::TXB1 => (InstructionCommand::RTS as u8 | 0x02),
            TXBN::TXB2 => (InstructionCommand::RTS as u8 | 0x04),
        };

        //defmt::println!("Request-To-Send instruction: {:08b}", rts_instruction); 
        let mut instruction_msg: [u8; 1] = [rts_instruction];
        self.cs.set_low();
        self.write(&instruction_msg);
        self.cs.set_high();
        self.tx_pending(false);
    }

    /// Performs 'Load TX' Instruction
    fn load_tx_buffer(&mut self, buffer: TXBN, mut data_in: CanMessage) {
        //defmt::println!("------> Load TX buffer instruction:");
        
        let instruction = match buffer {
            TXBN::TXB0 => InstructionCommand::LoadTX as u8,
            TXBN::TXB1 => InstructionCommand::LoadTX as u8 | 0x2,
            TXBN::TXB2 => InstructionCommand::LoadTX as u8 | 0x4,
        };

        let mut reg_offset = 0x01 as u8;
        let mut address = buffer as u8;
        let mut data_bytes = data_in.to_bytes();

        /* Transaction START ------------------------------- */
        self.cs.set_low();
        self.spi.write(&[instruction]);

        for data in data_bytes.into_iter(){
            let next_address = address + reg_offset; 
            //defmt::println!("Load TX, address: {:08b}, data in: {:08b}", next_address, data);
            //let mut instruction_msg: [u8; 3] = [InstructionCommand::Write as u8, next_address, data];
            self.spi.write(&[data]); 
            reg_offset+=0x01;
        }
        self.cs.set_high();
        /* Transaction END ---------------------------------- */

        // TODO: - Add target buffer to the 'self.tx_pending' method.
        self.tx_pending(true);
    }

    /// Performs the 'Read Rx' Instruction.
    /// During the spi transfer, we send dont cares, since the address is auto incremented.
    fn read_rx_buffer(&mut self, buffer: RXBN) -> Result<[u8; 13], SPI::Error> {
        //defmt::println!("--------> Read RX buffer instruction:");
        
        const DONT_CARE: u8 = 0x00;
        let mut rx_data: [u8; 3] = [0; 3];
        let mut rx_buffer = [0u8; 13];

        let mut instruction = match buffer {
            RXBN::RXB0 => InstructionCommand::ReadRxb0 as u8,
            RXBN::RXB1 => InstructionCommand::ReadRxb1 as u8,
        };

        /* Start of the transaction. */
        self.cs.set_low();
        let mut instruction_buf = [instruction];
        self.spi.write(&[instruction]);
        self.spi.transfer(&mut rx_buffer, &[0; 13]).ok(); // Read the 14 bytes of CAN data
        self.cs.set_high();
        /* End of transation. */
        
        let mut frame = CanMessage::from(rx_buffer);
        //defmt::println!("Reading RX buffer (received): {:08b}", rx_buffer);

        Ok(rx_buffer)
    }

    /// Reset the MCP2515 and its registers to its default values.
    /// This is usually called, during startup, before configuration. 
    fn reset_instruction(&mut self) {
        defmt::println!("Calling Reset instruction over SPI");
        self.cs.set_low();
        self.spi.write(&[InstructionCommand::Reset as u8]);
        self.cs.set_high();
    }

    /// This would CLEAR the TXREQ bits indicating the tx buffer is not pending
    /// before writing. When 'is_pending' flag is false, it indicate that
    /// transmit buffer is not pending transmission. Hence should be called
    /// before writing to the transmit buffer. Setting the flag to true,
    /// would flag a message buffer as being ready for transmission.
    ///
    /// ## TODO 
    ///
    /// Add which TX buffer to set as ready or pending, by adding argument.
    /// This is important so we dont CLEAR or set something that is not ready.
    fn tx_pending(&mut self, is_pending: bool) -> &mut Self {
        //defmt::info!("Logic for tx_pending:");
        const CLEAR: u8 = 0u8;
        const FILTERMASKOFF: u8 = 0b0110_0000;
        let mut reg_bits = 0u8;
        reg_bits |= 1 << 3;
        let bit_mask: u8 = 0b0000_1000;
        //reg_bits |= 0b0000_0011;

        if (is_pending == false) {
            self.write_register(MCP2515Register::TXB0CTRL, CLEAR);
            //self.write_register(MCP2515Register::TXB1CTRL, CLEAR);
            //self.write_register(MCP2515Register::TXB2CTRL, CLEAR);
            self.write_register(MCP2515Register::RXB0CTRL, CLEAR);
            self.write_register(MCP2515Register::RXB1CTRL, CLEAR);
            
            //self.write_register(MCP2515Register::RXB0CTRL, FILTERMASKOFF);            
            //self.write_register(MCP2515Register::RXB1CTRL, FILTERMASKOFF);
        }else {
            //defmt::println!("is_pending transmission = true\nWriting {:08b} to TXB0CTRL", reg_bits);
            self.write_register(MCP2515Register::TXB0CTRL, bit_mask);
            //self.write_register(MCP2515Register::TXB1CTRL, reg_bits);
            //self.write_register(MCP2515Register::TXB2CTRL, reg_bits);
        }
        self
    }

    /// This will set the RXBNCTRL.RXM bits, for setting the RX Operating Mode. 
    ///
    /// ## NOTE 
    ///
    /// RXM: Receive Buffer Operating Mode bits
    ///         ################################################
    ///         ## 11 = Turn mask/filters off; receive any message.
    ///         ## 10 = Receive only valid messages with extended identifiers that meet filter criteria.
    ///         ## 01 = Receive only valid messages with standard identifiers that meet filter criteria.
    ///         ## 00 = Receive all valid messages using either standard or extended identifiers that meet filter criteria.
    ///         ################################################
    fn set_rxm_mode(&mut self, rxm: ReceiveBufferMode) -> &mut Self{
        match (rxm){
            ReceiveBufferMode::ReceiveAny => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0000_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0000_0000);
            },
            ReceiveBufferMode::FilterOffReceiveAny => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0110_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0110_0000);
            },
            ReceiveBufferMode::OnlyExtendedId => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0100_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0100_0000);
            },
            ReceiveBufferMode::OnlyStandardId => {
                self.write_register(MCP2515Register::RXB0CTRL, 0b0010_0000);
                self.write_register(MCP2515Register::RXB1CTRL, 0b0010_0000);
            },
        }
        self
    }

    /// This would apply & configure the Can controller settings for the module.
    fn setup_configuration(&mut self, can_settings: &CanSettings) -> &mut Self {
        defmt::info!("Setting up Configuration: ");

        self.reset_instruction();

        let canstat_reg = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let canctrl_byte = self.get_canctrl_mask(can_settings.canctrl);
        
        /* Write configuration bits to the register that needs to be setup. */
        self.write_register(MCP2515Register::CANCTRL, canctrl_byte);
        self.write_register(MCP2515Register::TXRTSCTRL, 0b0000_0011);
        self.write_register(MCP2515Register::BFPCTRL, 0b0000_0011);
     
        /* Sanity check, reading so the configuration has written correctly */
        let canctrl_bits = self.read_register(MCP2515Register::CANCTRL, 0x00).unwrap(); 
        let txrtscttrl_reg = self.read_register(MCP2515Register::TXRTSCTRL, 0x00).unwrap();
        let bfpctrl_reg = self.read_register(MCP2515Register::BFPCTRL, 0x00).unwrap();

        /* The requested mode must be varified by reading the OPMOD[2:0] bits (CANSTAT[7:5])*/
        let canstat_new = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        defmt::println!("CANSTAT after reset instruction is: {:08b}", canstat_reg);  
        defmt::println!("CANCTRL mask prior writing: {:08b}", canctrl_byte);  
        defmt::println!("CANCTRL after setup is: {:08b}", canctrl_bits);  
        defmt::println!("CANSTAT after setup is: {:08b}", canstat_new);  
        defmt::println!("TXRTSCTRL after setup is: {:08b}", txrtscttrl_reg);  
        defmt::println!("BFPCTRL after setup is: {:08b}", bfpctrl_reg);  

        self
    }

    ///This change the mode and set it to Normal mode for sending and receiving on the CAN bus.
    pub fn activate_canbus(&mut self){
        const ONE_SHOT_MODE: bool = true;
        if (self.can_settings.canctrl.mode != OperationTypes::Normal){
            //let mut can_settings = CanSettings::default();
            let canctrl_settings = CanControllSettings::new(OperationTypes::Normal, true, CLKPRE::DIV1, false, !ONE_SHOT_MODE);
            self.can_settings.canctrl = canctrl_settings;
            defmt::info!("Setting Operation Type to: Normal mode");
            self.change_settings(self.can_settings);
        }
    }

    /// Transmit frame on CAN bus, by loading TXBN buffer, and write 'Request-To-Send' instruction.
    ///
    /// ## TODO
    ///
    /// Add logic for targeting TX buffers to read, that is not busy...
    fn transmit_can(&mut self, can_msg: &CanMessage){
        let can_frame = CanMessage::new(can_msg.id, &can_msg.data).unwrap();
        self.load_tx_buffer(TXBN::TXB0, can_frame); // How should I handle buffer target for transmission
        self.request_to_send(TXBN::TXB0);
    }

    /// Receive CAN frame by reading the appropriate RXBN buffer.
    ///
    /// ## TODO
    ///
    /// Write logic for picking appropriate RXBN buffer to read.
    /// Based on CANINTF, RXNCTRL.FILT bits, or through RX status instruction. 
    fn receive_can(&mut self) -> CanMessage{
        let rx_buffer = self.read_rx_buffer(RXBN::RXB0).unwrap();
        let mut frame = CanMessage::from(rx_buffer);
        defmt::info!("Can bus received the Frame:");
        frame.print_frame();
        let data_str: &str = core::str::from_utf8(frame.data()).unwrap(); 
        defmt::info!("Decoded frame data: {:?}", data_str);
        frame
    }

    /// Setting up the MCP speed and bitrate of the MCP2515 module. 
    fn setup_bitrate(&mut self) -> &mut Self {
        //TODO: - Fix parsing of can_settings to set other MCP speed and can bitrates

        /* Reference: https://github.com/autowp/arduino-mcp2515/blob/master/mcp2515.h
         * #define MCP_16MHz_125kBPS_CFG1 (0x03)
         * #define MCP_16MHz_125kBPS_CFG2 (0xF0)
         * #define MCP_16MHz_125kBPS_CFG3 (0x86)
         */
        defmt::info!("Setting up bitrate...");

        // This would set the MCP speed of 8MHz and a bitrate of 125kBPS
        self.write_register(MCP2515Register::CNF1, 0x01);
        self.write_register(MCP2515Register::CNF2, 0xB1);
        //self.write_register(MCP2515Register::CNF2, 0xF1); //transmit three times?
        self.write_register(MCP2515Register::CNF3, 0x85);

        self
    }

    /// Logic for transferring messages to the RXBn buffers only if acceptance
    /// filter criteria is met. Where the messages is handled in the Message
    /// Assembly Buffer (MAB).
    fn message_acceptance() {}

    /// Sets the acceptance filters in the peripheral.
    ///
    /// ## Note
    ///
    /// For this version of the crate we only support one filter per buffer.
    /// Future releases might support more but as this is developed as a part of
    /// a project this will be left was future work.
    ///
    /// ## Panics
    ///
    /// This function panics if the mask of filter exceed 11 bits.
    /// It also panics if the mask will ignore parts of the filter.
    ///
    /// None of these panics will happen in release.
    pub fn filter_message_id(&mut self, channel: RXBN, mask: u16, filter: u16) -> &mut Self {
        // Debug assert here. This will be run during init.
        // If
        debug_assert!(
            mask & filter == filter,
            "Mask will be atleast partially cancled by the filter"
        );
        debug_assert!((filter>>5) < 1 << 11, "Filter must be an 11 bit integer");
        debug_assert!((mask >> 5) < 1 << 11, "Mask must be an 11 bit integer");
        let ((mask_reg_high, mask_reg_low), (filter_reg_high, filter_reg_low)) = match channel {
            // RXB0 have the associated; RXM0, RXF0, RXF1.      
            RXBN::RXB0 => (
                (MCP2515Register::RXM0SIDH, MCP2515Register::RXM0SIDL),
                (MCP2515Register::RXF0SIDH, MCP2515Register::RXF0SIDL),
            ),
            // RXB1 have the associated; RXM1, RXF2, RXF3, RXF4, RXF5.
            RXBN::RXB1 => (
                (MCP2515Register::RXM1SIDH, MCP2515Register::RXM1SIDL),
                (MCP2515Register::RXF2SIDH, MCP2515Register::RXF2SIDL),
            ),
        };

        let rxmn_sidh = ((mask >> 8) & 0b1111_1111) as u8;
        let rxmn_sidl = (mask & 0b1110_0000) as u8;
        let rxfn_sidh = ((filter >> 8) & 0b1111_1111) as u8; 
        let rxfn_sidl = (filter & 0b1110_0000) as u8;
    
        defmt::info!("Mask and Filter:\nRXM_SIDH = {:08b}\nRXM_SIDL = {:08b}\nRXFN_SIDH = {:08b}\nRXFN_SIDL = {:08b}", 
            rxmn_sidh, rxmn_sidl, rxfn_sidh, rxfn_sidl);
      
        self.write_register(mask_reg_high, rxmn_sidh);
        self.write_register(mask_reg_low, rxmn_sidl);
        self.write_register(filter_reg_high, rxfn_sidh);
        self.write_register(filter_reg_low, rxfn_sidl);

        self
   }

    /// Parsing the provided CanSettings struct, and enable interrupts. 
    ///
    /// ## Note
    ///
    /// "When a message is moved into either of the receive
    ///    buffers, the appropriate CANINTF.RXnIF bit is set. This
    ///    bit must be cleared by the MCU in order to allow a new
    ///    message to be received into the buffer. This bit
    ///    provides a positive lockout to ensure that the MCU has
    ///    finished with the message before the MCP2515
    ///    attempts to load a new message into the receive buffer.
    ///    If the CANINTE.RXnIE bit is set, an interrupt will be
    ///    generated on the INT pin to indicate that a valid
    ///    message has been received."
    fn setup_interrupt(&mut self, can_settings: &CanSettings) -> &mut Self {
        let settings = can_settings.enable_interrupts(&[
            CanInte::MERRE,
            CanInte::TX2IE,
            CanInte::TX1IE,
            CanInte::TX0IE,
            CanInte::RX1IE,
            CanInte::RX0IE,
        ]);
        
        self.can_settings.interrupts = settings.interrupts;
        self.write_register(MCP2515Register::CANINTE, settings.interrupts);
        self.read_register(MCP2515Register::CANINTE, 0x00);

        self
    }

    /// This method, would clear the CANINTF at bit_pos param. 
    /// It send the Bit Modify instruction with mask and data bytes. 
    fn clear_interrupt_flag(&mut self, bit_pos: u8){
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        let mut mask_bytes: u8 = self.can_settings.interrupts; // enabled target interrupt bits.
        let mut data_bytes = canintf ^ (1 << bit_pos); //0bxxxx_1011 for clearing TX0IF

        self.cs.set_low();
        self.spi.write(&[
            InstructionCommand::Modify as u8,
            MCP2515Register::CANINTF as u8,
            mask_bytes,
            data_bytes,
        ]);
        self.cs.set_high();

        let canintf_after = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();

        /*
        defmt::info!("Bit Modify Instruction:\n
            Mask bytes: {:08b}\n
            Data byte: {:08b}\n
            CANINTF previous content: {:08b}\n
            CANINTF after Bit Modify: {:08b}", mask_bytes, data_bytes, canintf, canintf_after);
        */
    }

    /// Decodes the CANSTAT.ICOD bits, and map to matched InterruptFlagCode. 
    pub fn interrupt_decode(&mut self) -> Result<InterruptFlagCode, CanError> {
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let mut interrupt_code = (canstat & 0b0000_1110) >> 1; // clear OPMOD bits and shift right by 1. 
        //defmt::info!("Interrupt decode logic (ICOD), clear OPMOD and shift by 1 mask: {:08b}", interrupt_code);
        
        match InterruptFlagCode::try_from(interrupt_code) {
            Ok(flag_code) => {
                defmt::info!("Received the Interrupt type: {:?}", flag_code);
                Ok(flag_code)
            },
            Err(e) => {
                defmt::error!("Failed to decode InterruptFlagCode: {:?}", e);
                Err(CanError::MessageErrorInterrupt)
            },
        }
    }

    /// This just prints out, register bits related to a received error. 
    ///
    /// ## NOTE
    ///
    /// EFLG register - bits related to a specific ERROR FLAG. 
    /// TEC - Transmit Error Count.
    /// REC - Receive Error Count. 
    fn error_handling(&mut self){
        
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let errorflag_reg = self.read_register(MCP2515Register::EFLG, 0x00).unwrap();
        let txbnctrl = self.read_register(MCP2515Register::TXB0CTRL, 0x00).unwrap();
        let tec = self.read_register(MCP2515Register::TEC, 0x00).unwrap();
        let rec = self.read_register(MCP2515Register::REC, 0x00).unwrap();

        let mut eflags_output = [EFLG::UNKNOWN; 8];
        let eflgs_count = self.eflg_decode(errorflag_reg, &mut eflags_output); 
        let eflags_to_handle = &eflags_output[0..eflgs_count];

        let mut tx_req: bool = false;
        let mut tx_err: bool = false;
        let mut message_lost: bool = false; 

        if (txbnctrl & (1 << 4) != 0) {
            tx_err = true; 
        }
        if (txbnctrl & (1 << 3) != 0) {
            tx_req = true; 
        }
        if (txbnctrl & (1 << 5) != 0) {
            message_lost = true; 
        }

        defmt::info!("Error Decode Info: \nTXREQ = {:?}, TXERR = {:?}, MLOA = {:?}", tx_req, tx_err, message_lost);
        defmt::info!("TEC status: {:08b}", tec);
        defmt::info!("REC status: {:08b}", tec);
        defmt::info!("CANINTF register bits: {:08b}", canintf);
        defmt::info!("CANSTAT register bits: {:08b}", canstat);
        defmt::info!("EFLG register bits: {:08b}", errorflag_reg);
        defmt::info!("EFLG: {:?} to handle!", eflags_to_handle);
    }

    /// Decodes the EFLG register...
    ///
    /// ## NOTE 
    ///
    /// Still under progress!
    fn eflg_decode(&mut self, eflg_bits: u8, eflags: &mut [EFLG]) -> usize{
        let mut count = 0; 
        
        for bit_pos in 0..8 {
            if (eflg_bits & (1 << bit_pos) != 0){
                defmt::info!("Found EFLG: {:?}", EFLG::from_u8(bit_pos as u8));
                eflags[bit_pos] = EFLG::from_u8(bit_pos as u8);
                count += 1;
            }
        }
        defmt::info!("eflg_decode count: {:?}", count);
        count as usize
    }

    /// This would parse the received interrupt, triggered by the GPIOTE channel.
    /// Pass the decoded interrupt from 'interrupt_decode()' and handle it.
    /// E.g., clear neccessary regs such as clearing the appropriate CANINTF bits.
    ///
    /// ## NOTE
    ///
    /// This is still under progress... 
    ///
    /// ## TODO
    ///
    /// - Fix how different TX and RX buffers should be handled. 
    /// - Write handle logic for: ErrorInterrupt, WakeUpInterrupt, TXB1, TXB2. 
    pub fn handle_interrupt(&mut self, received_interrupt: InterruptFlagCode) {
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();

        match received_interrupt {
            InterruptFlagCode::NoInterrupt =>{
                if (canintf & (1 << 7) != 0) {
                    defmt::info!("Message Error Interrupt occured (MERRE)!");
                    defmt::println!("CANINTF register bits: {:08b}", canintf);
                    self.clear_interrupt_flag(7);
                    let all_handled = self.interrupt_is_cleared();
                    if (all_handled == false){
                        self.error_handling();
                        self.clear_interrupt_flag(7);
                        //return Err(CanError::MessageErrorInterrupt);
                    }
                }
                //self.clear_interrupt_flag(7);
            },
            InterruptFlagCode::ErrorInterrupt =>{
                defmt::info!("Received Error Interrupt, with flags...");
                self.clear_interrupt_flag(5);
            },
            InterruptFlagCode::WakeUpInterrupt =>{},
            InterruptFlagCode::TXB0Interrupt => {
                defmt::println!("TXB0 successfully transmitted a message!");
                //TODO: Clear the associated CANINTF bit for TXnIE below
                self.clear_interrupt_flag(2);
                self.tx_pending(false);
            },
            InterruptFlagCode::TXB1Interrupt => {},
            InterruptFlagCode::TXB2Interrupt => {},
            InterruptFlagCode::RXB0Interrupt => {
                defmt::println!("RXB0 received a message!");
                //TODO: FIX a try_from logic to map CANSTAT register as u8 to RXB0 /RXB1. 
                
                let rxb0_ctrl = self.read_register(MCP2515Register::RXB0CTRL, 0x00).unwrap();
                //defmt::info!("Filter bit hits of RXB0CTRL: {:08b}", rxb0_ctrl); 
                let mut frame = self.receive().unwrap();
 
                frame.data[0] = 0x4;
                frame.data[1] = 0x4;
                let dummy_data: &[u8];
                
                //let dummy_data: &[u8] = "Master".as_bytes();
 
                let rxb0_accept_id_node = StandardId::new(0x2).unwrap();
                let rxb0_accept_id = StandardId::new(0x1).unwrap();
                
                let id_rxb1 = Id::Standard(rxb0_accept_id_node);
                let id_rxb0 = Id::Standard(rxb0_accept_id);
                
                //frame.id = id_rxb1; 
                
                if (frame.id == id_rxb1){
                    dummy_data = "From:N".as_bytes();
                    frame.id = id_rxb0; 
                }else{
                    dummy_data = "From:M".as_bytes();
                    frame.id = id_rxb1; 
                } 


                let mut frame_response = CanMessage::new(frame.id, dummy_data).unwrap();
                

                //let received_byte_frame = self.read_rx_buffer(RXBN::RXB0).unwrap();
                //defmt::info!("Received byte frame: {:08b}", received_byte_frame);
                self.clear_interrupt_flag(0);
                self.transmit(&frame_response);
                //self.transmit(&frame);
            },
            InterruptFlagCode::RXB1Interrupt => {
                defmt::println!("RXB1 received a message!");
                //let received_byte_frame = self.read_rx_buffer(RXBN::RXB1).unwrap();
                //defmt::info!("Received byte frame: {:08b}", received_byte_frame); 
                let rxb1_ctrl = self.read_register(MCP2515Register::RXB1CTRL, 0x00).unwrap();
                //defmt::info!("Filter bit hits of RXB1CTRL: {:08b}", rxb1_ctrl);
                self.clear_interrupt_flag(1);
                
                //let mut frame = self.receive().unwrap();
             
                let rx_buffer = self.read_rx_buffer(RXBN::RXB1).unwrap();
                let mut frame = CanMessage::from(rx_buffer);
                //defmt::info!("Can bus received the Frame:");
                frame.print_frame();
                let data_str: &str = core::str::from_utf8(frame.data()).unwrap(); 
                defmt::info!("Decoded frame data: {:?}", data_str);

                // This modify the received frame, and change ID that is OK against the RXB1
                // acceptans filter...
                let rxb0_accept_id = StandardId::new(0x1).unwrap();
                let id_rxb0 = Id::Standard(rxb0_accept_id);
                frame.id = id_rxb0;
                let dummy_data = "Node".as_bytes();
                let mut frame_response = CanMessage::new(id_rxb0, dummy_data).unwrap();

                self.transmit(&frame_response);
            },
        }
    }

    /// This would check if all interrupt (interrupt flags) has been handled. 
    /// Will return true, if all has been handled, else false. 
    pub fn interrupt_is_cleared(&mut self) -> bool {
        const ZERO: u8 = 0u8;
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        if (canintf == ZERO) {
            true
        } else {
            false
        }
    }
}

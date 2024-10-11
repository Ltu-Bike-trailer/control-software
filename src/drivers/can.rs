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

//TODO: - Add CanModule struct type field to the CanDriver.

///
pub struct CanDriver<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> {
    pub spi: SPI,
    pub cs: PIN,
    pub interrupt_pin: PININT,
    pub can_settings: CanSettings,
}

/*
pub trait CanInterruptHandler{
    fn configure_gpiote(&mut self, gpiote: Gpiote);

    /// Setup/configure and enable the interrupts in the MCP2515 register CANINTE.
   fn setup_interrupt(&mut self, can_settings: CanSettings);

    /// Decodes and reads the interrupt type. Should also check the ICOD for interrupt priority.
    fn interrupt_decode(&mut self);

    /// Parse the pending interrupt, and clear the associated interrupt flag (CANINTF), when done.
    fn handle_interrupt(&mut self);
}
*/

/* Instruction commands specific to the MCP2515 via SPI */
#[repr(u8)]
enum InstructionCommand {
    Reset = 0xC0,
    Read = 0x03,
    Write = 0x02,
    Status = 0xA0, //1010_0000
    Modify = 0x05,
    LoadTX = 0x40,   //FIX to target B0, B1, or B2 buffers
    RTS = 0x80,      // 10000nnn
    ReadRxb0 = 0x90, // 0b10010000
    ReadRxb1 = 0x94, // 0b10010100
    RxStatus = 0xB0,
}

/// The MCP2515 modes of operation.
///REQOP: Request Operation Mode bits <2:0>
//    000 = Set Normal Operation mode
//    001 = Set Sleep mode
//    010 = Set Loopback mode
//    011 = Set Listen-only mode
//    100 = Set Configuration mode
#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum OperationTypes {
    Configuration = 0b100,
    Normal = 0b000,
    Sleep = 0b001,
    ListenOnly = 0b011,
    Loopback = 0b010,
}

/*
 * TXB0 - TXB3 = transmit buffers for MCP2515 module.
 * RXB0 and RXB1 = receive buffers for the MCP2515 module.
 */

#[repr(u8)]
enum TXBN {
    TXB0 = 0x30, //
    TXB1 = 0x40, //
    TXB2 = 0x50, //
}

#[repr(u8)]
/// Denotes the valid RX buffers.
pub enum RXBN {
    RXB0 = 0x60, //
    RXB1 = 0x70, //
}

pub enum ReceiveBufferMode{
    FilterOffReceiveAny,
    OnlyExtendedId,
    OnlyStandardId,
    ReceiveAny,
}

enum RXMN {
    RXM0, //Acceptans mask 0
    RXM1, // Acceptans mask 1
}

#[repr(u8)]
enum RXFN {
    RXF0 = 0b0001_0000, // Acceptance Filter N, n = 0, 1..., 5
    RXF1,
    RXF2,
    RXF3,
    RXF4,
    RXF5,
}

#[derive(Clone, Copy)]
#[repr(u8)]
pub enum CLKPRE {
    DIV1 = 0b000, // System Clock/1
    DIV2 = 0b001, // System Clock/2
    DIV4 = 0b010, // System Clock/4
    DIV8 = 0b011, // System Clock/8
}

#[repr(u8)]
enum MCP2515Register {
    CANSTAT = 0x0E, // 00001110
    CANCTRL = 0x0F, // 00001111
    CANINTE = 0x2B,
    CANINTF = 0x2C, // 00101100

    BFPCTRL = 0x0C,
    TXRTSCTRL = 0x0D, //00001101
    TEC = 0x1C,
    REC = 0x1D,
    CNF1 = 0x2A,
    CNF2 = 0x29,
    CNF3 = 0x28,
    EFLG = 0x2D,

    TXB0CTRL = 0x30, // 0011_0000 -> 0011_xxxx
    TXB1CTRL = 0x40, // 0100_0000 -> 0100_xxxx
    TXB2CTRL = 0x50, // 0101_0000 -> 0101_xxxx

    RXB0CTRL = 0x06,
    RXB1CTRL = 0x07,

    RXM0SIDH = 0x20,
    RXM0SIDL = 0x21,

    RXM1SIDH = 0x24,
    RXM1SIDL = 0x25,

    RXF0SIDH = 0x00,
    RXF0SIDL = 0x01,

    RXF1SIDH = 0x04,
    RXF1SIDL = 0x05,

    RXF2SIDH = 0x08,
    RXF2SIDL = 0x09,

    RXF3SIDH = 0x10,
    RXF3SIDL = 0x11,

    RXF4SIDH = 0x14,
    RXF4SIDL = 0x15,

    RXF5SIDH = 0x18,
    RXF5SIDL = 0x19,
}

#[derive(Debug)]
pub enum CanDriverError {
    SpiError,
    FrameError,
}

#[repr(u8)]
pub enum TxRxbn {
    // BELOW is the lower address bits for the corresponding TX/BX buffer N
    CTRL = 0x0,
    SIDH = 0x1,
    SIDL = 0x2,
    EID8 = 0x3,
    EID0 = 0x4,
    DLC = 0x5,
    D0 = 0x6,
    D1 = 0x7,
    D2 = 0x8,
    D3 = 0x9,
    D4 = 0x10,
    D5 = 0x11,
    D6 = 0x12,
    D7 = 0x13,
}

#[derive(Clone, Copy)]
pub struct CanControllSettings {
    mode: OperationTypes,
    clken: bool,
    clkpre: CLKPRE,
    abat: bool, // ABAT: Abort All Pending Transmission bit
    osm: bool,  // OSM: One-Shot Mode (1 = Enabled, 0 = disabled)
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CanInte {
    MERRE = 0b1000_0000,
    WAKIE = 0b0100_0000,
    ERRIE = 0b0010_0000,
    TX2IE = 0b0001_0000,
    TX1IE = 0b0000_1000,
    TX0IE = 0b0000_0100,
    RX1IE = 0b0000_0010,
    RX0IE = 0b0000_0001,
}

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

/// In Mega-hertz
#[derive(Clone, Copy)]
pub enum McpClock {
    MCP16,
    MCP8,
}

/// In kBPS
#[derive(Clone, Copy)]
pub enum CanBitrate {
    CAN125,
}

#[derive(Clone, Copy)]
pub struct CanSettings {
    pub canctrl: CanControllSettings,
    pub mcp_clk: McpClock, // Clock Frequency for MCP2515.
    pub can_bitrate: CanBitrate,
    pub interrupts: u8, 
}

impl CanControllSettings {
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
        Self {
            canctrl: canctrl_settings,
            mcp_clk: McpClock::MCP8,
            can_bitrate: CanBitrate::CAN125,
            interrupts: 0u8,
        }
    }
}

impl CanSettings {
    fn enable_interrupt(mut self, interrupt_type: CanInte) -> Self {
        self.interrupts |= interrupt_type as u8;
        self
    }

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
    RX1OVR = 7,
    RX0OVR = 6,
    TXBO = 5,
    TXEP = 4,
    RXEP = 3,
    TXWAR = 2,
    RXWAR = 1,
    EWARN = 0,
    UNKNOWN = 0xFF,
}

#[derive(Debug)]
pub enum CanError{
    TransmissionError,
    RX0Overflow,
    RX1Overflow,
    MessageErrorInterrupt,
    ReceiveError,
    DecodeError,
}


impl EFLG{
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


/// required trait methods goes here...
impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> Can
    for CanDriver<SPI, PIN, PININT>
{
    type Error = CanError;
    type Frame = CanMessage;
    
    fn transmit(&mut self, frame: &Self::Frame) -> Result<(), Self::Error> {
        //defmt::println!("Executing 'self.transmit_can(frame)");
        //self.read_status();
        self.transmit_can(frame); // This is the custom driver logic for transmitting on the CAN bus. 
        Ok(())
    }

    fn receive(&mut self) -> Result<Self::Frame, Self::Error> {
        let frame = self.receive_can(); 
        //let dummy_data = [0u8; 8];
        //Ok(CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), &dummy_data).unwrap()) 
        Ok(frame)
    }
}

impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> CanDriver<SPI, PIN, PININT> {
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
            .set_rxm_mode(ReceiveBufferMode::OnlyStandardId);

        let dummy_id = StandardId::new(0x1).unwrap();
        let mut frame = CanMessage::new(embedded_can::Id::Standard(dummy_id), &[0x01, 0x02, 0x03]).unwrap();
       
        let id_for_rxb1 = (StandardId::new(0x2).unwrap().as_raw() << 5);
        defmt::info!("StandardId as raw: {:016b}", id_for_rxb1);

        let filter_id_rxb0 = frame.id_raw();
        let rx_mask = 0b1111_1111_1110_0000 as u16;
        defmt::info!("filter id = {:016b} vs mask = {:016b}", filter_id_rxb0, rx_mask);
        driver.filter_message_id(RXBN::RXB0, rx_mask, filter_id_rxb0);
        driver.filter_message_id(RXBN::RXB1, rx_mask, id_for_rxb1);

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

    fn read(&mut self, data: &mut [u8]) {
        self.spi.read(data);
    }

    fn write(&mut self, data: &[u8]) {
        self.spi.write(data);
    }

    fn flush(&mut self) {
        self.cs.set_low();
        self.spi.flush();
        self.cs.set_high();
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]) {
        self.cs.set_low();
        self.spi.transfer(read, write);
        self.cs.set_high();
    }

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

    fn change_settings(&mut self, settings: CanSettings) {
        self.can_settings.canctrl = settings.canctrl;

        let bitmask_canctrl = self.get_canctrl_mask(self.can_settings.canctrl);
        self.write_register(MCP2515Register::CANCTRL, bitmask_canctrl);
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        
        //defmt::info!("Bitmask for changing settings: {:08b}", bitmask_canctrl);         
        //defmt::info!("After change settings in CANSTAT: {:08b}", canstat);
    }

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
    ///
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
        defmt::println!("------> Load TX buffer instruction:");
        
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
        defmt::println!("--------> Read RX buffer instruction:");
        
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
    pub fn filter_message_id(&mut self, channel: RXBN, mask: u16, filter: u16) {
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
                defmt::info!("Filter bit hits of RXB0CTRL: {:08b}", rxb0_ctrl); 
                let mut frame = self.receive().unwrap();
                frame.data[0] = 0x4;
                frame.data[1] = 0x4;
                
                let rxb1_accept_id = StandardId::new(0x2).unwrap();
                let id_rxb1 = Id::Standard(rxb1_accept_id);
                frame.id = id_rxb1; 
                
                //let received_byte_frame = self.read_rx_buffer(RXBN::RXB0).unwrap();
                //defmt::info!("Received byte frame: {:08b}", received_byte_frame);
                self.clear_interrupt_flag(0);
                self.transmit(&frame);
                //self.transmit(&frame);
            },
            InterruptFlagCode::RXB1Interrupt => {
                defmt::println!("RXB1 received a message!");
                //let received_byte_frame = self.read_rx_buffer(RXBN::RXB1).unwrap();
                //defmt::info!("Received byte frame: {:08b}", received_byte_frame); 
                let rxb1_ctrl = self.read_register(MCP2515Register::RXB1CTRL, 0x00).unwrap();
                defmt::info!("Filter bit hits of RXB1CTRL: {:08b}", rxb1_ctrl);
                self.clear_interrupt_flag(1);
                
                //let mut frame = self.receive().unwrap();
             
                let rx_buffer = self.read_rx_buffer(RXBN::RXB1).unwrap();
                let mut frame = CanMessage::from(rx_buffer);
                defmt::info!("Can bus received the Frame:");
                frame.print_frame();

                // This modify the received frame, and change ID that is OK against the RXB1
                // acceptans filter...
                let rxb0_accept_id = StandardId::new(0x1).unwrap();
                let id_rxb0 = Id::Standard(rxb0_accept_id);
                frame.id = id_rxb0;

                self.transmit(&frame);
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

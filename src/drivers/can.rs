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
    Reset = 0b1100_0000,
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
enum RXBN {
    RXB0 = 0x60, // 
    RXB1 = 0x70, //
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
    DIV1 = 0b000, // System Clock/1
    DIV2 = 0b001, // System Clock/2
    DIV4 = 0b010, // System Clock/4
    DIV8 = 0b011, // System Clock/8
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

    TXB0CTRL = 0x30, // 0011_0000 -> 0011_xxxx
    TXB1CTRL = 0x40, // 0100_0000 -> 0100_xxxx
    TXB2CTRL = 0x50, // 0101_0000 -> 0101_xxxx

    RXB0CTRL = 0x60,
    RXB1CTRL = 0x70,
}


#[derive(Debug)]
pub enum CanDriverError{
    SpiError,
    FrameError,
}

#[repr(u8)]
pub enum TxRxbn{
    // Full buffer length are: [u8; 14] for all the below fields and data: [u8; 8],
    
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


pub struct CanMessage{
    id: embedded_can::Id,
    dlc: u8, // Data length code = length of data field. 
    data: [u8; 8], // D7 - D0
}

pub struct CanControllSettings{
    mode: OperationTypes,
    clken: bool,
    clkpre: CLKPRE,
    abat: bool, // ABAT: Abort All Pending Transmission bit
    osm: bool, // OSM: One-Shot Mode (1 = Enabled, 0 = disabled)
}

#[repr(u8)]
pub enum CanInte{
    MERRE,
    WAKIE,
    ERRIE,
    TX2IE,
    TX1IE, 
    TX0IE,
    RX1IE,
    RX0IE,
}

pub struct CanSettings{
    pub canctrl: CanControllSettings,
    //pub caninte: CanInte, 
    pub mcp_clk: u8, // Clock Frequency for MCP2515. 
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
        Some(CanMessage::new(id.into(), data).unwrap())
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
        //embedded_can::Id::Standard((StandardId::ZERO));
        self.id
    }

    fn dlc(&self) -> usize {
       self.dlc as usize 
    }

    fn data(&self) -> &[u8] {
        &self.data
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
        let dummy_data = [0u8; 8];
        Ok(CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), &dummy_data).unwrap()) 
    }

}


/// Here goes custom logic for the CanMessage struct type
impl CanMessage {
    pub fn new(id: embedded_can::Id, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let dlc = data.len() as u8;
        let mut can_msg = CanMessage{
            id,
            dlc,
            data: [0u8; 8],
        };

        can_msg.data[..data.len()].copy_from_slice(data);
        return Some(can_msg);
    }

    /*
    pub fn parse_frame(buffer: &[u8]) -> Self {
        //TODO: - Needs to parse the bit ranges and map each part to its correct message type 
        CanMessage{}
    }
    */
    
}

/* This From trait would parse a type T into a CanMessage frame. */

//TODO: - UNDER PROGRESS...
impl From<[u8;14]> for CanMessage{
    fn from(byte_data: [u8;14]) -> Self {
        let dummy_data = [0u8; 8];
        let msg = CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), &dummy_data).unwrap();
        return msg;
    }
}


impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin> CanDriver<SPI, PIN>{

    pub fn init(spi: SPI, cs: PIN, can_settings: CanSettings) -> Self{
        let mut driver = Self{spi, cs};
        driver.setup_configuration(can_settings);
        driver.tx_pending(false);
        
        driver.get_txn(TXBN::TXB0);
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

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]){
        self.cs.set_low();
        self.spi.transfer(read, write);
        self.cs.set_high();
    }

    fn transfer_inplace(&mut self, data: &mut [u8]){
        self.spi.transfer_in_place(data);
    }

    fn flush(&mut self){
        self.spi.flush();
    }
    
     
    fn read_register(&mut self, reg: MCP2515Register) -> Result<u8, SPI::Error>{
        const DONTCARES: u8 = 0u8;
        let mut read_buf: [u8; 3] = [0; 3];
        let mut read_msg: [u8; 3] = [InstructionCommand::Read as u8, reg as u8, DONTCARES];
        
        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &read_msg);
        self.cs.set_high();

        defmt::println!("SPI transfer (READ-WRITE), Read buffer: {:08b}", read_buf);
        Ok(read_buf[1])
    }

    fn write_register(&mut self, reg: MCP2515Register, data: u8) -> Result<(), SPI::Error>{
        let reg_address: u8 = reg as u8;
        let maskbit: u8; 
        // Expected format: [u8; N] --> [Instruction byte, Address byte, Data byte]
        let mut byte_msg: [u8; 3] = [InstructionCommand::Write as u8, reg_address, data];
        //defmt::println!("write_register, byte msg: {:08b}", byte_msg);

        self.cs.set_low();
        self.spi.write(&byte_msg);
        self.cs.set_high();
        Ok(())
    }

    fn get_rxn(&mut self, buf_num: RXBN) -> [u8; 14]{
        const BUF0_ADDR: u8 = RXBN::RXB0 as u8; // 
        const BUF1_ADDR: u8 = RXBN::RXB1 as u8;
        let mut rxn: [u8; 14] = [0; 14];
        return rxn;
    }

    fn get_txn(&mut self, buf_num: TXBN) -> [u8; 14]{
        let buf_addr: u8 = buf_num as u8; 
        let mut txn: [u8; 14] = [0; 14];        
        
        let mut txb_nodata: [u8; 6] = [
            buf_addr | TxRxbn::CTRL as u8,
            buf_addr | TxRxbn::SIDH as u8,
            buf_addr | TxRxbn::SIDL as u8,
            buf_addr | TxRxbn::EID8 as u8,
            buf_addr | TxRxbn::EID0 as u8,
            buf_addr | TxRxbn::DLC as u8,
        ];

        let mut txb_data: [u8; 8] = [
            buf_addr | TxRxbn::D0 as u8,
            buf_addr | TxRxbn::D1 as u8,
            buf_addr | TxRxbn::D2 as u8,
            buf_addr | TxRxbn::D3 as u8,
            buf_addr | TxRxbn::D4 as u8,
            buf_addr | TxRxbn::D5 as u8,
            buf_addr | TxRxbn::D6 as u8,
            buf_addr | TxRxbn::D7 as u8,
        ];

        txn[0..6].copy_from_slice(&txb_nodata);
        txn[6..14].copy_from_slice(&txb_data);
        defmt::info!("TXBN: {:08b}", txn);
        
        // N = N | (1<<k)
        // TXB0 address starts at: 0011_0000 = TXB0CTRL register 
        // E.g., TXB0SIDH = 0011_0001 -> 0011_0000 OR 0000_0001
        // E.g., TXB0D0 = 0011_0110 -> 0011_0000 OR 0000_0110 

        return txn;
    }

    pub fn loopback_test(&mut self){
        // loopback test - by writing to the transmit buffers and then read the receive buffers
        // 1. Initiate transmission by setting the TXREQ bit (TXBnCTRL[3]) for each buffer
        // 2. Write to the register via SPI WRITE command
        //let byte_msg: [u8; 3] = [InstructionCommand::Write as u8, MCP2515Register::TXB0CTRL as u8, data];
        let dummy_data = "dummy_data 123".as_bytes();
        let mut frame = CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), dummy_data);
       
        let mut instruction_bit = InstructionCommand::LoadTX as u8;
        let sidh: u8 = instruction_bit;
        let d0: u8 = instruction_bit | 0x01;

        //let mut read_buf: [u8; 3] = [0; 3];
        let loadtx_msg_id: [u8; 2] = [sidh, 0x00];
        let loadtx_msg_data: [u8; 2] = [d0, 0x4 as u8];

        self.cs.set_low();
        self.spi.write(&loadtx_msg_id);
        self.spi.write(&loadtx_msg_data);
        self.cs.set_high();
        self.tx_pending(true);

        let mut read_buffer_id: [u8; 11] = [0; 11];
        let mut read_buffer_data: [u8; 8] = [0; 8];

        self.tx_pending(false);
        self.cs.set_low();
        
        let mut instr_bit = InstructionCommand::ReadRx as u8;
        let rx_sidh: [u8;1] = [instr_bit];
        let rx_d0: [u8;1] = [instr_bit | 0x02];
        self.spi.transfer(&mut read_buffer_id, &rx_sidh);
        self.spi.transfer(&mut read_buffer_data, &rx_d0);
        
        self.cs.set_high();
        self.tx_pending(true);

        defmt::println!("RX buffer SIDH: {:08b}", read_buffer_id);
        defmt::println!("RX buffer DATA: {:08b}", read_buffer_data);
 
    }

    pub fn set_canctrl_register(&mut self, canctrl_settings: CanControllSettings) -> u8 {
        let mut canctrl_byte = 0u8; // (1): data_byte |= (reg_value << reg_pos)
        let mode_bits: u8 = (canctrl_settings.mode as u8) << 5; //REQOP[2:0] (bits 7-5)
        let prescale: u8 = canctrl_settings.clkpre as u8; //CLKPRE[1:0] (bits 1-0)
        let clk_enabled = canctrl_settings.clken;
         
        defmt::println!("1. canctrl_byte: 0b{:08b}", canctrl_byte);
        canctrl_byte |= mode_bits; //(2)
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

    fn enable_interrupt(&mut self, caninte: CanInte){
        // The TXnIE bit in the CANINTE register need to be toggled/set to enable interrupt.
        let mut bitmask = 0u8;
        //self.write_register(MCP2515Register::CANINTE, data)
        
    }

    fn reset_instruction(&mut self){
        self.cs.set_low();
        self.spi.write(&[InstructionCommand::Reset as u8]);
        self.cs.set_high();
    }

    fn tx_pending(&mut self, is_pending: bool){
        const CLEAR: u8 = 0u8;
        let mut reg_bits = 0u8;
        reg_bits |= 1 << 3;

        /* When 'is_pending' flag is false, it indicate that transmit buffer is not pending
         * transmission, and should be called before writing to the transmit buffer. Setting the
         * flag to true, would flag a message buffer as being ready for transmission. */
        if (is_pending == false){
            self.write_register(MCP2515Register::TXB0CTRL, CLEAR);
            self.write_register(MCP2515Register::TXB1CTRL, CLEAR);
            self.write_register(MCP2515Register::TXB2CTRL, CLEAR);
        }else {
            self.write_register(MCP2515Register::TXB0CTRL, reg_bits);
            self.write_register(MCP2515Register::TXB1CTRL, reg_bits);
            self.write_register(MCP2515Register::TXB2CTRL, reg_bits); 
        }
        
    }

    /// This would apply and configure the Can controller settings for the module.
    fn setup_configuration(&mut self, can_settings: CanSettings){
        //TODO: : 
        // 1. Need to initially be in CONFIGURATION MODE.
        // 2. After changing mode, the requested mode must be verified by reading the OPMOD[2:0]
        //    bits in CANTSTAT[7:5]. 

        defmt::info!("Setting up Configuration: ");

        self.reset_instruction();
        // Check if in sleep mode and if you need to wake it up! 
        let canstat_reg = self.read_register(MCP2515Register::CANSTAT).unwrap();
        defmt::println!("Initial mode is: {:08b}", canstat_reg);  

        let canctrl_byte = self.set_canctrl_register(can_settings.canctrl);
        defmt::println!("CANCTRL register settings value bitmask: 0b{:08b}", canctrl_byte);
        defmt::info!("Before writing to CANCTRL register: ");
        self.read_register(MCP2515Register::CANCTRL); 
 
        defmt::info!("AFTER writing to CANCTRL register");
        self.write_register(MCP2515Register::CANCTRL, canctrl_byte);
        
        /* The requested mode must be varified by reading the OPMOD[2:0] bits (CANSTAT[7:5])*/
        self.read_register(MCP2515Register::CANSTAT);
        self.read_register(MCP2515Register::CANCTRL); 
        
        //TODO: - Write other relevant register settings below...
        //self.initiate_transmission(instruction, address_byte);
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
        // read from RXBN buffer
    }

    /// The 'Bit Timing Logic' (BTL) - monitors the bus line input and handle the bus-related,
    /// bit timing according to the CAN protocol.
    fn setup_bit_timing(&mut self){
        
    }

    /// Logic for transferring messages to the RXBn buffers only if acceptance filter criteria is
    /// met. Where the messages is handled in the Message Assembly Buffer (MAB).
    fn message_acceptance(){

    }

    fn start_transmission(&mut self, instruction: InstructionCommand, address_byte: u8){
        // 1. Need to set the TXBnCTRL.TXREQ bit for each buffer
        // 2. Configure using the TXRTSCTRL register (need to be in 'configuration mode').
        // Format: [u8; N] --> [Instruction byte, Address byte, Data byte]
      
        /* Prior sending the message: 
         * "The TXREQ bit (TXBnCTRL[3]) must be clear (indicating the transmit buffer is not
         * pending transmission) before writing to the transmit buffer".
         */


        
    }


}

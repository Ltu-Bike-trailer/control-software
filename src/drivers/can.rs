#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]


use core::{str, usize};

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

/// 
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
    Status = 0xA0, //1010_0000
    Modify = 0x05,
    LoadTX = 0x40, //FIX to target B0, B1, or B2 buffers
    RTS = 0x87,
    ReadRxb0 = 0x90, // 0b10010000
    ReadRxb1 = 0x94, // 0b10010100
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
    CANSTAT = 0x0E, // 00001110 
    CANCTRL = 0x0F, // 00001111
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

/// In Mega-hertz
pub enum McpClock {
    MCP16,
    MCP8,
}

/// In kBPS
pub enum CanBitrate {
    CAN125, 
}

pub struct CanSettings{
    pub canctrl: CanControllSettings,
    pub mcp_clk: McpClock, // Clock Frequency for MCP2515. 
    pub can_bitrate: CanBitrate,
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
        CanMessage::new(id.into(), data)
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
        &self.data[0..self.dlc()]
    }
}


#[derive(Debug)]
pub enum CanError{}

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
        self.transmit_can(frame); // This is the custom driver logic for transmitting on the CAN bus. 
        Ok(None)
    }

    fn receive(&mut self) -> nb::Result<Self::Frame, Self::Error> {
        self.receive_can(); // This is the custom driver logic for transmitting on the CAN bus. 
        let dummy_data = [0u8; 8];
        Ok(CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), &dummy_data).unwrap()) 
    }
}


/// Here goes custom logic for the CanMessage struct type
impl CanMessage {
    pub fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        if data.len() > 8 {
            return None;
        }
        let _dlc = data.len() as u8;
        let mut can_msg = CanMessage{
            id: id.into(),
            dlc: _dlc,
            data: [0u8; 8],
        };

        can_msg.data[..data.len()].copy_from_slice(data);
        return Some(can_msg);
    }

    pub fn to_bytes(&mut self) -> [u8; 13] {
        let mut byte_frame: [u8; 13] = [0; 13];
        let mut id_bytes: [u8; 2];
        let mut raw_id: u16;
        
        if let embedded_can::Id::Standard(id) = self.id {
            raw_id = id.as_raw();
        } else {
            raw_id = 0;
        }

        // Split the u16 raw_id into two u8 values
        let sidh = (raw_id >> 3) as u8;  // Most significant byte
        let sidl = (raw_id as u8 & 0x07) << 5;  // Least significant byte, 0x07 = 00000_0111

        //TODO: - Fix the parsing logic for the extended id bits...
        let extended_id8: u8 = 0;  
        let extended_id0: u8 = 0;

        defmt::info!("to_bytes: \nSIDH: {:08b}\nSIDL: {:08b}\nEID8: {:08b}\nEID0: {:08b}", 
            sidh, sidl, extended_id8, extended_id0);

        let data_start = 5 as usize;
        let data_end = data_start + self.dlc(); 

        /*
        byte_frame = [sidh, sidl, extended_id8, extended_id0, self.dlc() as u8, 
            self.data[0], self.data[1], self.data[2], self.data[3], self.data[4], 
            self.data[5], self.data[6], self.data[7]];
        */
        byte_frame[0..data_start].copy_from_slice(&[sidh, sidl, extended_id8, extended_id0, self.dlc() as u8]);
        byte_frame[data_start..data_end].copy_from_slice(&self.data[0..self.dlc()]); 

        defmt::info!("In to_bytes logic, self.data() contains: {:08b}", self.data());
        defmt::info!("In to_bytes logic, self.data() vs self.data field: {:08b}", self.data);
       
        return byte_frame;
    }

    pub fn data_to_string(data: &[u8]) -> &str{
        let data_str = str::from_utf8(&data).unwrap();
        data_str
    }
    
}

/* This From trait would parse a type T into a CanMessage frame. */
impl From<[u8;13]> for CanMessage{
    fn from(byte_data: [u8;13]) -> Self {
        //let mut data = [0u8; 8];
        let mut id_bytes: [u8; 2]; 
        let mut raw_id: u16 = 0u16;
        
        let slice_sidh = byte_data[0] as u16;
        let slice_sidl = (byte_data[1] & 0xE0) as u16; // 
        let dlc = byte_data[4];
        
        let data_start = 5 as usize;
        let data_end = data_start + dlc as usize; 
        let mut _data = &byte_data[data_start..data_end];

        //data[0..].copy_from_slice(&byte_data[5..]);
        // Shift MSB u16 SIDH part and OR the LSB u16 SIDL part 
        raw_id = ((slice_sidh << 3) + (slice_sidl >> 5));
        
        defmt::println!("byte_data[0] = SIDH reg: {:08b}", byte_data[0]);
        defmt::println!("byte_data[1] = SIDL reg: {:08b}", byte_data[1]);
        defmt::println!("CanMessage::from, lower_bits: {:016b}", slice_sidl);
        defmt::println!("CanMessage::from, raw_id: {:016b}", raw_id);
        defmt::println!("data length (DLC): {:?}", _data.len());        
        defmt::println!("CanMessage::from, data slice: {:08b}, data as str: {}", _data, CanMessage::data_to_string(_data));

        //WARN: - Would return None, if raw is out of range of an 11 bit integer.   
        let frame_id = StandardId::new(raw_id).unwrap();
        let msg = CanMessage::new(embedded_can::Id::Standard(frame_id), &_data).unwrap();
        
        return msg;
    }
}


impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin> CanDriver<SPI, PIN>{

    pub fn init(spi: SPI, cs: PIN, can_settings: CanSettings) -> Self{
        let mut driver = Self{spi, cs};
        driver.setup_configuration(can_settings);
        
        /* This would CLEAR the TXREQ bits indicating the tx buffer is not pending before writing. */
        driver.tx_pending(false);
        let received_reg = driver.read_register(MCP2515Register::TXB0CTRL, 0x00).unwrap();
        defmt::println!("TXREQ register: {:08b}", received_reg);
        driver.transfer_test(MCP2515Register::CANCTRL);
        driver    
    }

    fn read(&mut self, data: &mut [u8]) {
        self.spi.read(data);
    }


    fn write(&mut self, data: &[u8]){
        /*example: self.spi.write(&[0b01000101,*byte]); */
        self.spi.write(data);
    }

    fn flush(&mut self){
        self.cs.set_low();
        self.spi.flush();
        self.cs.set_high();
    }

    pub fn transfer(&mut self, read: &mut [u8], write: &[u8]){
        self.cs.set_low();
        self.spi.transfer(read, write);
        self.cs.set_high();
    }

    fn read_register(&mut self, reg: MCP2515Register, reg_offset: u8) -> Result<u8, SPI::Error>{
        const DONTCARES: u8 = 0u8;
        let mut read_buf: [u8; 3] = [0; 3];
        let register: u8 = reg as u8 + reg_offset;
        let mut read_msg: [u8; 3] = [InstructionCommand::Read as u8, register as u8, DONTCARES];
        
        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &read_msg);
        self.cs.set_high();

        defmt::info!("Read instruction, Sent (MOSI): {:08b}, Received (MISO): {:08b}", read_msg, read_buf);
        Ok(read_buf[2])
    }

    /// Expected format: [u8; N] --> [Instruction byte, Address byte, Data byte].
    /// For writing to a specific register
    fn write_register(&mut self, reg: MCP2515Register, data: u8) -> Result<(), SPI::Error>{
        let reg_address: u8 = reg as u8;
        let maskbit: u8; 
        let mut byte_msg: [u8; 3] = [InstructionCommand::Write as u8, reg_address, data];

        self.cs.set_low();
        self.spi.write(&byte_msg);
        self.cs.set_high();
        defmt::info!("Write instruction, Sent (MOSI): {:08b}, MISO = High-Impedance", byte_msg);
        Ok(())
    }

    fn change_settings(&mut self, settings: CanControllSettings){
        let bitmask_canctrl = self.get_canctrl_mask(settings);
        self.write_register(MCP2515Register::CANCTRL, bitmask_canctrl);
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        
        defmt::info!("Bitmask for changing settings: {:08b}", bitmask_canctrl);         
        defmt::info!("After change settings in CANSTAT: {:08b}", canstat);
    }

    fn transfer_test(&mut self, register: MCP2515Register){
        let mut read_buf: [u8; 3] = [0; 3];
        let write_buf: [u8; 3] = [InstructionCommand::Read as u8, register as u8, 0x00];
        self.cs.set_low();
        self.spi.transfer(&mut read_buf, &write_buf);
        self.cs.set_high();
        defmt::info!("Sent: {:08b}, Received: {:08b}", write_buf, read_buf);
    }

    /// Loopback test, that sets the mode to loopback, and LOAD TX and READ RX buffer.
    /// -------------------------------------------------------------------------*
    /// (1). Change mode by writing to CANCTRL register with the settings bitmask.    
    /// (2). Create a new CAN frame and convert to byte array.
    /// (3). Perform 'Load TX buffer' instruction, loading the CAN byte frame.     
    /// (4). Initiate transmission by setting the TXREQ bit (TXBnCTRL[3]) for each buffer.
    /// (5). Poll the target RX buffer, checking if message has been received. Then read RX! 
    /// -------------------------------------------------------------------------*    
    /// Example format of byte message: ...
    /// ... let byte_msg: [u8; 3] = [InstructionCommand::Write as u8, MCP2515Register::TXB0CTRL as u8, data];
    pub fn loopback_test(&mut self){
        defmt::info!("Loopback Test:");
        let settings = CanControllSettings{
            mode: OperationTypes::Loopback,
            clken: false,
            clkpre: CLKPRE::DIV1,
            osm: false,
            abat: false,
        };

        self.change_settings(settings); 

        let data: [u8; 4] = [0xDE, 0xAD, 0xBE, 0xEF];
        let dummy_data = "dummy123".as_bytes();
        defmt::println!("Dummy data length: {:?}", dummy_data.len());

        let dummy_id = StandardId::new(0x1).unwrap();
        let mut frame = CanMessage::new(embedded_can::Id::Standard(dummy_id), &dummy_data).unwrap();
        //let mut frame = CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), dummy_data).unwrap();
       
        let mut frame_id: u16 = 0x0000; 
        if let embedded_can::Id::Standard(id) = frame.id {
            frame_id = id.as_raw();
        }

        //self.read_status();
        self.load_tx_buffer(TXBN::TXB0, frame);
        self.poll_rx(RXBN::RXB0);
        //self.rx_status();
    }

    fn poll_rx(&mut self, rx: RXBN){
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
                    if (rx_status & (1 << 2) != 0) {
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
    /// Format for setting bit: N = N | (1<<k)
    pub fn get_canctrl_mask(&mut self, canctrl_settings: CanControllSettings) -> u8 {
        let mut canctrl_byte = 0u8; // (1): data_byte |= (reg_value << reg_pos)
        let mode_bits: u8 = (canctrl_settings.mode as u8) << 5; //REQOP[2:0] (bits 7-5)
        let prescale: u8 = canctrl_settings.clkpre as u8; //CLKPRE[1:0] (bits 1-0)
        let clk_enabled = canctrl_settings.clken;
         
        canctrl_byte |= mode_bits; //(2)
        defmt::println!("Mode bitmask: 0b{:08b}", mode_bits);
        defmt::println!("Mode bitmask applied to CANCTRL register: 0b{:08b}", canctrl_byte);


        if (canctrl_settings.abat == true){
            canctrl_byte |= 1 << 4;
        }

        if (canctrl_settings.osm == true){
            canctrl_byte |= 1 << 3;
        }

        if (clk_enabled == true){
            canctrl_byte |= 1 << 2;
        }else {
            canctrl_byte &= !(1 << 2); 
        }

        canctrl_byte |= prescale; //(3)
        defmt::println!("3. CANCTRL bitmask after applying settings: 0b{:08b}", canctrl_byte);

        return canctrl_byte;
    }

    fn enable_interrupt(&mut self, caninte: CanInte){
        // The TXnIE bit in the CANINTE register need to be toggled/set to enable interrupt.
        let mut bitmask = 0u8;
        //self.write_register(MCP2515Register::CANINTE, data)
    }

    fn read_status(&mut self){
        let instruction_msg: [u8; 3] = [InstructionCommand::Status as u8, 0x00, 0x00];
        let mut data_out: [u8; 3] = [0; 3];
        self.cs.set_low();
        self.spi.transfer(&mut data_out, &instruction_msg);  
        self.cs.set_high();
        defmt::info!("Read Status: {:08b}", data_out);
    }

    fn rx_status(&mut self){
        let instruction_msg: [u8; 3] = [InstructionCommand::RxStatus as u8, 0x00, 0x00];
        let mut data_out: [u8; 3] = [0; 3];
        self.cs.set_low();
        self.spi.transfer(&mut data_out, &instruction_msg);  
        self.cs.set_high();
        defmt::info!("RX status, Sent (MOSI): {:08b}, Received (MISO): {:08b}", instruction_msg, data_out);
    }

    fn request_to_send(&mut self, buffer: TXBN){
        let rts_instruction: u8 = match buffer {
            TXBN::TXB0 => (InstructionCommand::RTS as u8 | 0x01), 
            TXBN::TXB1 => (InstructionCommand::RTS as u8 | 0x02),
            TXBN::TXB2 => (InstructionCommand::RTS as u8 | 0x04),
        };

        let mut instruction_msg: [u8; 1] = [rts_instruction];
        self.cs.set_low();
        self.write(&instruction_msg);
        self.cs.set_high();
    }

    fn load_tx_buffer(&mut self, buffer: TXBN, mut data_in: CanMessage) {
        defmt::info!("Load TX buffer instruction:");
        
        let instruction = match buffer {
            TXBN::TXB0 => InstructionCommand::LoadTX as u8, // LoadTx instruction = 0x40 = 0100_0abc
            TXBN::TXB1 => InstructionCommand::LoadTX as u8 | 0x2,
            TXBN::TXB2 => InstructionCommand::LoadTX as u8 | 0x4,
        };

        let mut reg_offset = 0x01 as u8;
        let mut address = buffer as u8;
        let mut data_bytes = data_in.to_bytes();
        defmt::info!("data frame as bytes to load: {:08b}", data_bytes);

        self.cs.set_low();   
        /* Transaction START ------------------------------- */
        self.spi.write(&[instruction]);
        defmt::info!("Writing Load TX instruction, Sent (MOSI): {:08b}, Received (MISO) = High-Impedance", [instruction]);

        for data in data_bytes.into_iter(){
            let next_address = address + reg_offset; 
            defmt::println!("Load TX, address: {:08b}, data in: {:08b}", next_address, data);
            //let mut instruction_msg: [u8; 3] = [InstructionCommand::Write as u8, next_address, data];
            self.spi.write(&[data]); 
            reg_offset+=0x01;
        }
        self.cs.set_high();
        /* Transaction END ---------------------------------- */
        
        self.tx_pending(true);
        let tx_ctrl = self.read_register(MCP2515Register::TXB0CTRL, 0x00).unwrap();
        defmt::info!("TXB0CTRL register after load_tx: 0b{:08b}", tx_ctrl);

    }

    fn read_rx_buffer(&mut self, buffer: RXBN) -> Result<[u8; 13], SPI::Error> {
        defmt::info!("Read RX buffer instruction:");
        
        const DONT_CARE: u8 = 0x00;
        let mut rx_data: [u8; 3] = [0; 3];
        let mut rx_buffer = [0u8; 13];
        let mut reg_offset = 0x01 as u8;
         
        let mut address = match buffer {
            RXBN::RXB0 => MCP2515Register::RXB0CTRL as u8, 
            RXBN::RXB1 => MCP2515Register::RXB1CTRL as u8,
        };

         let mut instruction = match buffer {
            RXBN::RXB0 => InstructionCommand::ReadRxb0 as u8, 
            RXBN::RXB1 => InstructionCommand::ReadRxb1 as u8,  
        };

        self.cs.set_low();

        // Send the read RX buffer instruction
        let mut instruction_buf = [instruction];
        self.spi.write(&[instruction]);
        defmt::info!("Read RX buffer instruction, Sent (MOSI): {:08b}, Received (MISO): {:08b}", [instruction], instruction_buf);
 
        //self.spi.transfer(&mut rx_buffer, &[0; 14]).ok(); // Read the 14 bytes of CAN data
        self.spi.transfer(&mut rx_buffer, &[0; 13]).ok(); // Read the 14 bytes of CAN data
 
        self.cs.set_high();
        
        let mut frame = CanMessage::from(rx_buffer);
        let frame_byte = frame.to_bytes();
        
        defmt::println!("Reading RX buffer (received): {:08b}", rx_buffer);
        defmt::println!("Reading RX buffer frame.to_bytes: {:08b}", frame_byte);

        Ok(rx_buffer)
    }

    fn reset_instruction(&mut self){
        self.cs.set_low();
        self.spi.write(&[InstructionCommand::Reset as u8]);
        self.cs.set_high();
    }

    fn tx_pending(&mut self, is_pending: bool){
        defmt::info!("Logic for tx_pending:");
        const CLEAR: u8 = 0u8;
        const FILTERMASKOFF: u8 = 0b0110_0000;
        let mut reg_bits = 0u8;
        reg_bits |= 1 << 3;

        /* When 'is_pending' flag is false, it indicate that transmit buffer is not pending
         * transmission, and should be called before writing to the transmit buffer. Setting the
         * flag to true, would flag a message buffer as being ready for transmission. */
        if (is_pending == false){
            self.write_register(MCP2515Register::TXB0CTRL, CLEAR);
            self.write_register(MCP2515Register::TXB1CTRL, CLEAR);
            self.write_register(MCP2515Register::TXB2CTRL, CLEAR);
            self.write_register(MCP2515Register::RXB0CTRL, FILTERMASKOFF);
            self.write_register(MCP2515Register::RXB1CTRL, FILTERMASKOFF);
        }else {
            defmt::println!("is_pending transmission = true\nWriting {:08b} to TXB0CTRL", reg_bits);
            self.write_register(MCP2515Register::TXB0CTRL, reg_bits);
            //self.write_register(MCP2515Register::TXB1CTRL, reg_bits);
            //self.write_register(MCP2515Register::TXB2CTRL, reg_bits); 
        }
        
    }

    /// This would apply and configure the Can controller settings for the module.
    fn setup_configuration(&mut self, can_settings: CanSettings){

        defmt::info!("Setting up Configuration: ");

        defmt::println!("Calling Reset instruction over SPI");
        self.reset_instruction();
        
        let canstat_reg = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let mut instruction_msg: [u8; 3] = [InstructionCommand::Read as u8, MCP2515Register::CANSTAT as u8, 0x00 as u8];
        
        defmt::println!("MOSI sending instruction: {:08b}", instruction_msg);         
        defmt::println!("Read register instruction (CANSTAT), MISO returned: {:08b}", canstat_reg); 
        
        self.rx_status();

        let canctrl_byte = self.get_canctrl_mask(can_settings.canctrl);

        self.write_register(MCP2515Register::CANCTRL, canctrl_byte);
        let canctrl_bits = self.read_register(MCP2515Register::CANCTRL, 0x00).unwrap(); 
        
        /* The requested mode must be varified by reading the OPMOD[2:0] bits (CANSTAT[7:5])*/
        let canstat_new =self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        defmt::println!("CANCTRL after setup is: {:08b}", canctrl_bits);  
        defmt::println!("CANSTAT after setup is: {:08b}", canstat_new);  
       
        self.setup_bitrate();
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

    
    fn setup_bitrate(&mut self){
        
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
        self.write_register(MCP2515Register::CNF3, 0x85);

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

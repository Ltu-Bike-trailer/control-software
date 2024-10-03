#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]

use super::message::CanMessage;

use core::borrow::Borrow;
use core::{str, usize};
use digital::ErrorKind;
use embedded_can::StandardId;
use nrf52840_hal::comp::OperationMode;
use nrf52840_hal::{self as _, spi};
use nrf52840_hal::gpio::{Level, Port};
use nrf52840_hal::gpiote::{Gpiote, GpioteInputPin};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use embedded_hal::*;
use embedded_hal::digital::{OutputPin, InputPin};
use embedded_hal::spi::SpiBus;
use embedded_can::{Error, Frame, nb::Can};
use nb;
use defmt::Format;

//TODO: - Add CanModule struct type field to the CanDriver. 

/// 
pub struct CanDriver<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin>{
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
///REQOP: Request Operation Mode bits <2:0>
//    000 = Set Normal Operation mode
//    001 = Set Sleep mode
//    010 = Set Loopback mode
//    011 = Set Listen-only mode
//    100 = Set Configuration mode
#[derive(Clone, Copy)]
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

#[derive(Clone, Copy)]
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
    CANINTF = 0x2C, // 00101100

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

#[derive(Clone, Copy)]
pub struct CanControllSettings{
    mode: OperationTypes,
    clken: bool,
    clkpre: CLKPRE,
    abat: bool, // ABAT: Abort All Pending Transmission bit
    osm: bool, // OSM: One-Shot Mode (1 = Enabled, 0 = disabled)
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum CanInte{
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
pub enum InterruptFlagCode{
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

// let settings = CanSettings::new(..).
#[derive(Clone, Copy)]
pub struct CanSettings{
    pub canctrl: CanControllSettings,
    pub mcp_clk: McpClock, // Clock Frequency for MCP2515. 
    pub can_bitrate: CanBitrate,
    pub interrupts: u8, // mask for the enabled interrupt type bits
}


impl CanControllSettings {
    pub fn new(mode: OperationTypes, clken: bool, clkpre: CLKPRE, abat: bool, osm: bool) -> Self{
        Self{
            mode, clken, clkpre, abat, osm
        }
    }

}

impl Default for CanSettings {
    fn default() -> CanSettings {
        let canctrl_settings = CanControllSettings::new(OperationTypes::Configuration, false, CLKPRE::DIV1, false, false);
        Self {
            canctrl: canctrl_settings,
            mcp_clk: McpClock::MCP8,
            can_bitrate: CanBitrate::CAN125,
            interrupts: 0u8,
        }
    }
}

impl CanSettings{
    fn enable_interrupt(mut self, interrupt_type: CanInte) -> Self {
        self.interrupts |= interrupt_type as u8;
        self
    }
    fn enable_interrupts(mut self, interrupt_types: &[CanInte]) -> Self {
        interrupt_types.iter().for_each(|el| self.interrupts |= *el as u8);
        self
    }

    fn canctrl_bitmask(&mut self){
        
    }
}

impl TryFrom<u8> for InterruptFlagCode{
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


#[derive(Debug)]
pub enum CanError{}

impl embedded_can::Error for CanError{
    fn kind(&self) -> embedded_can::ErrorKind {
        embedded_can::ErrorKind::Other
    }
}

/// required trait methods goes here...
impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> Can for CanDriver<SPI, PIN, PININT> {

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

impl<SPI: embedded_hal::spi::SpiBus, PIN: OutputPin, PININT: InputPin> CanDriver<SPI, PIN, PININT>{

    pub fn init(spi: SPI, cs: PIN, interrupt_pin: PININT, can_settings: CanSettings) -> Self{
        let initial_settings = can_settings.borrow(); 
        let mut driver = Self{spi, cs, interrupt_pin, can_settings};
         
        driver
            .setup_configuration(initial_settings) // Setup CANCTRL register.
            .setup_bitrate() // Setup bitrate and clk freq, in registers: CNF1, CNF2, & CNF3.
            .setup_interrupt(initial_settings) // Setup interrupts in CANINTE register.
            .tx_pending(false); // CLEAR the TXREQ bits indicating the tx buffer is not pending before writing. 
            
        let received_reg = driver.read_register(MCP2515Register::TXB0CTRL, 0x00).unwrap();
        defmt::println!("TXREQ register: {:08b}", received_reg);
        
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

        //defmt::info!("Read instruction, Sent (MOSI): {:08b}, Received (MISO): {:08b}", read_msg, read_buf);
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
        //defmt::info!("Write instruction, Sent (MOSI): {:08b}, MISO = High-Impedance", byte_msg);
        Ok(())
    }

    fn change_settings(&mut self, settings: CanControllSettings){
        self.can_settings.canctrl = settings;

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
    pub fn loopback_test(&mut self, can_msg: CanMessage){
        defmt::info!("Loopback Test:");
        let settings = CanControllSettings{
            mode: OperationTypes::Loopback,
            clken: false,
            clkpre: CLKPRE::DIV1,
            osm: false,
            abat: false,
        };

        self.change_settings(settings); 

        let dummy_data = "dummy".as_bytes();
        defmt::println!("Dummy data length: {:?}", dummy_data.len());

        let dummy_id = StandardId::new(0x1).unwrap();
        let mut frame = CanMessage::new(embedded_can::Id::Standard(dummy_id), &dummy_data).unwrap();
        //let mut frame = CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), dummy_data).unwrap();

        //self.read_status();
        self.load_tx_buffer(TXBN::TXB0, can_msg);
        //self.load_tx_buffer(TXBN::TXB0, frame);
        //self.poll_rx(RXBN::RXB0);
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

    /// This would CLEAR the TXREQ bits indicating the tx buffer is not pending before writing.
    /// When 'is_pending' flag is false, it indicate that transmit buffer is not pending transmission. 
    /// Hence should be called before writing to the transmit buffer. 
    /// Setting the flag to true, would flag a message buffer as being ready for transmission.
    fn tx_pending(&mut self, is_pending: bool) -> &mut Self{
        defmt::info!("Logic for tx_pending:");
        const CLEAR: u8 = 0u8;
        const FILTERMASKOFF: u8 = 0b0110_0000;
        let mut reg_bits = 0u8;
        reg_bits |= 1 << 3;

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

        self
    }

    /// This would apply and configure the Can controller settings for the module.
    fn setup_configuration(&mut self, can_settings: &CanSettings) -> &mut Self{

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

        self
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

    fn setup_bitrate(&mut self) -> &mut Self{
        
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

        self
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
  
      fn setup_interrupt(&mut self, can_settings: &CanSettings) -> &mut Self {

        // The TXnIE bit in the CANINTE register need to be toggled/set to enable interrupt.
        // CANINTE - enable 
        // CANINTF - interrupt flags
        // CANSTAT ICOD - interrupt flag code bits. 

        /*When a message is moved into either of the receive
        buffers, the appropriate CANINTF.RXnIF bit is set. This
        bit must be cleared by the MCU in order to allow a new
        message to be received into the buffer. This bit
        provides a positive lockout to ensure that the MCU has
        finished with the message before the MCP2515
        attempts to load a new message into the receive buffer.
        If the CANINTE.RXnIE bit is set, an interrupt will be
        generated on the INT pin to indicate that a valid
        message has been received. */ 
        
        //NOTE:: 
        // The source of a pending interrupt is indicated in the CANSTAT.ICOD reg.
        // ----------------------------------------------------------------------
        // 1. The INT pin will remain low until all interrupts have been reset by the MCU.
        //      - The CANSTAT.ICOD bits will reflect the code for the highest priority interrupt.
        //... Where interrupts are internally prioritized that lower ICOD value = higher priority.
        // ----------------------------------------------------------------------
        // 2. Once the highest priority interrupt condition has been cleared...
        // ...the next highest priority interrupt that is pending (if any) will be reflected...
        // ...by the ICOD bits. 
        

        let settings = can_settings.enable_interrupts(&[CanInte::TX2IE, CanInte::TX1IE, CanInte::TX0IE, CanInte::RX1IE, CanInte::RX0IE]);
        self.can_settings.interrupts = settings.interrupts;

        defmt::info!("Enable interrupt mask: {:08b}", settings.interrupts);
        self.write_register(MCP2515Register::CANINTE, settings.interrupts);
        self.read_register(MCP2515Register::CANINTE, 0x00);

        self
    }

    fn clear_interrupt_flag(&mut self, canintf: u8, bit_pos: u8){
        defmt::info!("---Inside 'clear_interrupt_flag' logic---");
        let mut mask_bytes: u8 = self.can_settings.interrupts; // enabled target interrupt bits.
                         
        // determine what value the modified bits will change
        
        //CanInte::TX0IE = 0b0000_0100
        //                     AND
        // value =         0b1111_1011
        // -> 
        let mut data_bytes = canintf ^ (1 << bit_pos); //0bxxxx_1011 for clearing TX0IF
        
        self.cs.set_low();
        self.spi.write(
            &[InstructionCommand::Modify as u8, 
            MCP2515Register::CANINTF as u8, 
            mask_bytes, 
            data_bytes]);
        self.cs.set_high();

        let canintf_after = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        defmt::info!("Bit Modify Instruction:\n
            Mask bytes: {:08b}\n
            Data byte: {:08b}\n
            CANINTF previous content: {:08b}\n
            CANINTF after Bit Modify: {:08b}", mask_bytes, data_bytes, canintf, canintf_after);
    }

      
    pub fn interrupt_decode(&mut self) -> InterruptFlagCode {
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        let mut interrupt_code = (canstat & 0b0000_1110) >> 1; // clear OPMOD bits and shift right by 1. 
        defmt::info!("Interrupt decode logic (ICOD), clear OPMOD and shift by 1 mask: {:08b}", interrupt_code);
     
        let flag_code = InterruptFlagCode::try_from(interrupt_code).unwrap(); 
        defmt::info!("Received the Interrupt type: {:?}", flag_code); 
        flag_code
    }

    pub fn handle_interrupt(&mut self, received_interrupt: InterruptFlagCode) {
        // Pass the decoded interrupt from 'interrupt_decode()' and handle it.
        // E.g., clear neccessary regs such as clearing the appropriate CANINTF bits.
        defmt::info!("---Inside 'handle_interrupt' logic---");
        defmt::info!("Handle Pending Interrupt: {:?}", received_interrupt);
        
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        let canstat = self.read_register(MCP2515Register::CANSTAT, 0x00).unwrap();
        
        defmt::info!("CANINTF register bits: {:08b}", canintf);
        defmt::info!("CANSTAT register bits: {:08b}", canstat);

        match received_interrupt {
            InterruptFlagCode::NoInterrupt =>{},
            InterruptFlagCode::ErrorInterrupt =>{},
            InterruptFlagCode::WakeUpInterrupt =>{},
            InterruptFlagCode::TXB0Interrupt => {
                defmt::println!("TXB0 successfully transmitted a message!");
                //TODO: Clear the associated CANINTF bit for TXnIE below
                self.clear_interrupt_flag(canintf, 2);
            },
            InterruptFlagCode::TXB1Interrupt => {},
            InterruptFlagCode::TXB2Interrupt => {},
            InterruptFlagCode::RXB0Interrupt => {
                defmt::println!("RXB0 received a message!");
                //TODO: FIX a try_from logic to map CANSTAT register as u8 to RXB0 /RXB1. 
                if (canintf & (1 << 0) != 0) {}
                
                let received_byte_frame = self.read_rx_buffer(RXBN::RXB0).unwrap();
                defmt::info!("Received byte frame: {:08b}", received_byte_frame);
                self.clear_interrupt_flag(canintf, 0);
            },
            InterruptFlagCode::RXB1Interrupt => {
                defmt::println!("RXB1 received a message!");
                let received_byte_frame = self.read_rx_buffer(RXBN::RXB1).unwrap();
                defmt::info!("Received byte frame: {:08b}", received_byte_frame); 
            },
        }
    }

    pub fn interrupt_is_cleared(&mut self) -> bool{
        const ZERO: u8 = 0u8;
        let canintf = self.read_register(MCP2515Register::CANINTF, 0x00).unwrap();
        if (canintf == ZERO){
            true
        }else{
            false
        }
    }
    

}





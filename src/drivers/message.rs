//! CAN frame message support.
//! ## Usage

#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]

use core::{str, usize};
use digital::ErrorKind;
use embedded_can::StandardId;
use nrf52840_hal::comp::OperationMode;
use nrf52840_hal::pac::i2s::config::format;
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
use defmt::{Format, Formatter, write};

/// Represent a CAN message. 
#[derive(Debug)]
pub struct CanMessage{
    /// CAN identifier, can be either Standard or Extended. 
    pub id: embedded_can::Id,
    /// Data Length Code (DLC) - length of the data byte fields. 
    pub dlc: u8,
    /// Data bytes associated to a CAN frame. 
    ///
    /// Every RX/TX buffer, can use the data register D7 to D0 (8 bytes max).
    pub data: [u8; 8], // D7 - D0
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


/// Here goes custom logic for the CanMessage struct type
impl CanMessage {
    /// Creates a new CAN message. 
    ///
    /// Would fail, if the provided data, exceedes 8 bytes. 
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
        can_msg.data[.._dlc as usize].copy_from_slice(data);
        return Some(can_msg);
    }

    /// Converts the SIDH + SIDL of 11 bits to 16 bits.
    pub fn id_raw(&mut self) -> u16 {
        let mut raw_id: u16;        
        if let embedded_can::Id::Standard(id) = self.id {
            raw_id = id.as_raw();
        } else {
            raw_id = 0;
        }

        //TODO: - Replace with:
        let full_standardid_as_u16 = raw_id << 5;

        let sidh = (raw_id >> 3) as u8;  // Most significant byte
        let sidl = (raw_id as u8 & 0x07) << 5;  // Least significant byte, 0x07 = 00000_0111
        let combined = ((sidh as u16) << 8) | (sidl as u16);
        //defmt::info!("sidh: {:08b}, sidl: {:08b}, 11bit ID: {:016b}", sidh, sidl, combined);
        //defmt::info!("full_standardid_as_u16: {:016b}", full_standardid_as_u16);
        full_standardid_as_u16
    }

    /// Prints the ID, DLC, Data fields of a CAN message frame. 
    pub fn print_frame(&mut self){
        let id = self.id_raw() >> 5;
        let dlc = self.dlc;
        let data = self.data();
        defmt::info!("Id({:x}), DLC({:x}), Data({:08b})", id, dlc, data);
    }

    /// Maps a CAN message to a byte sequence.
    /// [0] = SIDH
    /// [1] = SIDL
    /// [2] = EID8
    /// [3] = EID0
    /// [4] = DLC
    /// [5] = D0
    /// [6] = D1
    /// [7] = D2
    /// [8] = D3
    /// [9] = D4
    /// [10] = D5
    /// [11] = D6
    /// [12] = D7
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

        /*
        defmt::println!("to_bytes: \nSIDH: {:08b}\nSIDL: {:08b}\nEID8: {:08b}\nEID0: {:08b}", 
            sidh, sidl, extended_id8, extended_id0);
        */
        let data_start = 5 as usize;
        let data_end = data_start + self.dlc(); 
        //defmt::info!("Inside to_bytes logic, dlc: {:08b}", self.dlc());
        /*
        byte_frame = [sidh, sidl, extended_id8, extended_id0, self.dlc() as u8, 
            self.data[0], self.data[1], self.data[2], self.data[3], self.data[4], 
            self.data[5], self.data[6], self.data[7]];
        */
        byte_frame[0..data_start].copy_from_slice(&[sidh, sidl, extended_id8, extended_id0, self.dlc() as u8]);
        byte_frame[data_start..data_end].copy_from_slice(&self.data[0..self.dlc()]); 

        //defmt::info!("In to_bytes logic, self.data() contains: {:08b}", self.data());
        //defmt::info!("In to_bytes logic, self.data() vs self.data field: {:08b}", self.data);
       
        return byte_frame;
    }

    /*
    pub fn data_to_string(data: &[u8]) -> &str{
        let data_str = str::from_utf8(&data).unwrap();
        data_str
    }
    */
    
}

///This From trait would parse a type T into a CanMessage frame.
impl From<[u8;13]> for CanMessage{
    fn from(byte_data: [u8;13]) -> Self {
        //let mut data = [0u8; 8];
        let mut id_bytes: [u8; 2]; 
        let mut raw_id: u16 = 0u16;
        
        let slice_sidh = byte_data[0] as u16; //0000_0000_xxxx_xxxx
        let slice_sidl = (byte_data[1] & 0xE0) as u16; // 0xE0 = 11100000, as u16 0000_0000_1110_0000 
        let dlc = byte_data[4];
        
        let data_start = 5 as usize;
        let data_end = data_start + dlc as usize; 
        //defmt::info!("dlc: {:?}", dlc);

        //defmt::info!("data range: {:?}:{:?}", data_start, data_end);

        let mut _data = &byte_data[data_start..];
        
        //data[0..].copy_from_slice(&byte_data[5..]);
        // Shift MSB u16 SIDH part and OR the LSB u16 SIDL part 
        raw_id = ((slice_sidh << 3) + (slice_sidl >> 5));
       
        let sidh = (raw_id >> 3) as u8;  // Most significant byte
        let sidl = (raw_id as u8 & 0x07) << 5;  // Least significant byte, 0x07 = 00000_0111
        //let combined = (((slice_sidh) << 8) | (slice_sidl)) >> 5; // 

        //defmt::println!("byte_data[0] = SIDH reg: {:08b}", byte_data[0]);
        //defmt::println!("byte_data[1] = SIDL reg: {:08b}", byte_data[1]);
        //defmt::println!("CanMessage::from, lower_bits: {:016b}", slice_sidl);
        //defmt::println!("CanMessage::from, raw_id: {:016b}", raw_id);
        //defmt::println!("data length (DLC): {:?}", _data.len());        
       // defmt::println!("CanMessage::from, data slice: {:08b}, data as str: {}", _data, CanMessage::data_to_string(_data));

        //WARN: - Would return None, if raw is out of range of an 11 bit integer.   
        let frame_id = StandardId::new(raw_id).unwrap();
        let msg = CanMessage::new(embedded_can::Id::Standard(frame_id), &_data).unwrap();
        
        return msg;
    }
}




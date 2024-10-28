//! HX711 driver - 24-bit Analog-to-Digital Converter (ADC).
//!
//! ## Note
//!
//! This driver, is still under progress...
#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]

use core::{borrow::Borrow, str};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt::{unwrap, Format};
use defmt_rtt as _;
use embedded_hal::digital::{OutputPin, InputPin};

pub struct Hx711Driver<PINOUT: OutputPin, PININ: InputPin> {
    pd_sck: PININ,
    dout: PINOUT,
}

impl<PINOUT: OutputPin, PININ: InputPin>
    Hx711Driver<PINOUT, PININ>
{
    /// Initialize and creates a new HX711 driver instance.
    pub fn init(sck: PININ, digital_out: PINOUT) -> Self {
        let mut driver = Self {
            pd_sck: sck,
            dout: digital_out,
        };
        driver
    }

    /// When DOUT is low, data is ready for reception.
    /// It starts with the MSB and ends with LSB. 
    ///
    /// "By applying 25~27 positive clock pulses at the
    /// PD_SCK pin, data is shifted out from the DOUT output pin. 
    /// Each PD_SCK pulse shifts out one bit,
    /// starting with the MSB bit first, until all 24 bits are
    /// shifted out".
    fn read_data(&mut self){
        
    }

    fn decode_data(&mut self, data_in: u32){}
}

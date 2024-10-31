//! HX711 driver - 24-bit Analog-to-Digital Converter (ADC).
//!
//! ## Note
//!
//! This driver, is still under progress...
#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]


use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt::{unwrap, Format};
use defmt_rtt as _;
use embedded_hal::{delay::DelayNs, digital::{InputPin, OutputPin}};
use rtic_monotonics::{
    nrf::timer::{ExtU64, Timer0 as Mono},
    systick::fugit::Duration,
    Monotonic,
};

pub type Instant = <Mono as rtic_monotonics::Monotonic>::Instant;

pub struct Hx711Driver<PINOUT: OutputPin, PININ: InputPin> {
    pub pd_sck: PINOUT,
    pub dout: PININ,
    pub dout_data: i32,
    pub bit_index: u8,
    /// The gain act as the 'stop index'.
    /// Where:
    /// - Gain(128) => 25 bit pulses.
    /// - Gain(32) => 26 bit pulses.
    /// - Gain(64) => 27 bit pulses. 
    pub gain: Gain,
}

pub struct DataSample{
    data_out: i32,
    next_gain: Gain,
}

#[repr(u8)]
#[derive(Clone, Copy)]
pub enum Gain{
    Apply128 = 25,
    Apply32 = 26,
    Apply64 = 27,
}

/// T1: DOUT falling edge to PD_SCK rising edge.
/// T2: PD_SCK rising edge to DOUT data ready.
/// T3: PD_SCK high time. 
/// T4: PD_SCK low time.
pub enum DataTiming{
    /// MIN 0.1 µs .
    T1,
    /// MAX 0.1 µs.
    T2,
    /// MIN 0.2 µs and MAX 50 µs. 
    T3,
    /// MIN 0.2 µs.   
    T4,
}

impl<const NOM:u32, const DENOM:u32> Into<Duration<u64, NOM, DENOM>> for DataTiming{
    fn into(self) -> Duration<u64, NOM, DENOM> {
       //DataTiming::T1 =>rtic_monotonics::nrf::rtc::ExtU64::micros(1),
        match self {
            Self::T1 => 100.nanos(), // 0.1 µs
            Self::T2 => 10.nanos(), // 0.01 µs
            Self::T3 => 300.nanos(), // 0.3 µs
            Self::T4 => 300.nanos(), // 0.3 µs 
        }
    }
}

impl Into<u8> for Gain{
    fn into(self) -> u8 {
        match self {
            Self::Apply128 => 25,
            Self::Apply32 => 26,
            Self::Apply64 => 27,
        }
    }
}

impl<PINOUT: OutputPin, PININ: InputPin>
    Hx711Driver<PINOUT, PININ>
{
    pub const STOP_INDEX: u8 = 25 - 1;

    /// Initialize and creates a new HX711 driver instance.
    pub fn init(sck: PINOUT, digital_out: PININ, gain_amount: Gain) -> Self {
        let mut driver = Self {
            pd_sck: sck,
            dout: digital_out,
            dout_data: 0i32,
            bit_index: 0,
            gain: gain_amount,

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
    async fn read_data_pulse(&mut self, stop_count: u8) -> bool {
        
        let mut read_done = false;
        self.pd_sck.set_high();

        if (self.bit_index < 24){
            Mono::delay(DataTiming::T2.into()).await;
            let read = unsafe { self.dout.is_high().unwrap_unchecked() } as i32;
            self.dout_data = (self.dout_data << 1) | read;
        }

        if (self.bit_index == stop_count) {
            // when self.bit_index > 23
            self.bit_index = 0;
            read_done = true;
        }
        self.bit_index += 1;
        
        Mono::delay(DataTiming::T3.into()).await;        
        self.pd_sck.set_low();
        Mono::delay(DataTiming::T4.into()).await;        
        
        read_done
    }

    fn next_conversion_gain(&mut self, gain_amount: Gain) -> u8{
        self.gain = gain_amount;
        let stop_index: u8 = match gain_amount {
            Gain::Apply128 => Gain::Apply128.into(),
            Gain::Apply32 => Gain::Apply32.into(),
            Gain::Apply64 => Gain::Apply64.into(),
        };
        stop_index - 1
    }


    pub async fn read_full_period(&mut self, next_gain: Gain) -> i32{
        let stop_count = self.next_conversion_gain(next_gain);
        Mono::delay(DataTiming::T1.into()).await; 
        loop  {
            // Apply 25~27 positive clock pulses here
            let full_read = self.read_data_pulse(stop_count).await;
            if (full_read){
                let data: i32 = self.dout_data;
                self.dout_data = 0i32;
                return data;
            }
        }
    }

    pub fn decode_data(&mut self, data_in: u32) -> DataSample{
        // Two's complement logic and apply gain and return data...
        // Can I use i32 directly (two's complement) or do I need to wrap, 
        // and convert the u32 to i32? 
        let data_sample = unsafe {i32::try_from(data_in).unwrap_unchecked()};

        // Does the next conversion gain reflect directly in the output bits for one period?

        //TODO: - Define a new data struct, that return the gain, and the data bits.
        DataSample{
            data_out: data_sample,
            next_gain: self.gain,
        }
        
    }
}

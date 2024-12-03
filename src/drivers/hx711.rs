//! HX711 driver - 24-bit Analog-to-Digital Converter (ADC).
//!
//! ## Note
//!
//! This driver, is still under progress...
#![no_main]
#![no_std]
#![allow(unused)]
#![allow(missing_docs)]
#![feature(generic_arg_infer)]
#![allow(clippy::future_not_send)]
use core::{future::Future, ops::RangeBounds};

use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt::{unwrap, Format};
use defmt_rtt as _;
use embedded_hal::{
    delay::DelayNs,
    digital::{InputPin, OutputPin},
};
use rtic_monotonics::{
    fugit::{Duration, ExtU64},
    Monotonic,
};

/// Hx711 driver struct.
pub struct Hx711Driver<PINOUT: OutputPin + Send, PININ: InputPin + Send> {
    /// Clock pin, for `Power down control (high active) and serial clock
    /// input`.
    pub pd_sck: PINOUT,
    /// Serial data output pin, act as "data in" to MCU master.
    pub dout: PININ,
    /// The associated data to the `dout` pin.
    pub dout_data: i32,
    /// Act as a counter, to check until all 24 bits are shifted out.
    bit_index: u8,
    /// Applied gain-value for next conversion period.
    gain: Gain,
    /// Helper tuple struct for checking and getting valid timings.
    data_timings: ValidTimings,
}

#[derive(Debug)]
/// This represent one data sample for one period conversion.
pub struct DataSample {
    /// Represent the 24-bit signed (two's complement) data output.
    pub data_out: i32,
    /// Represent the applied gain for next conversion period.     
    pub next_gain: Gain,
}

/// The gain act as the 'stop index' for the pulses.
/// Where:
/// - Gain(128) => 25 bit pulses.
/// - Gain(32) => 26 bit pulses.
/// - Gain(64) => 27 bit pulses.
#[repr(u8)]
#[derive(Debug, Clone, Copy)]
pub enum Gain {
    /// This would apply 128 in gain for next upcoming reading period.
    Apply128 = 25,
    /// This would apply 32 in gain for next upcoming reading period.
    Apply32 = 26,
    /// This would apply 27 in gain for next upcoming reading period.
    Apply64 = 27,
}

/// T1: DOUT falling edge to `PD_SCK` rising edge.
/// T2: `PD_SCK` rising edge to DOUT data ready.
/// T3: `PD_SCK` high time.
/// T4: `PD_SCK` low time.
#[derive(Debug, Clone, Copy)]
enum DataTiming {
    /// T1: MIN 0.1 µs (100 ns).
    T1(u64),
    /// T2: MAX 0.1 µs (100 ns).
    T2(u64),
    /// T3: MIN 0.2 µs (200 ns) and MAX 50 µs.
    T3(u64),
    /// T4: MIN 0.2 µs (200 ns).   
    T4(u64),
}

/// Error type, for whenever, you try to create `DataTiming` 
/// instances that are not within acceptable range 
#[derive(Debug, Clone, Copy)]
pub struct DataTimingInvalidRange;

/// A tuple struct containing valid `DataTiming` values. 
#[derive(Debug, Clone, Copy)]
pub struct ValidTimings(DataTiming, DataTiming, DataTiming, DataTiming);

impl<const NOM: u32, const DENOM: u32> From<DataTiming> for Duration<u64, NOM, DENOM> {

    #[allow(clippy::match_same_arms)]
    fn from(val: DataTiming) -> Self {
        match val {
            DataTiming::T1(t1) => t1.nanos(), 
            DataTiming::T2(t2) => t2.nanos(),  
            DataTiming::T3(t3) => t3.nanos(),
            DataTiming::T4(t4) => t4.nanos(),
        }
    }
}

impl DataTiming {
    pub const T3_MAX: u64 = 50000; 

    fn valid_timing(&self) -> bool {
        match self {
            Self::T1(val) => *val >= 100,
            Self::T2(val) => *val <= 100,
            Self::T3(val) => (200..=Self::T3_MAX).contains(val),
            Self::T4(val) => *val >= 200,
        }
    }

    fn get_valid_timings(timings: [Self; 4]) -> Result<ValidTimings, DataTimingInvalidRange>{
        for t in &timings{
            if !t.valid_timing(){
                return Err(DataTimingInvalidRange);
            }
        }
        Ok(ValidTimings(timings[0], timings[1], timings[2], timings[3]))
    }

}

impl Default for ValidTimings {
    fn default() -> Self {
        Self(DataTiming::T1(100), DataTiming::T2(10), DataTiming::T3(300), DataTiming::T4(300))
    }
}

impl ValidTimings {
    /// Creates a new tuple struct containing checked duration values. 
    /// That's within the acceptable range.
    /// Returns `Self::default()` if any of the provided timings are invalid.
    #[must_use]
    pub fn new(t1: u64, t2: u64, t3: u64, t4: u64) -> Self {
        let timings: [DataTiming; 4] = [
            DataTiming::T1(t1),
            DataTiming::T2(t2),
            DataTiming::T3(t3),
            DataTiming::T4(t4),
        ];
        DataTiming::get_valid_timings(timings).unwrap_or_else(|_| Self::default())
    }
}

impl From<Gain> for u8 {
    fn from(val: Gain) -> Self {
        match val {
            Gain::Apply128 => 25,
            Gain::Apply32 => 26,
            Gain::Apply64 => 27,
        }
    }
}

impl<PINOUT: OutputPin + Send, PININ: InputPin + Send> Hx711Driver<PINOUT, PININ> {
    /// This is just a placeholder or constant for the calibration parameter.
    ///
    /// #NOTE
    ///
    /// This is just for showcasing, and need implemention for the
    /// `run_calibration` method.
    pub const CALIBRATION_CONST: i32 = 3_065_448_i32;

    /// Initialize and creates a new HX711 driver instance.
    pub const fn init(sck: PINOUT, digital_out: PININ, gain_amount: Gain, timings: ValidTimings) -> Self {
        
        Self {
            pd_sck: sck,
            dout: digital_out,
            dout_data: 0i32,
            bit_index: 0,
            gain: gain_amount,
            data_timings: timings,
        }
    }


    /// When DOUT is low, data is ready for reception.
    /// It starts with the MSB and ends with LSB.
    ///
    /// "By applying 25~27 positive clock pulses at the
    /// `PD_SCK` pin, data is shifted out from the DOUT output pin.
    /// Each `PD_SCK` pulse shifts out one bit,
    /// starting with the MSB bit first, until all 24 bits are
    /// shifted out".
    async fn read_data_pulse<T, const NOM: u32, const DENOM: u32>(&mut self, stop_count: u8) -> bool
    where
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>> + Send,
    {
        let ValidTimings(t1, t2, t3, t4) = self.data_timings;
        
        let mut read_done = false;
        self.pd_sck.set_high();

        if (self.bit_index < 24) {
            T::delay(t2.into()).await;
            let read = i32::from(unsafe { self.dout.is_high().unwrap_unchecked() });
            self.dout_data = (self.dout_data << 1) | read;
        }

        if (self.bit_index == stop_count) {
            // when self.bit_index > 23
            self.bit_index = 0;
            read_done = true;
        }
        self.bit_index += 1;

        T::delay(t3.into()).await;
        self.pd_sck.set_low();
        T::delay(t4.into()).await;

        read_done
    }

    /// This method, would apply the gain for the next data
    /// sample period.
    fn next_conversion_gain(&mut self, gain_amount: Gain) -> u8 {
        self.gain = gain_amount;
        let stop_index: u8 = gain_amount.into();
        stop_index - 1
    }

    /// Reads a 24-bit full period data. Where data is
    /// shifted out on the `dout` pin
    pub async fn read_full_period<T, const NOM: u32, const DENOM: u32>(
        &mut self,
        next_gain: Gain,
    ) -> i32
    where
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>> + Send,
    {
        let ValidTimings(t1, t2, t3, t4) = self.data_timings;
        let stop_count = self.next_conversion_gain(next_gain);
        T::delay(t1.into()).await;
        loop {
            // Apply 25~27 positive clock pulses here
            let full_read = self.read_data_pulse::<T, NOM, DENOM>(stop_count).await;
            if (full_read) {
                let data: i32 = self.dout_data;
                self.dout_data = 0i32;
                return data;
            }
        }
    }

    /// Under progess - but should return the decoded data struct.
    pub fn decode_data(&mut self, data_in: i32) -> DataSample {
        // Two's complement logic and apply gain and return data...
        // Can I use i32 directly (two's complement) or do I need to wrap,
        // and convert the u32 to i32?
        //let data_sample = unsafe { i32::try_from(data_in).unwrap_unchecked() };

        let data_sample = if data_in & 0x0080_0000 != 0 {
            //data_in.wrapping_neg() - Self::CALIBRATION_CONST
            data_in | !0x00FF_FFFF
        } else {
            data_in
        };

        //TODO: - Define a new data struct, that return the gain, and the data bits.
        DataSample {
            data_out: data_sample,
            next_gain: self.gain,
        }
    }

    /// Calibration logic - TODO...
    pub fn run_calibration(&mut self, num_epochs: u8) {
        unimplemented!()
    }
}

//! Defines a simple calibration program for the s-type load cell.
#![allow(warnings, dead_code, unused_variables, unreachable_code)]
#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

// PHASE ORDER: Yellow, blue green.

use controller as _;
// global logger + panicking-behavior + memory layout
use rtic_monotonics::nrf::timer::prelude::*;
nrf_timer4_monotonic!(Mono, 16_000_000);

#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0,TIMER1]
)]
mod app {
    //use embedded_hal::digital::InputPin;
    use nrf52840_hal::{
        saadc::{SaadcConfig, Time},
        Saadc,
    };

    use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {}

    // Local resources go here
    #[local]
    struct Local {}

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        nrf52840_hal::Clocks::new(cx.device.CLOCK).enable_ext_hfosc();
        defmt::info!("Clock config done.");
        Mono::start(cx.device.TIMER4);

        let p0 = nrf52840_hal::gpio::p0::Parts::new(cx.device.P0);

        // ADC configuration.
        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_40US;
        cfg.gain = adc_gain();
        cfg.oversample = nrf52840_hal::saadc::Oversample::OVER256X;
        cfg.reference = nrf52840_hal::saadc::Reference::INTERNAL;

        // The input pin.
        let mut input = p0.p0_29;

        let mut adc = Saadc::new(cx.device.SAADC, cfg);

        // Hack to stop rustc being mad.
        if true {
            loop {
                let ret = adc.read_channel(&mut input).unwrap();
                // We only measure positive values.
                let measured = conv(ret as u16);
                // Scale in between.
                const GAIN: f32 = 10.;
                let converted = GAIN * measured;
                defmt::info!("Measured {}N", converted);
            }
        }
        (Shared {}, Local {})
    }

    use nrf52840_hal::saadc;
    /// Resistance to ground for the current sense pins.
    pub const R_PHASE_SENSE_TO_GROUND: f32 = 10_000.;

    /// Resistance to vin for the current sense pins.
    pub const R_PHASE_SENSE_TO_VIN: f32 = 4_700.;

    /// The shunt resistor that the current sense measures voltage across.
    ///
    /// The Sense resistor is 10 mOhm on the esc.
    pub const R_SHUNT: f32 = 0.01;

    /// The gain that the sensor applies to the differential measurement.
    pub const SENSE_GAIN: f32 = 50.;

    /// The system voltage.
    pub const V_REF: f32 = 0.6;

    /// The gain in the nrf chip.
    pub const ADC_BITS: usize = 14;

    /// The gain of the adc on the nrf chip.
    pub const ADC_GAIN: f32 = 1. / 6.;

    /// The voltage divider used for current sensing.
    /// $I_shunt = (2.5-V_sense *  ( R_5+R_4 ) / R_5) / ( R_shunt  * NågonGain )
    /// $
    ///
    ///
    /// RESULT = [V(P) – V(N)] * (GAIN/REFERENCE) * 1<<(RESOLUTION - m)
    /// V(P) - V(N) = RESULT/((GAIN/REFERENCE) * 1<<(RESOLUTION - m))
    pub const fn raw_adc_to_current_factor() -> f32 {
        //(ADC_GAIN / V_REF) * ((1 << ADC_BITS) as f32) * v_div()
        //(V_REF / ((1 << ADC_BITS) as f32)) / ADC_GAIN

        (V_REF / (ADC_GAIN * (1 << ADC_BITS) as f32) * v_div()) as f32
    }

    /// Returns the voltage divider
    pub const fn v_div() -> f32 {
        (R_PHASE_SENSE_TO_GROUND + R_PHASE_SENSE_TO_VIN) / R_PHASE_SENSE_TO_GROUND
    }

    /// Combines the gain and shunt value in one.
    ///
    /// This is multiplied by a factor of 1000
    pub const fn sensor_gain() -> f32 {
        SENSE_GAIN as f32
    }

    /// Converts the adc value in to mA.
    #[inline(always)]
    fn conv(val: u16) -> f32 {
        2.5 / sensor_gain() - f32::from(val) * raw_adc_to_current_factor() / sensor_gain()
    }

    /// Converts the adc gain to the type representation at compile time.
    pub const fn adc_gain() -> saadc::Gain {
        if ADC_GAIN == 1. / 6. {
            saadc::Gain::GAIN1_6
        } else if ADC_GAIN == 1. / 5. {
            saadc::Gain::GAIN1_5
        } else if ADC_GAIN == 1. / 4. {
            saadc::Gain::GAIN1_4
        } else if ADC_GAIN == 1. / 3. {
            saadc::Gain::GAIN1_3
        } else if ADC_GAIN == 1. / 2. {
            saadc::Gain::GAIN1_2
        } else if ADC_GAIN == 1. {
            saadc::Gain::GAIN1
        } else if ADC_GAIN == 2. {
            saadc::Gain::GAIN2
        } else {
            panic!("Invalid ADC_GAIN")
        }
    }
}

<<<<<<< HEAD
#![allow(clippy::cast_precision_loss)]
#![allow(clippy::pedantic)]
=======
>>>>>>> d3bcbce (rebasing with main)
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
<<<<<<< HEAD
/// $`I_shunt` = (2.5-V_sense *  ( `R_5+R_4` ) / `R_5`) / ( `R_shunt`  *
/// `some_gain` ) $
=======
/// $I_shunt = (2.5-V_sense *  ( R_5+R_4 ) / R_5) / ( R_shunt  * some_gain )
/// $
>>>>>>> d3bcbce (rebasing with main)
///
///
/// RESULT = [V(P) – V(N)] * (GAIN/REFERENCE) * 1<<(RESOLUTION - m)
/// V(P) - V(N) = RESULT/((GAIN/REFERENCE) * 1<<(RESOLUTION - m))
#[must_use]
pub const fn raw_adc_to_current_factor() -> f32 {
    //(ADC_GAIN / V_REF) * ((1 << ADC_BITS) as f32) * v_div()
    //(V_REF / ((1 << ADC_BITS) as f32)) / ADC_GAIN

    //TODO appease clippy
    V_REF / (ADC_GAIN * (1 << ADC_BITS) as f32) * v_div()
}

/// Returns the voltage divider
#[must_use]
pub const fn v_div() -> f32 {
    (R_PHASE_SENSE_TO_GROUND + R_PHASE_SENSE_TO_VIN) / R_PHASE_SENSE_TO_GROUND
}

/// Combines the gain and shunt value in one.
///
/// This is multiplied by a factor of 1000
#[must_use]
pub const fn sensor_gain() -> f32 {
    SENSE_GAIN
}

/// Converts the adc value in to mA.
#[inline(always)]
#[must_use]
pub fn conv(val: u16) -> f32 {
    2.5 / sensor_gain() - f32::from(val) * raw_adc_to_current_factor() / sensor_gain()
}

/// Converts the adc gain to the type representation at compile time.
#[must_use]
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

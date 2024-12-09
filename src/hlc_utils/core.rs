//! Defines core functionality and traits for the module,
//! related to the "High-Level-Controller.
#![deny(warnings, missing_docs)]
#![allow(unused_imports)]

use nrf52840_hal::{
    gpio::*,
    gpiote::*,
};

/// Common functionality related to how to handle sensor input. 
pub trait SensorHandler {
    /// get periodic input samples 
    fn get_periodic_input();
}

/// This would listen for programable switch event, 
/// and start calibration. 
pub trait Calibration {

}

/// Core functionality related to the Canbus that is not 
/// driver specific. 
pub trait Canbus {

}

/// This represent the Controller, that takes force as input, 
/// and send torque output to motor as output.
///
/// Where: u(k) represent a sample input signal. 
pub struct Controller{
}


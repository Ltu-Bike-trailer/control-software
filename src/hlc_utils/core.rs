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

}

/// This would listen for programable switch event, 
/// and start calibration. 
pub trait Calibration {

}

/// Core functionality related to the Canbus that is not 
/// driver specific. 
pub trait Canbus {

}

/// This is a time scheduler, for when to send 
/// control response back on the CAN bus line. 
pub struct ControlScheduler{}

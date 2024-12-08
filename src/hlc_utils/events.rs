//! Defines external events for the "High-Level-Controller" (HLC).
#![deny(warnings, missing_docs)]
#![allow(unused_imports)]

use lib::protocol::*;

#[derive(Clone, Debug)]
/// Possible events, that the HLC, should react to. 
pub enum HlcEvents {
    /// Calibration event.
    Calibration,
    /// Sensor (input) specific event.
    SensorInput(MessageType),

    
}

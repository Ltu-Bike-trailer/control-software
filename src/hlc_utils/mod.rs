//! Defines common utils related to the "High-Level-Controller.
//! Also defines board specific configuration such as PIN
//! setup and GPIOTE events.

#![deny(warnings, missing_docs, unused_imports)]

pub mod config;
pub mod core;
pub mod events;
/// all the constants/functions used for the s-type loadcell
pub mod stype_calibration;

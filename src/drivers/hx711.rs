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

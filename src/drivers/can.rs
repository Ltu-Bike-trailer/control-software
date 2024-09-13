#![no_main]
#![no_std]
#![allow(unused)]

// pick a panicking behavior

use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;

pub struct CanDriver<SPI>{
    spi : SPI
}

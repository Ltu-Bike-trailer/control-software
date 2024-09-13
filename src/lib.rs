//! Defines some panic behavior and common utilities for the application.
//!
//! ## [Wrapper](wrapper)
//!
//! This module mainly exports a few helper types for unit conversion such as
//! [degrees](wrapper::Degrees).

#![no_main]
#![no_std]
#![deny(warnings)]
#![feature(async_fn_traits)]
#![feature(async_closure)]
//#![deny(missing_docs)]
#![deny(clippy::all)]
#![allow(clippy::manual_range_contains)]
use core::sync::atomic::{AtomicUsize, Ordering};

use defmt_rtt as _;
use panic_probe as _;

pub mod gain_scheduled;
pub mod pid;
pub mod wrapper;
pub mod drivers;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is
// invoked
#[defmt::panic_handler]
fn panic() -> ! {
    cortex_m::asm::udf()
}

static COUNT: AtomicUsize = AtomicUsize::new(0);
defmt::timestamp!("{=usize}", {
    // NOTE(no-CAS) `timestamps` runs with interrupts disabled
    let n = COUNT.load(Ordering::Relaxed);
    COUNT.store(n + 1, Ordering::Relaxed);
    n
});

/// Terminates the application and makes `probe-rs` exit with exit-code = 0
pub fn exit() -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[derive(Clone)]
pub struct OwnedItterator<Item: Sized + Clone, const SIZE: usize> {
    buff: [Item; SIZE],
    ptr: usize,
}

#[allow(dead_code)]
impl<Item: Sized + Clone, const SIZE: usize> OwnedItterator<Item, SIZE> {
    fn new(buff: [Item; SIZE]) -> Self {
        Self { buff, ptr: 0 }
    }
}

impl<Item: Sized + Clone, const SIZE: usize> Iterator for OwnedItterator<Item, SIZE> {
    type Item = Item;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr >= SIZE {
            return None;
        }
        let item = self.buff[self.ptr].clone();
        self.ptr += 1;
        Some(item)
    }
}

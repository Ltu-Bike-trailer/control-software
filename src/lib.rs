//! Defines some panic behavior and common utilities for the application.
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![no_main]
#![no_std]
#![feature(async_fn_traits)]
#![feature(async_closure)]
#![deny(
    warnings,
    missing_docs,
    clippy::all,
    clippy::pedantic,
    clippy::nursery,
    rustdoc::all,
    rust_2018_idioms,
    rust_2024_compatibility
)]
#![allow(clippy::manual_range_contains, clippy::inline_always)]

use core::{
    //panic::PanicInfo,
    sync::atomic::{AtomicUsize, Ordering},
};
#[cfg(feature = "esc")]
pub mod bldc;
pub mod cart;
//pub mod svm;

use defmt_rtt as _;

use panic_probe as _;


// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is
// invoked
#[allow(dead_code)]
#[inline(never)]
extern "C" fn _defmt_panic() -> ! {
    cortex_m::asm::udf()
}

    /* 
#[panic_handler]
#[allow(elided_lifetimes_in_paths)]
#[unsafe(no_mangle)]
#[inline(never)]
fn panic(_info: &PanicInfo) -> ! {
    loop {}
}
    */

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
/// A simple iterator of fixed size.
pub struct OwnedItterator<Item: Sized + Clone, const SIZE: usize> {
    buff: [Item; SIZE],
    ptr: usize,
}

#[allow(dead_code)]
impl<Item: Sized + Clone, const SIZE: usize> OwnedItterator<Item, SIZE> {
    const fn new(buff: [Item; SIZE]) -> Self {
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

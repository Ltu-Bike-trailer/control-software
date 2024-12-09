//! Defines some panic behavior and common utilities for the application.

#![no_main]
#![no_std]
#![feature(async_fn_traits, generic_const_exprs, async_closure)]
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
#![allow(
    clippy::manual_range_contains,
    clippy::single_match_else,
    clippy::inline_always,
    incomplete_features
)]
use core::sync::atomic::{AtomicUsize, Ordering};
#[cfg(feature = "esc")]
pub mod bldc;
pub mod cart;

use defmt_rtt as _;
use panic_probe as _;
pub mod boards;
pub mod drivers;

// same panicking *behavior* as `panic-probe` but doesn't print a panic message
// this prevents the panic message being printed *twice* when `defmt::panic` is
// invoked
#[allow(dead_code)]
#[inline(never)]
extern "C" fn _defmt_panic() -> ! {
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

/// Provides a simple ring buffer. This is used to set reference signals while
/// testing the cart.
pub struct RingBuffer<T: Sized + Clone + Default, const N: usize> {
    data: [T; N],
    ptr: usize,
}


impl<T: Sized + Clone + Default, const N: usize> RingBuffer<T, N> {
    /// Creates a new ring buffer using the `buffer` provided.
    ///
    /// ## Panics
    ///
    /// This function panics if the buffer is empty.
    #[must_use]
    #[inline(always)]
    pub const fn new(buffer: [T; N]) -> Self {
        assert!(N != 0);
        Self {
            data: buffer,
            ptr: 0,
        }
    }

    /// Borrows the underlying data.
    pub const fn borrow_data<'a>(&'a self) -> &'a [T] {
        &self.data
    }

    /// Assigns to the next element in the queue.
    ///
    /// ## Panics
    ///
    /// This panics if the buffer len is 0.
    pub fn assign_next(&mut self, new_value: T) {
        match self.data.get_mut(self.ptr) {
            Some(value) => {
                self.ptr += 1;
                *value = new_value;
            }
            _ => {
                self.ptr = 0;
                self.data[0] = new_value;
            }
        }
    }
}

impl<T: Sized + Clone + Copy + Default, const N: usize> Iterator for RingBuffer<T, N> {
    type Item = T;

    #[allow(clippy::single_match_else)]
    fn next(&mut self) -> Option<Self::Item> {
        match self.data.get(self.ptr) {
            Some(value) => {
                self.ptr += 1;
                Some(*value)
            }
            _ => {
                self.ptr = 0;
                Some(self.data[0])
            }
        }
    }
}

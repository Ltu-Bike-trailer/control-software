//! Defines our shared libraries.
//!
//! ## [`Protocol`](protocol)
//!
//! Defines our can protocol and message enumerations for trivial parsing.
//!
//! ## [`PID`](pid)
//!
//! Defines a simple PID controller.
//!
//! ## [`Gain scheduled`](gain_scheduled)
//!
//! Defines a PID controller that follows a parameter scheduled defined at
//! compiletime.
//!
//! ## [`Wrapper`](wrapper)
//!
//! Defines some numerical wrappers for easier computations.

#![deny(
    clippy::all,
    clippy::pedantic,
    clippy::nursery,
    warnings,
    missing_docs,
    rustdoc::all,
    rust_2018_idioms,
    keyword_idents_2024
)]
#![cfg_attr(not(feature = "std"), no_std)]
pub mod constants;
pub mod gain_scheduled;
pub mod pid;
pub mod protocol;
pub mod wrapper;

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
#![allow(
    clippy::option_if_let_else,
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::match_bool,
    // I see the benefits of this but it is not worth it for our usecase.
    clippy::from_over_into
)]
#![cfg_attr(not(feature = "std"), no_std)]
pub mod constants;
pub mod gain_scheduled;
pub mod pid;
pub mod protocol;
pub mod wrapper;

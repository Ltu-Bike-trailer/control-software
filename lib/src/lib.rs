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
//! compile time.
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
    // I see the benefits of this but it is not worth it for our use case.
    clippy::from_over_into,
    unstable_features,
    incomplete_features,
    clippy::inline_always
)]
#![feature(
    f16,
    generic_const_exprs,
    const_trait_impl,
    generic_associated_types_extended,
    generic_const_items,
    generic_arg_infer,
    adt_const_params
)]
#![cfg_attr(not(feature = "std"), no_std, allow(internal_features))]
pub mod constants;
// Remove these for now. They are unused and will likely continue to be unused.
pub mod gain_scheduled;
pub mod pid;
pub mod protocol;
pub mod wrapper;

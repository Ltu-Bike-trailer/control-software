//! Defines core functionality and traits for the module,
//! related to the "High-Level-Controller.
#![deny(warnings, missing_docs)]
#![allow(unused_imports, dead_code)]
#![allow(clippy::option_if_let_else)]
#![allow(clippy::type_complexity)]
#![allow(clippy::pedantic)]

use core::f32;

use nrf52840_hal::{gpio::*, gpiote::*};

/// Common functionality related to how to handle sensor input.
pub trait SensorHandler {
    /// get periodic input samples
    fn get_periodic_input();
}

/// This would listen for programmable switch event,
/// and start calibration.
pub trait Calibration {}

/// Core functionality related to the Canbus that is not
/// driver specific.
pub trait Canbus {}

enum Constants {
    A1,
    A2,
    A3,
    B1,
    B2,
    B3,
}

/// This represent the Controller, that takes force as input,
/// and send moment output to motor as output torque.
///
/// The equation for the discrete controller, takes three
/// past/lagging values for the input and output.
/// As (k-1), (k-2), and (k-3), for the corresponding output
/// and input together with their associated `Constants`.
pub struct Controller {
    t_output: [f32; 3],
    e_input: [f32; 3],
    t_prior: usize,
    e_prior: usize,
}

impl Default for Controller {
    fn default() -> Self {
        Self::new()
    }
}

impl Controller {
    /// controller output constants.
    const A: (f32, f32, f32) = (1.271, 0.0289, 0.01832);
    /// Input constants for controller.
    const B: (f32, f32, f32) = (0.3248, 0.4279, 0.1309);

    #[inline(always)]
    #[must_use]
    /// Creates a new controller instance with lagged values
    /// for output and input (memory).
    pub const fn new() -> Self {
        Self {
            t_output: [0f32; 3],
            e_input: [0f32; 3],
            t_prior: 0,
            e_prior: 0,
        }
    }

    #[inline(always)]
    /// Equation:
    /// T(k) =
    /// 1.271*T(k-1)-0.0289*T(k-2)+0.01832*T(k-3)+0.3248*e(k-1)-0.4279*e(k-2)+0.
    /// 1309*e(k-3)
    ///
    /// Where T(k) is output from controller as moment.
    /// While e(k) is the force insignal to the controller (loadcells).
    pub fn actuate(&mut self, error: f32) -> Option<f32> {
        // self.get_err::<3> is the newest value, and oldest value is self.get_err::<2>.
        let ek: (f32, f32, f32) = (
            self.get_err::<3>(),
            self.get_err::<1>(),
            self.get_err::<2>(),
        );
        let tk: (f32, f32, f32) = (
            self.get_moment::<3>(),
            self.get_moment::<1>(),
            self.get_moment::<2>(),
        );

        let not_ready_e = self.e_input.into_iter().any(|val| val == 0f32);
        let not_ready_t = self.t_output.into_iter().any(|val| val == 0f32);

        let actuate = Self::A.0 * tk.0 - Self::A.1 * tk.1 + Self::A.2 * tk.2 + Self::B.0 * ek.0
            - Self::B.1 * ek.1
            + Self::B.2 * ek.2;

        self.write_moment(actuate);
        self.write_err(error);

        if not_ready_e && not_ready_t {
            // Do nothing wait for all data to be present.
            None
        } else {
            Some(actuate)
        }
    }

    #[inline(always)]
    const fn get_err<const N: usize>(&self) -> f32 {
        let ptr = match self.e_prior.checked_sub(N) {
            Some(val) => val,
            None => {
                //3 - (N - self.e_prior)
                self.e_input.len() - (N - self.e_prior)
                // 2, 1, 0, then wrap to 0 ... idx
            }
        };
        self.e_input[ptr]
    }

    #[inline(always)]
    fn write_err(&mut self, err: f32) {
        let err = err.clamp(-100., 100.);
        self.e_prior += 1;
        if self.e_prior >= 3 {
            self.e_prior = 0;
        }
        self.e_input[self.e_prior] = err;
    }

    #[inline(always)]
    fn write_moment(&mut self, torque: f32) {
        let moment = torque.clamp(-2.0, 2.0);
        self.t_prior += 1;
        if self.t_prior >= 3 {
            self.t_prior = 0;
        }
        self.t_output[self.t_prior] = moment;
    }

    #[inline(always)]
    const fn get_moment<const N: usize>(&self) -> f32 {
        let ptr = match self.t_prior.checked_sub(N) {
            Some(val) => val,
            None => self.t_output.len() - (N - self.t_prior),
        };
        self.t_output[ptr]
    }
}

impl From<Constants> for f32 {
    fn from(val: Constants) -> Self {
        match val {
            Constants::A1 => 1.271,
            Constants::A2 => 0.0289,
            Constants::A3 => 0.01832,
            Constants::B1 => 0.3248,
            Constants::B2 => 0.4279,
            Constants::B3 => 0.1309,
        }
    }
}

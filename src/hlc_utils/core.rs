//! Defines core functionality and traits for the module,
//! related to the "High-Level-Controller.
#![deny(warnings, missing_docs)]
#![allow(unused_imports, dead_code)]

use core::f32;

use nrf52840_hal::{gpio::*, gpiote::*};

/// Common functionality related to how to handle sensor input. 
pub trait SensorHandler {
    /// get periodic input samples 
    fn get_periodic_input();
}

/// This would listen for programable switch event,
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
/// and send torque output to motor as output.
///
/// The equation for the discrete controller, takes three 
/// past/lagging values for the input and output.
/// As (k-1), (k-2), and (k-3), for the corresponding output 
/// and input togheter with their associated `Constants`.
pub struct Controller{
    t_output: [f32; 3],
    e_input:  [f32; 3],
    t_prior: usize, 
    e_prior: usize,
}


impl Controller {
    #[inline(always)]
    /// Creates a new controller instance with lagged values 
    /// for output and input (memory).
    pub const fn new() -> Self {
        Controller {
            t_output: [0f32; 3],
            e_input: [0f32; 3],
            t_prior: 0,
            e_prior: 0, 
        }
    }
    #[inline(always)]
    ///
    pub fn actuate(&mut self, output_torque: f32, error: f32) -> f32{
        let ek1 = self.get_err::<1>();
        
        // Do things
        //
        self.write_torque(output_torque);
        self.write_err(error);
        unimplemented!();
    }

    #[inline(always)]
    const fn get_err<const N:usize>(&self) -> f32 {
        let ptr = match self.e_prior.checked_sub(N) {
            Some(val) => {val},
            None => {
                // 2, 1, 0, then wrap to 0 ... idx
                3-(N-self.e_prior)
            },
        };
        self.e_input[ptr]
    }

    #[inline(always)]
    fn write_err(&mut self, err: f32) {
        self.e_prior += 1;
        if self.e_prior >=3 {
            self.e_prior = 0;
        }
        self.e_input[self.e_prior] = err;
    }

    #[inline(always)] 
    fn write_torque(&mut self, torque: f32) {
        self.t_prior += 1; 
        if self.t_prior >=3 {
            self.t_prior = 0;
        }
        self.t_output[self.t_prior] = torque;
    }

    
    #[inline(always)] 
    fn get_torque<const N: usize>(&self) -> f32 {
        let ptr = match self.t_prior.checked_sub(N) {
            Some(val) => {val},
            None => {
                3-(N-self.t_prior)
            },
        };
        self.t_output[ptr]
    }
    
} 

impl Into<f32> for Constants {
    fn into(self) -> f32 {
        match self {
            Self::A1 => 1.271,
            Self::A2 => 0.0289,
            Self::A3 => 0.01832,
            Self::B1 => 0.3248,
            Self::B2 => 0.4279,
            Self::B3 => 0.1309,
        }
    }
}


//! Defines space vector modulation for our carts motor.

use core::ops::Mul;

use nrf52840_hal::spi::Frequency;
//use rtic_monotonics::fugit::Duration;

#[allow(dead_code)]
/// Provides utilities for space vector modulation.
pub struct SVM {
    state: (f32, f32, f32),
    velocity: f32,
    pwm_freq: f32,
}

impl SVM {
    /// Constructs a new [`SVM`] controller.
    pub const fn new(pwm_freq: nrf52840_hal::spi::Frequency) -> Self {
        Self {
            state: (0., 0., 0.),
            velocity: 0.,
            pwm_freq: match pwm_freq {
                Frequency::K125 => 125_000.,
                Frequency::K250 => 250_000.,
                Frequency::K500 => 500_000.,
                Frequency::M1 => 1_000_000.,
                Frequency::M2 => 2_000_000.,
                Frequency::M4 => 4_000_000.,
                Frequency::M8 => 8_000_000.,
            },
        }
    }

    /// Sets the target velocity for the controller.
    #[inline(always)]
    pub fn set_velocity(&mut self, velocity: f32) {
        self.velocity = velocity;
    }
}
/*
impl<'a> Iterator for &'a SVM {
    type Item = ([f32; 3], u64);

    /// Gets the next pwm signals to apply.
    ///
    /// This function also returns the time to delay, until setting the next
    /// signal.
    fn next(&mut self) -> Option<Self::Item> {

    }
}
*/

pub struct ColumnVector<const N: usize> {
    data: [f32; N],
}

pub struct RowVector<const N: usize> {
    data: [f32; N],
}

pub struct RowMatrix<const M: usize, const N: usize> {
    data: [[f32; M]; N],
}

pub struct ColumnMatrix<const M: usize, const N: usize> {
    data: [[f32; N]; M],
}



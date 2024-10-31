//! Defines the switching pattern for a bldc motor.

use nrf52840_hal::pwm::{Instance, Pwm};

/// Duty cycle as a fraction of the maximum.
pub trait FloatDuty {
    /// Sets the duty cycle as a percentage of the maximum duty cycle.
    fn set_duty(&self, duty_cycle: f32);
}

impl<T: Instance> FloatDuty for Pwm<T> {
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    fn set_duty(&self, duty_cycle: f32) {
        debug_assert!(duty_cycle <= 1., "Duty cycle has to be a percentage");
        debug_assert!(
            duty_cycle >= 0.,
            "Duty cycle has to be a positive percentage"
        );
        let duty_cycle = 1. - duty_cycle;
        self.set_duty_on_common((f32::from(self.max_duty()) * duty_cycle) as u16);
    }
}

/// Denotes the pattern in which we drive the BLDC motors on the ESC.
#[derive(Default)]
pub struct DrivePattern {
    pattern: [u8; 8],
    idx: usize,
}

impl DrivePattern {
    /// Creates a new switching pattern.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            idx: 0,
            pattern: [
                0b00_00_00, // Initally assume that we are in 100 state. This allows initial
                // driving of the motor. it might cost a litle extra current but this should be
                // fine.

                // Matlab version :/
                /*
                    0b00_10_01, // 0b001
                    0b10_01_00, // 0b010
                    0b10_00_01, // 0b011
                    0b01_00_10, // 0b100
                    0b01_10_00, // 0b101
                    0b00_01_10, // 0b110
                */
                // micro chip version
                /*
                0b01_00_10, // 0b001
                0b00_10_01, // 0b010
                0b01_10_00, // 0b011
                0b10_01_00, // 0b100
                0b00_01_10, // 0b101
                0b10_00_01, // 0b110
                */
                // micro chip version ccw
                0b10_00_01, 0b00_01_10, 0b10_01_00, 0b01_10_00, 0b00_10_01, 0b01_00_10,
                // Needed because the hal effects are funky
                0b01_00_10,
            ],
        }
    }

    #[inline(always)]
    /// Sets the bit that indicates that the first phase is high.
    pub fn set_a(&mut self) {
        self.idx |= 0b1;
    }

    #[inline(always)]
    /// Clears the bit that indicates that the first phase is high.
    pub fn clear_a(&mut self) {
        self.idx &= 0b110;
    }

    #[inline(always)]
    /// Sets the bit that indicates that the second phase is high.
    pub fn set_b(&mut self) {
        self.idx |= 0b10;
    }

    #[inline(always)]
    /// Clears the bit that indicates that the second phase is high.
    pub fn clear_b(&mut self) {
        self.idx &= 0b101;
    }

    #[inline(always)]
    /// Sets the bit that indicates that the first phase is high.
    pub fn set_c(&mut self) {
        self.idx |= 0b100;
    }

    #[inline(always)]
    /// Clears the bit that indicates that the first phase is high.
    pub fn clear_c(&mut self) {
        self.idx &= 0b011;
    }

    #[inline(always)]
    /// Forces the motor to advance to the next step in commutation.
    pub fn next(&mut self) {
        self.idx = match self.idx {
            0b000 => 0b000,
            0b100 => 0b101,
            0b101 => 0b001,
            0b001 => 0b011,
            0b011 => 0b010,
            0b010 => 0b110,
            0b110 => 0b100,
            _ => unreachable!(),
        }
    }

    #[inline(always)]
    /// Forces the motor to advance to the next step in commutation.
    pub fn previous(&mut self) {
        self.idx = match self.idx {
            0b000 => 0b000,
            0b101 => 0b100,
            0b001 => 0b101,
            0b011 => 0b001,
            0b010 => 0b011,
            0b110 => 0b010,
            0b100 => 0b110,
            _ => unreachable!(),
        }
    }

    /// Returns the current switching pattern.
    pub fn get(&self) -> ((bool, bool), (bool, bool), (bool, bool)) {
        //debug_assert!(self.idx <= 0b110);
        let pattern = self.pattern[self.idx];
        (
            (pattern & 0b100000 != 0, pattern & 0b10000 != 0),
            (pattern & 0b1000 != 0, pattern & 0b100 != 0),
            (pattern & 0b10 != 0, pattern & 0b1 != 0),
        )
    }

    /// Returns the latest state as a u8.
    pub fn get_pattern_u8(&self) -> u8 {
        self.pattern[self.idx]
    }

    #[allow(clippy::cast_possible_truncation)]
    /// Returns the current state.
    pub fn get_state(&self) -> u8 {
        self.idx as u8
    }
}

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
    idx: usize,
}

/// Drives the motor clock wise.
const CW_PATTERN: [u8; 8] = [
    0b00_00_00, 0b10_00_01, // 0b001
    0b00_01_10, // 0b010
    0b10_01_00, // 0b011
    0b01_10_00, // 0b100
    0b00_10_01, // 0b101
    0b01_00_10, // 0b110
    0b01_00_10, // If out of bounds for some reason, final state.
];

/// Drives the motor counter clock wise.
const CCW_PATTERN: [u8; 8] = [
    0b00_00_00, 0b00_10_01, // 0b001
    0b10_01_00, // 0b010
    0b10_00_01, // 0b011
    0b01_00_10, // 0b100
    0b01_10_00, // 0b101
    0b00_01_10, // 0b110
    0b00_01_10, // If out of bounds for some reason, final state.
];

impl DrivePattern {
    /// Creates a new switching pattern.
    #[must_use]
    pub const fn new() -> Self {
        Self { idx: 0 }
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
    #[must_use]
    #[inline(always)]
    pub const fn get(&self) -> Pattern {
        //debug_assert!(self.idx <= 0b110);
        Pattern(CW_PATTERN[self.idx], CCW_PATTERN[self.idx])
    }

    /// Returns the latest state as a u8.
    #[must_use]
    #[inline(always)]
    pub fn get_pattern_u8(&self) -> u8 {
        unsafe { *CCW_PATTERN.get_unchecked(self.idx) }
    }

    #[allow(clippy::cast_possible_truncation)]
    #[must_use]
    #[inline(always)]
    /// Returns the current state.
    pub const fn get_state(&self) -> u8 {
        self.idx as u8
    }
}

/// A simple named tuple that allows the user to remuneratively break
/// if the duty cycle is less than 0.
#[derive(Clone, Copy, Default)]
#[allow(dead_code)]
pub struct Pattern(u8, u8);

impl PartialEq for Pattern {
    fn eq(&self, other: &Self) -> bool {
        self.0 == other.0
    }
}

impl Pattern {
    #[inline(always)]
    #[must_use]
    /// A simple named tuple that allows the user to change directions.
    pub fn get(self, duty: f32) -> ((bool, bool), (bool, bool), (bool, bool)) {
        if duty >= 0. {
            return Self::conv(self.0);
        }
        // Drive the low side mosfets if we want to break.
        ((false, true), (false, true), (false, true))
    }

    /// Returns a u8 representation of the pattern.
    #[inline(never)]
    #[unsafe(no_mangle)]
    pub fn get_u8(self, duty: f32) -> u8 {
        if duty >= 0. {
            return self.0;
        }
        0b010101
    }

    #[must_use]
    /// Converts the pattern in to a set of bools to simplify the switching
    /// logic.
    const fn conv(pattern: u8) -> ((bool, bool), (bool, bool), (bool, bool)) {
        (
            (pattern & 0b10_0000 != 0, pattern & 0b1_0000 != 0),
            (pattern & 0b1000 != 0, pattern & 0b100 != 0),
            (pattern & 0b10 != 0, pattern & 0b1 != 0),
        )
    }
}

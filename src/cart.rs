//! Defines some overall specifics for our cart.

/// Defines some constants for the cart.
pub mod constants {

    /// The angle in between each of the HAL effect sensors.
    pub const PHASE_SHIFT_HAL: f32 = core::f32::consts::PI / 3.;

    /// The diameter of the wheel.
    pub const WHEEL_DIAMETER: f32 = 0.55;

    /// Sample current every millisecond.
    pub const CURRENT_SAMPLE_RATE: u64 = 15_000;

    /// The maximum pwm duty cycle.
    pub const PWM_MAX: i32 = 32767;

    /// The Proportional gain for the motor.
    pub const MOTOR_KP: i32 = 1000;

    /// The Integral gain for the motor.
    pub const MOTOR_KI: i32 = 250;

    /// The derivative gain for the motor.
    pub const MOTOR_KD: i32 = 5;

    /// Motor sample time in the [`MOTOR_TIMESCALE`] time-frame.
    pub const MOTOR_TS: u32 = 1_000;

    /// Motor time resolution
    pub const MOTOR_TIMESCALE: i32 = 1_000_000;

    /// The decimal point in the controller.
    ///
    /// So each KP,KI,KD is divided by this.
    pub const MOTOR_FIXED_POINT: u32 = 3;
}

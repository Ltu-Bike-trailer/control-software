//! Defines some overall specifics for our cart.

/// Defines some constants for the cart.
pub mod constants {

    /// The angle in between each of the HAL effect sensors.
    pub const PHASE_SHIFT_HAL: f32 = core::f32::consts::PI / 3.;

    /// The diameter of the wheel.
    pub const WHEEL_DIAMETER: f32 = 0.55;

    /// Sample current every millisecond.
    pub const CURRENT_SAMPLE_RATE: u64 = 1_000;

    /// The maximum pwm duty cycle.
    pub const PWM_MAX: i32 = 32767;

    /// The Proportional gain for the motor.
    pub const KP: f32 = 200.;

    /// The Integral gain for the motor.
    pub const KI: f32 = 300.;

    /// The derivative gain for the motor.
    pub const KD: f32 = 0.1; //-0.5;

    /// Motor sample time in the [`MOTOR_TIMESCALE`] time-frame.
    pub const MOTOR_TS: u32 = 5_000;

    /// Motor time resolution
    pub const MOTOR_TIMESCALE: i32 = 1_000_000;

    /// The decimal point in the controller.
    ///
    /// So each KP,KI,KD is divided by this.
    pub const MOTOR_FIXED_POINT: u32 = 3;

    /// Create a [`MOTOR_TS`] duration in actual time.
    pub const DURATION: rtic_monotonics::fugit::Duration<u64, 1, 16_000_000> =
        rtic_monotonics::fugit::Duration::<u64, 1, { MOTOR_TIMESCALE as u32 }>::from_ticks(
            MOTOR_TS as u64,
        )
        .convert();

    /// The minimum duty permitted as output form the PID controller.
    pub const MIN_DUTY: f32 = -1.0;

    /// The maximum duty permitted as output form the PID controller.
    pub const MAX_DUTY: f32 = 0.95;

    /// The range of values permitted as output from the PID controller.
    pub const RANGE: f32 = MAX_DUTY - MIN_DUTY;
}

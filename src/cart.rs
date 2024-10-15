//! Defines some overall specifics for our cart.

/// Defines some constants for the cart.
pub mod constants {

    /// The angle inbetween each of the HAL effect sensors.
    pub const PHASE_SHIFT_HAL: f32 = core::f32::consts::PI / 3.;

    /// The diamater of the wheel.
    pub const WHEEL_DIAMETER: f32 = 0.55;

    /// Sample current every millisecond.
    pub const CURRENT_SAMPLE_RATE: u64 = 15_000;
}

/// Defines the controllers used in the cart.
pub mod controllers {
    use lib::pid::PidDynamic;

    /// The Proportional gain for the motor.
    pub const MOTOR_KP: i32 = 10;
    /// The Integral gain for the motor.
    pub const MOTOR_KI: i32 = 10;
    /// The derivative gain for the motor.
    pub const MOTOR_KD: i32 = 10;
    /// The decimal point in the controller.
    ///
    /// So each KP,KI,KD is divided by this.
    pub const MOTOR_FIXED_POINT: u32 = 10;

    /// The motor PID controller.
    ///
    /// This type makes it easier to change the controller without sideeffects.
    pub type MotorPid = PidDynamic<
        (),
        f32,
        f32,
        // We will not buffer any control signals.
        1,
        MOTOR_KP,
        MOTOR_KI,
        MOTOR_KD,
        // We want percentages.
        1,
        0,
        // We will use uS as timescale.
        1_000_000,
        MOTOR_FIXED_POINT,
    >;
}

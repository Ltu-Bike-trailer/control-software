//! Defines some overall specifics for our cart.

/// Defines some constants for the cart.
pub mod constants {

    /// The angle inbetween each of the HAL effect sensors.
    pub const PHASE_SHIFT_HAL: f32 = core::f32::consts::PI / 3.;

    /// The diamater of the wheel.
    pub const WHEEL_DIAMETER: f32 = 0.55;

    /// Sample current every millisecond.
    pub const CURRENT_SAMPLE_RATE: u64 = 15_000;

    /// The maximum pwm duty cycle.
    pub const PWM_MAX: i32 = 32767;
}

/// Defines the controllers used in the cart.
pub mod controllers {
    use lib::pid::Pid;

    /// The Proportional gain for the motor.
    pub const MOTOR_KP: i32 = 1_000;
    /// The Integral gain for the motor.
    pub const MOTOR_KI: i32 = 50;
    /// The derivative gain for the motor.
    pub const MOTOR_KD: i32 = 10;
    /// Motor sample time in the [`MOTOR_TIMESCALE`] time-frame.
    pub const MOTOR_TS: u32 = 100_000;
    /// Motor time resolution
    pub const MOTOR_TIMESCALE: i32 = 1_000_000;
    /// The decimal point in the controller.
    ///
    /// So each KP,KI,KD is divided by this.
    pub const MOTOR_FIXED_POINT: u32 = 2;

    /// The motor PID controller.
    ///
    /// This type makes it easier to change the controller without sideeffects.
    pub type MotorPid = Pid<
        (),
        u16,
        f32,
        MOTOR_KP,
        MOTOR_KI,
        MOTOR_KD,
        { MOTOR_TS as i32 },
        // We want percentages.
        { super::constants::PWM_MAX },
        // This is a bit odd.
        // See https://docs.nordicsemi.com/bundle/ps_nrf52840/page/pwm.html#ariaid-title24
        3,
        // We will use uS as timescale.
        MOTOR_TIMESCALE,
        MOTOR_FIXED_POINT,
    >;
}

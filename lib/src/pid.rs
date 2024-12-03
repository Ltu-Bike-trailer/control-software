//! Defines a PID controller.

#![allow(clippy::module_name_repetitions)]

use core::{
    convert::Infallible,
    fmt::Debug,
    marker::PhantomData,
    ops::{Add, AddAssign, Div, Mul, Sub},
};

use arraydeque::ArrayDeque;

/// Provides a light weight interface for setting the output of a value
pub trait Channel<Error: Debug> {
    /// The value type that the channel accepts.
    type Output: Sized;
    /// Sets the output value for the type.
    ///
    /// # Errors
    ///
    /// Returns an error if there is some underlying issue.
    /// See specific implementations for details.
    fn set(&mut self, value: Self::Output) -> Result<(), Error>;
}

/// Represents that a type can be doubled in size.
///
/// This is trivially true for all of the types it is implemented on.
pub trait DoubleSize {
    /// A type with double the size of self.
    type Ret: Sized;
    /// Potential errors that can occur.
    type Error: Debug;
    /// Doubles the size of self.
    fn double_size(self) -> Self::Ret;

    /// Halves the size of the value.
    ///
    /// # Errors
    ///
    /// Returns error if the value does not fit.
    fn half_size(value: Self::Ret) -> Result<Self, Self::Error>
    where
        Self: Sized;
}

/// Enumerates the error cases for the controller.
#[derive(Debug)]
pub enum ControllerError<Error: Debug> {
    /// User tried to use the controller before assigning a control sequence.
    BufferEmpty,
    /// The value written to the channel caused some error.
    ChannelError(Error),
    /// Value to large.
    ///
    /// This is thrown when a value is to large to fit in half the size.
    ValueToLarge,

    /// Multiplication resulted in overflow
    MultiplicationOverflow,
}

/// This assumes that we have a i32 as data.
pub struct Pid<
    Error: Debug,
    Interface: Channel<Error>,
    Output: Sized,
    const KP: i32,
    const KI: i32,
    const KD: i32,
    const TS: i32,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i32,
    const FIXED_POINT: u32,
> {
    err: PhantomData<Error>,
    out: Interface,
    reference: Output,
    previous: Output,
    // This might need to be changed in to i64 to not cause errors.
    integral: Output,
    measurement: (u32, Output),
    previous_actuation: Output,
}

/// This assumes that we have a i32 as data.
pub struct PidFixedPoint<
    Error: Debug,
    const KP: u64,
    const KI: u64,
    const KD: u64,
    const TS: u64,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i64,
    const FIXED_POINT_NOM: i64,
    const FIXED_POINT_DENOM: i64,
> {
    err: PhantomData<Error>,
    _out: u32,
    reference: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM>,
    previous: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM>,
    // This might need to be changed in to i64 to not cause errors.
    integral: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM>,
    measurement: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM>,
}
/// This assumes that we have a i32 as data.
pub struct PidF32<
    Error: Debug,
    const KP: u64,
    const KI: u64,
    const KD: u64,
    const TS: u64,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i64,
    const FIXED_POINT_NOM: i64,
    const FIXED_POINT_DENOM: i64,
> {
    err: PhantomData<Error>,
    _out: u32,
    reference: f32,
    previous: f32,
    // This might need to be changed in to i64 to not cause errors.
    integral: f32,
    measurement: f32,
}
/// This assumes that we have a i32 as data.
pub struct PidDynamic<
    Error: Debug,
    Interface: Channel<Error>,
    Output: Sized,
    const BUFFER: usize,
    const KP: i32,
    const KI: i32,
    const KD: i32,
    const THRESHOLD_MAX: i32,
    const THRESHOLD_MIN: i32,
    const TIMESCALE: i32,
    const FIXED_POINT: u32,
> {
    err: PhantomData<Error>,
    out: Interface,
    reference: ArrayDeque<Output, BUFFER>,
    previous: Output,
    // This might need to be changed in to i64 to not cause errors.
    integral: Output,
    measurement: (u32, Output),
    previous_actuation: Output,
}

/// Wraps the info about a specific time step in the control sequence.
#[derive(Debug)]
pub struct ControlInfo<Output: Sized> {
    /// The expected value.
    pub reference: Output,
    /// The actual value read from the [`Channel`].
    pub measured: Output,
    /// The actuation applied to the [`Channel`].
    pub actuation: Output,
    /// The contribution from the p term.
    pub p: Output,
    /// The contribution from the i term.
    pub i: Output,
    /// The contribution from the d term.
    pub d: Output,
    /// The output pre-threshold.
    pub pre_threshold: Output,
}

impl<
        Error: Debug,
        const KP: u64,
        const KI: u64,
        const KD: u64,
        const TS: u64,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i64,
        const FIXED_POINT_NOM: i64,
        const FIXED_POINT_DENOM: i64,
    >
    PidF32<
        Error,
        KP,
        KI,
        KD,
        TS,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT_NOM,
        FIXED_POINT_DENOM,
    >
where
    [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM } as usize]:,
    [(); { FIXED_POINT_NOM * FIXED_POINT_DENOM } as usize]:,
    [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM }
        as usize]:,
    [(); { FIXED_POINT_NOM * FIXED_POINT_NOM } as usize]:,
    [(); { FIXED_POINT_DENOM * FIXED_POINT_NOM } as usize]:,
    [(); { 1 * FIXED_POINT_NOM } as usize]:,
    [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM } as usize]:,
    [(); { 1 * FIXED_POINT_DENOM } as usize]:,
    [(); FIXED_POINT_DENOM as usize]:,
    [(); FIXED_POINT_NOM as usize]:,
{
    const KD: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = FixedPoint::new(KD as i64);
    const KI: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = FixedPoint::new(KI as i64);
    const KP: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = FixedPoint::new(KP as i64);
    const THRESHOLD_MAX: FixedPoint<1, 1> = FixedPoint::new(THRESHOLD_MAX as i64);
    const THRESHOLD_MIN: FixedPoint<1, 1> = FixedPoint::new(THRESHOLD_MIN as i64);
    const TS: FixedPoint<1, TIMESCALE> = FixedPoint::<1, TIMESCALE>::new(TS as i64).convert();
    const TWO: FixedPoint<2, 1> = FixedPoint::<2, 1>::new(1).convert();

    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new() -> Self {
        Self {
            err: PhantomData,
            reference: 0.,
            previous: 0.,
            integral: 0.,
            measurement: 0.,
            _out: 0,
        }
    }

    /// Completely erases previous control signal.
    #[inline(always)]
    pub fn follow(&mut self, value: f32) {
        self.reference = value;
    }

    /// Registers the most recent measurement.
    #[inline(always)]
    pub fn register_measurement(&mut self, value: f32) {
        self.measurement = value;
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying [`Channel`] fails or
    /// any numeric conversions fail.
    #[inline(always)]
    pub fn actuate(&mut self) -> Result<f32, ControllerError<Error>> {
        let output = self.compute_output();
        Ok(output)
    }

    /// Computes the latest control output.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying [`Channel`] fails or
    /// any numeric conversions fail.
    #[inline(always)]
    fn compute_output(&mut self) -> f32 {
        // TODO: Remove all of the converts. The entire point of this is to not use
        // that.
        let target = self.reference;

        let actual = self.measurement;

        let error = target - actual;

        let p = error * (Self::KP).into_float();
        // Integral is approximated as a sum of discrete signals.
        let avg = self.previous + error;
        let avg = avg / Self::TWO.into_float();
        let avg = avg * Self::TS.into_float();
        self.integral = self
            .integral
            .add(avg)
            .min(Self::THRESHOLD_MAX.into_float())
            .max(Self::THRESHOLD_MIN.into_float());

        let i: f32 = self.integral * Self::KI.into_float();

        let d: f32 = error - self.previous;
        let d: f32 = d / Self::TS.into_float();
        let d: f32 = d * Self::KD.into_float();

        let output: f32 = p + i + d;
        let output = output.clamp(
            Self::THRESHOLD_MIN.into_float(),
            Self::THRESHOLD_MAX.into_float(),
        );
        output

        /*

        // Compute the rate of change between previous time-step and this time-step.
        let d = kd * time_scale * (error - self.previous) / ts;

        self.previous = error;

        let output = ((p + i + d) / fixed_point)
            .max(threshold_min)
            .min(threshold_max);

        ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        }*/
    }
}

impl<
        Error: Debug,
        const KP: u64,
        const KI: u64,
        const KD: u64,
        const TS: u64,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i64,
        const FIXED_POINT_NOM: i64,
        const FIXED_POINT_DENOM: i64,
    >
    PidFixedPoint<
        Error,
        KP,
        KI,
        KD,
        TS,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT_NOM,
        FIXED_POINT_DENOM,
    >
where
    [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM } as usize]:,
    [(); { FIXED_POINT_NOM * FIXED_POINT_DENOM } as usize]:,
    [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM }
        as usize]:,
    [(); { FIXED_POINT_NOM * FIXED_POINT_NOM } as usize]:,
    [(); { FIXED_POINT_DENOM * FIXED_POINT_NOM } as usize]:,
    [(); { 1 * FIXED_POINT_NOM } as usize]:,
    [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM } as usize]:,
    [(); { 1 * FIXED_POINT_DENOM } as usize]:,
    [(); FIXED_POINT_DENOM as usize]:,
    [(); FIXED_POINT_NOM as usize]:,
{
    const KD: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = FixedPoint::new(KD as i64);
    const KI: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = FixedPoint::new(KI as i64);
    const KP: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = FixedPoint::new(KP as i64);
    const THRESHOLD_MAX: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> =
        FixedPoint::new(THRESHOLD_MAX as i64);
    const THRESHOLD_MIN: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> =
        FixedPoint::new(THRESHOLD_MIN as i64);
    const TS: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> =
        FixedPoint::<1, TIMESCALE>::new(TS as i64).convert();
    const TWO: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> =
        FixedPoint::<2, 1>::new(1).convert();

    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new() -> Self {
        Self {
            err: PhantomData,
            reference: FixedPoint::new(0),
            previous: FixedPoint::new(0),
            integral: FixedPoint::new(0),
            measurement: FixedPoint::new(0),
            _out: 0,
        }
    }

    /// Completely erases previous control signal.
    #[inline(always)]
    pub fn follow(&mut self, value: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM>) {
        self.reference = value;
    }

    /// Registers the most recent measurement.
    #[inline(always)]
    pub fn register_measurement(&mut self, value: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM>) {
        self.measurement = value;
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying [`Channel`] fails or
    /// any numeric conversions fail.
    #[inline(always)]
    pub fn actuate(&mut self) -> Result<f32, ControllerError<Error>> {
        let output = self.compute_output();
        Ok(output)
    }

    /// Computes the latest control output.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying [`Channel`] fails or
    /// any numeric conversions fail.
    #[inline(always)]
    fn compute_output(&mut self) -> f32
    where
        [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM } as usize]:,
        [(); { FIXED_POINT_NOM * FIXED_POINT_DENOM } as usize]:,
        [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM }
            as usize]:,
        [(); { FIXED_POINT_NOM * FIXED_POINT_NOM } as usize]:,
        [(); { FIXED_POINT_DENOM * FIXED_POINT_NOM } as usize]:,
        [(); { 1 * FIXED_POINT_NOM } as usize]:,
        [(); { FIXED_POINT_DENOM * FIXED_POINT_DENOM * FIXED_POINT_DENOM } as usize]:,
        [(); { 1 * FIXED_POINT_DENOM } as usize]:,
        [(); FIXED_POINT_DENOM as usize]:,
        [(); FIXED_POINT_NOM as usize]:,
    {
        // TODO: Remove all of the converts. The entire point of this is to not use
        // that.
        let target = self.reference;

        let actual = self.measurement;

        let error: FixedPoint<1, FIXED_POINT_DENOM> = target.sub(actual).convert();

        let p: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = error.mul(Self::KP).convert();
        // Integral is approximated as a sum of discrete signals.
        let avg: FixedPoint<1, FIXED_POINT_DENOM> = self.previous.add(error).convert();
        let avg: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = avg.div(Self::TWO).convert();
        let avg: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = avg.mul(Self::TS).convert();
        self.integral = self
            .integral
            .add(avg)
            .const_min::<_, _, FIXED_POINT_NOM, FIXED_POINT_DENOM>(&Self::THRESHOLD_MAX)
            .const_max(&Self::THRESHOLD_MIN);

        let i: FixedPoint<1, FIXED_POINT_DENOM> = self.integral.mul(Self::KI).convert();

        let d: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = error.sub(self.previous).convert();
        let d: FixedPoint<FIXED_POINT_NOM, FIXED_POINT_DENOM> = d.div(Self::TS).convert();
        let d: FixedPoint<1, FIXED_POINT_DENOM> = d.mul(Self::KD).convert();

        let output: FixedPoint<1, FIXED_POINT_DENOM> = p.add(i).convert();
        output.add(d).into_float()

        /*

        // Compute the rate of change between previous time-step and this time-step.
        let d = kd * time_scale * (error - self.previous) / ts;

        self.previous = error;

        let output = ((p + i + d) / fixed_point)
            .max(threshold_min)
            .min(threshold_max);

        ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        }*/
    }
}

#[allow(dead_code)]
#[derive(Clone, Copy)]
/// Represents a fixed point number.
///
/// This is intended to be used instead of a float to save mcp lcp instructions.
pub struct FixedPoint<const NOM: i64, const DENOM: i64> {
    val: i64,
}

#[allow(dead_code)]
impl<const NOM: i64, const DENOM: i64> FixedPoint<NOM, DENOM> {
    /// Returns a new instance of the fixed value.
    #[inline(always)]
    pub const fn new(val: i64) -> Self {
        Self { val }
    }

    /// Returns the maximum of self or the other values.
    #[inline(always)]
    pub const fn const_max<
        const ONOM: i64,
        const ODENOM: i64,
        const TNOM: i64,
        const TDENOM: i64,
    >(
        &self,
        other: &FixedPoint<ONOM, ODENOM>,
    ) -> FixedPoint<TNOM, TDENOM> {
        if self.val * (NOM * ODENOM) > other.val * (ONOM * DENOM) {
            return self.convert();
        }
        return other.convert();
    }

    /// Returns the minimum of self or the other value.
    #[inline(always)]
    pub const fn const_min<
        const ONOM: i64,
        const ODENOM: i64,
        const TNOM: i64,
        const TDENOM: i64,
    >(
        &self,
        other: &FixedPoint<ONOM, ODENOM>,
    ) -> FixedPoint<TNOM, TDENOM> {
        if self.val * (NOM * ODENOM) < other.val * (ONOM * DENOM) {
            return self.convert();
        }
        return other.convert();
    }

    #[inline(always)]
    /// Multiplies self with other.
    pub const fn mul<const ONOM: i64, const ODENOM: i64>(
        self,
        other: FixedPoint<ONOM, ODENOM>,
    ) -> FixedPoint<{ NOM * ONOM }, { DENOM * ODENOM }> {
        FixedPoint {
            val: self.val * other.val,
        }
    }

    #[inline(always)]
    /// Divides two numbers.
    pub const fn div<const ONOM: i64, const ODENOM: i64>(
        self,
        other: FixedPoint<ONOM, ODENOM>,
    ) -> FixedPoint<{ NOM * ODENOM }, { DENOM * ONOM }> {
        FixedPoint {
            // This is an issue. Look in to this :)
            val: self.val / other.val,
        }
    }

    #[inline(always)]
    /// Returns the maximum of two values.
    pub const fn max(a: i64, b: i64) -> i64 {
        if a > b {
            return a;
        }
        return b;
    }

    #[inline(always)]
    /// Adds self to other.
    pub const fn add<const ONOM: i64, const ODENOM: i64>(
        self,
        other: FixedPoint<ONOM, ODENOM>,
    ) -> FixedPoint<1, { DENOM * ODENOM }>
    where
        [(); { DENOM * ODENOM } as usize]:,
        [(); { ODENOM * DENOM } as usize]:,
        [(); { DENOM * ODENOM } as usize]:,
    {
        FixedPoint {
            // This is an issue. Look in to this :)

            /*
                   1         1
               afb -   + dec -
                   cf        fc
            */
            val: self.val * NOM * ODENOM + other.val * ONOM * DENOM,
        }
    }

    #[inline(always)]
    /// Subtract other from self.
    pub const fn sub<const ONOM: i64, const ODENOM: i64>(
        self,
        other: FixedPoint<ONOM, ODENOM>,
    ) -> FixedPoint<1, { DENOM * ODENOM }>
    where
        [(); { DENOM * ODENOM } as usize]:,
    {
        FixedPoint {
            // This is an issue. Look in to this :)

            /*
                   1         1
               afb --  - dec --
                   cf        fc
            */
            val: self.val * NOM * ODENOM - other.val * ONOM * DENOM,
        }
    }

    #[inline(always)]
    /// Converts the number in to the destination size.
    ///
    /// This can cause precision.
    pub const fn convert<const TNOM: i64, const TDENOM: i64>(self) -> FixedPoint<TNOM, TDENOM> {
        if TNOM == NOM && DENOM == TDENOM {
            return FixedPoint { val: self.val };
        }

        // a * b/c = d * e/f
        debug_assert!(NOM * TDENOM > TNOM * DENOM);
        let val = self.val * (NOM * TDENOM / (TNOM * DENOM));
        FixedPoint { val }
    }

    #[inline(always)]
    /// Converts the number in to a float.
    pub const fn into_float(self) -> f32 {
        (self.val as f32) * ({ NOM as f32 } / { DENOM as f32 })
    }
}

impl<
        Error: Debug,
        Interface: Channel<Error, Output = Output>,
        Output: Sized,
        const KP: i32,
        const KI: i32,
        const KD: i32,
        const TS: i32,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i32,
        const FIXED_POINT: u32,
    >
    Pid<
        Error,
        Interface,
        Output,
        KP,
        KI,
        KD,
        TS,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT,
    >
where
    Output: Sub<Output, Output = Output>
        + Mul<Output, Output = Output>
        + Div<Output, Output = Output>
        + DoubleSize
        + Default
        + AddAssign<Output>
        + Add<Output, Output = Output>
        + Copy
        + PartialOrd
        + CmpExt,
    Output::Ret: Add<Output::Ret, Output = Output::Ret> + Div<Output::Ret, Output = Output::Ret>,
    i32: Convert<Output>,
{
    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new(channel: Interface) -> Self {
        Self {
            out: channel,
            err: PhantomData,
            reference: Output::default(),
            previous: Output::default(),
            integral: Output::default(),
            measurement: (0, Output::default()),
            previous_actuation: Output::default(),
        }
    }

    /// Completely erases previous control signals.
    pub fn follow(&mut self, values: Output) {
        self.reference = values;
    }

    /// Registers the most recent measurement.
    pub fn register_measurement(&mut self, value: Output, time_step: u32) {
        self.measurement = (time_step, value);
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying [`Channel`] fails or
    /// any numeric conversions fail.
    pub fn actuate(&mut self) -> Result<ControlInfo<Output>, ControllerError<Error>>
    where
        <Output as DoubleSize>::Ret: Debug + Copy,
        Output: Debug + Copy,
    {
        let output = self.compute_output();

        self.out
            .set(output.actuation)
            .map_err(|err| ControllerError::ChannelError(err))?;
        self.previous_actuation = output.actuation;

        Ok(output)
    }

    /// Computes the latest control output.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying [`Channel`] fails or
    /// any numeric conversions fail.
    #[inline(always)]
    fn compute_output(&mut self) -> ControlInfo<Output> {
        let target: Output = self.reference.clone();

        let kp: Output = KP.convert();
        let ki: Output = KI.convert();
        let kd: Output = KD.convert();

        let time_scale: Output = TIMESCALE.convert();
        let ts: Output = TS.convert();

        let threshold_min = THRESHOLD_MIN.convert();
        let threshold_max = THRESHOLD_MAX.convert();

        let fixed_point = { 10i32.pow(FIXED_POINT) }.convert();

        let actual: Output = self.measurement.1;

        let error = target - actual;

        let p = error * kp;

        // Integral is approximated as a sum of discrete signals.
        let avg: Output::Ret = self.previous.double_size() + error.double_size();
        let two: Output = 2.convert();
        let two = two.double_size();
        if let Ok(val) = Output::half_size(avg / two) {
            self.integral += val * ts / time_scale;
        } else {
            self.integral = Output::default();
        }

        self.integral = self.integral.max(threshold_min).min(threshold_max);

        let i = self.integral.mul(ki);

        // Compute the rate of change between previous time-step and this time-step.
        let d = kd * time_scale * (error - self.previous) / ts;

        self.previous = error;

        let output = ((p + i + d) / fixed_point)
            .max(threshold_min)
            .min(threshold_max);

        ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        }
    }
}

impl<
        Error: Debug,
        Interface: Channel<Error, Output = Output>,
        Output: Sized,
        const BUFFER: usize,
        const KP: i32,
        const KI: i32,
        const KD: i32,
        const THRESHOLD_MAX: i32,
        const THRESHOLD_MIN: i32,
        const TIMESCALE: i32,
        const FIXED_POINT: u32,
    >
    PidDynamic<
        Error,
        Interface,
        Output,
        BUFFER,
        KP,
        KI,
        KD,
        THRESHOLD_MAX,
        THRESHOLD_MIN,
        TIMESCALE,
        FIXED_POINT,
    >
where
    Output: Sub<Output, Output = Output>
        + Mul<Output, Output = Output>
        + Div<Output, Output = Output>
        + DoubleSize
        + Default
        + AddAssign<Output>
        + Add<Output, Output = Output>
        + Copy
        + PartialOrd
        + CmpExt,
    Output::Ret: Add<Output::Ret, Output = Output::Ret> + Div<Output::Ret, Output = Output::Ret>,
    i32: Convert<Output>,
    u64: Convert<Output>,
{
    /// Creates a new controller that sets the output on the
    /// [`Interface`](`Channel`) using a PID control strategy.
    pub fn new(channel: Interface) -> Self {
        Self {
            out: channel,
            err: PhantomData,
            reference: ArrayDeque::new(),
            previous: Output::default(),
            integral: Output::default(),
            measurement: (0, Output::default()),
            previous_actuation: Output::default(),
        }
    }

    /// Completely erases previous control signals.
    pub fn follow<I: IntoIterator<Item = Output>>(&mut self, values: I) {
        self.reference.clear();
        self.reference.extend(values);
    }

    /// Extends the reference signal with new values.
    pub fn extend<I: IntoIterator<Item = Output>>(&mut self, values: I) {
        self.reference.extend(values);
    }

    /// Registers the most recent measurement.
    pub fn register_measurement(&mut self, value: Output, time_step: u32) {
        self.measurement = (time_step, value);
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    ///
    /// # Errors
    ///
    /// This errors if any of the numeric conversions fail or the [`Channel`]
    /// fails.
    pub fn actuate_rate_limited(
        &mut self,
        rate_limit: Output,
        ts: u64,
    ) -> Result<ControlInfo<Output>, ControllerError<Error>>
    where
        <Output as DoubleSize>::Ret: Debug + Copy,
        Output: Debug + Copy,
    {
        let mut output_pre_rate_limit = self.compute_output(ts)?;

        let output = match (
            output_pre_rate_limit.actuation > (self.previous_actuation + rate_limit),
            output_pre_rate_limit.actuation < (self.previous_actuation - rate_limit),
        ) {
            (true, _) => self.previous_actuation + rate_limit,
            (_, true) => self.previous_actuation - rate_limit,
            (_, _) => output_pre_rate_limit.actuation,
        };

        self.out
            .set(output)
            .map_err(|err| ControllerError::ChannelError(err))?;
        self.previous_actuation = output;
        output_pre_rate_limit.actuation = output;
        Ok(output_pre_rate_limit)
    }

    /// Computes the control signal using a PID control strategy.
    ///
    /// if successful it returns the expected value and the read value.
    ///
    /// # Errors
    ///
    /// This errors if any of the numeric conversions fail or the [`Channel`]
    /// fails.
    pub fn actuate(&mut self, ts: u64) -> Result<ControlInfo<Output>, ControllerError<Error>>
    where
        <Output as DoubleSize>::Ret: Debug + Copy,
        Output: Debug + Copy,
    {
        let output = self.compute_output(ts)?;

        self.out
            .set(output.actuation)
            .map_err(|err| ControllerError::ChannelError(err))?;
        self.previous_actuation = output.actuation;

        Ok(output)
    }

    /// Computes the desired output for the latest set of measurements and
    /// reference signals.
    ///
    /// # Errors
    ///
    /// If any numeric conversions fail.
    fn compute_output(&mut self, ts: u64) -> Result<ControlInfo<Output>, ControllerError<Error>> {
        let target: Output = match self.reference.pop_front() {
            Some(value) => value,
            None => return Err(ControllerError::BufferEmpty),
        };

        let kp: Output = KP.convert();
        let ki: Output = KI.convert();
        let kd: Output = KD.convert();

        let time_scale: Output = TIMESCALE.convert();
        let ts: Output = ts.convert();

        let threshold_min = THRESHOLD_MIN.convert();
        let threshold_max = THRESHOLD_MAX.convert();

        let fixed_point = { 10i32.pow(FIXED_POINT) }.convert();

        let actual: Output = self.measurement.1;

        let error = target - actual;

        let p = error * kp;

        // Integral is approximated as a sum of discrete signals.
        let avg: Output::Ret = self.previous.double_size() + error.double_size();
        let two: Output = 2.convert();
        let two = two.double_size();
        self.integral +=
            (Output::half_size(avg / two).map_err(|_| ControllerError::ValueToLarge)?) * ts
                / time_scale;

        self.integral = self.integral.max(threshold_min).min(threshold_max);

        let i = self.integral * ki;

        // Compute the rate of change between previous time-step and this time-step.
        let d = kd * time_scale * (error - self.previous) / ts;

        self.previous = error;

        let output = Output::default()
            - ((p + i + d) / fixed_point)
                .max(threshold_min)
                .min(threshold_max);

        Ok(ControlInfo {
            reference: target,
            measured: actual,
            actuation: output,
            p,
            i,
            d,
            pre_threshold: (p + i + d) / fixed_point,
        })
    }
}

#[macro_export]
/// Makes the instantiation of a new [`PID`](Pid) controller a bit more
/// readable.
///
/// Syntax :
///
/// ```rust
///   use lib::pid::prelude::*;
///   // Note that the channel here is probably some output thing, esc, servo etc.
///   let channel = 0f32;
///   let target = [6f32, 6f32];
///   let mut pid = new_pid!(
///     buffer_size:2, // This is optional, leaving it out will generate a buffer of size 1.
///     kp:2,
///     ki:1,
///     kd:1,
///     sample_time:1,
///     threshold_max:100,
///     threshold_min:0,
///     time_scale:1,
///     fixed_point:0,
///     output:channel
///   );
///   pid.follow(target);
///
///   pid.register_measurement(1f32, 0);
///   let actuation = pid.actuate().unwrap();
///   let expected = 2f32 * (actuation.reference - actuation.measured) + 5f32 / 2f32 + 5f32;
///   assert!(actuation.actuation == expected);
///
///   pid.register_measurement(2f32, 1);
///   let actuation = pid.actuate().unwrap();
///   let expected =
///     2f32 * (actuation.reference - actuation.measured) + (5f32 + 4f32) / 2f32 + 2.5f32 - 1f32;
///   // Accumulated sum + average from previous time step to the current time step.
///   assert!(actuation.actuation == expected);
/// ```
#[allow(clippy::module_name_repetitions)]
macro_rules! new_pid {
    (
        buffer_size:
        $buffer:literal,kp:
        $kp:literal,ki:
        $ki:literal,kd:
        $kd:literal,sample_time:
        $ts:literal,threshold_max:
        $max:literal,threshold_min:
        $min:literal,time_scale:
        $time_scale:literal,fixed_point:
        $fixed_point:literal,output:
        $channel:tt
    ) => {{
        let ret: Pid<_, _, _, $buffer, $kp, $ki, $kd, $ts, $max, $min, $time_scale, $fixed_point> =
            Pid::new($channel);
        ret
    }};
    (
        kp:
        $kp:literal,ki:
        $ki:literal,kd:
        $kd:literal,sample_time:
        $ts:literal,threshold_max:
        $max:literal,threshold_min:
        $min:literal,time_scale:
        $time_scale:literal,fixed_point:
        $fixed_point:literal,output:
        $channel:tt
    ) => {{
        let ret: Pid<_, _, _, 1, $kp, $ki, $kd, $ts, $max, $min, $time_scale, $fixed_point> =
            Pid::new($channel);
        ret
    }};
}

/// Re-exports all of the needed parts of a PID controller.
pub mod prelude {

    impl Channel<()> for f32 {
        type Output = Self;

        fn set(&mut self, value: Self) -> Result<(), ()> {
            *self = value;
            Ok(())
        }
    }
    impl Channel<()> for i32 {
        type Output = Self;

        #[inline(always)]
        fn set(&mut self, value: Self::Output) -> Result<(), ()> {
            *self = value;
            Ok(())
        }
    }
    /*impl Channel<()> for u16 {
        type Output = i32;

        #[inline(always)]
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        fn set(&mut self, value: Self::Output) -> Result<(), ()> {
            *self = value as Self;
            Ok(())
        }
    }*/
    impl Channel<()> for u16 {
        type Output = f32;

        #[inline(always)]
        #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
        fn set(&mut self, value: Self::Output) -> Result<(), ()> {
            *self = value as Self;
            Ok(())
        }
    }
    pub use super::{Channel, Pid};
    pub use crate::new_pid;
}

mod sealed {

    /// Converts a value in to the destination type if possible.

    pub trait Convert<Dest: Sized> {
        /// Converts the value in to the destination type.
        fn convert(self) -> Dest;
    }

    impl Convert<Self> for i32 {
        #[inline(always)]
        fn convert(self) -> Self {
            self
        }
    }

    impl Convert<i32> for u64 {
        #[inline(always)]
        fn convert(self) -> i32 {
            self as i32
        }
    }

    impl Convert<f32> for u64 {
        #[inline(always)]
        fn convert(self) -> f32 {
            self as f32
        }
    }

    impl Convert<f32> for i32 {
        #[inline(always)]
        fn convert(self) -> f32 {
            self as f32
        }
    }
    pub trait CmpExt {
        #[inline(always)]
        fn max(self, other: Self) -> Self
        where
            Self: PartialOrd<Self> + Sized,
        {
            match self > other {
                true => self,
                false => other,
            }
        }

        #[inline(always)]
        fn min(self, other: Self) -> Self
        where
            Self: PartialOrd<Self> + Sized,
        {
            match self < other {
                true => self,
                false => other,
            }
        }
    }

    macro_rules! ext {
        ($($ty:ident),*) => {
            $(
                impl CmpExt for $ty {}
            )*
        };
    }
    ext!(i64, i32, i16, i8, u64, u32, u16, u8, f64, f32);
}
use sealed::{CmpExt, Convert};

impl DoubleSize for i32 {
    type Error = ();
    type Ret = i64;

    #[inline(always)]
    fn double_size(self) -> Self::Ret {
        self.into()
    }

    #[inline(always)]
    fn half_size(value: Self::Ret) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        value.try_into().map_err(|_| ())
    }
}

impl DoubleSize for f32 {
    type Error = Infallible;
    type Ret = f64;

    #[inline(always)]
    fn double_size(self) -> Self::Ret {
        self.into()
    }

    #[inline(always)]
    fn half_size(value: Self::Ret) -> Result<Self, Self::Error>
    where
        Self: Sized,
    {
        Ok(value as Self)
    }
}

#[cfg(feature = "std")]
#[cfg(test)]
mod test {
    use super::prelude::*;

    impl<Iter: Iterator<Item = i32>> Channel<()> for (Iter, i32) {
        type Output = i32;

        fn set(&mut self, value: i32) -> Result<(), ()> {
            self.1 = value;
            Ok(())
        }
    }

    impl Channel<()> for i32 {
        type Output = f32;

        fn set(&mut self, value: f32) -> Result<(), ()> {
            *self = value as i32;
            Ok(())
        }
    }

    #[test]
    fn test_p() {
        let channel = ([1, 2, 3, 5, 6].into_iter(), 0);
        let target = [6, 6, 6, 6, 6];
        let mut pid: Pid<_, _, _, 5, 2, 0, 0, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        while let Ok(actuation) = pid.actuate() {
            assert!(actuation.actuation == 2 * (actuation.reference - actuation.measured));
        }
    }

    #[test]
    fn test_pi() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, i32, 2, 2, 1, 0, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(1, 0);
        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 2 * (actuation.reference - actuation.measured) + 5 / 2);

        pid.register_measurement(2, 1);
        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(
            actuation.actuation == 2 * (actuation.reference - actuation.measured) + (5 + 4) / 2 + 2
        );
    }

    #[test]
    fn test_pid() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, i32, 2, 2, 1, 1, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(1, 0);
        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 2 * (actuation.reference - actuation.measured) + 5 / 2 + 5);

        pid.register_measurement(2, 1);
        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(
            actuation.actuation
                == 2 * (actuation.reference - actuation.measured) + (5 + 4) / 2 + 2 - 1
        );
    }

    // #[allow(non_snake_case)]
    // /// A collection of PID parameters.
    // pub struct PidParams {
    //     /// The proportional error coefficient.
    //     pub KP: i32,
    //     /// The integral of error coefficient.
    //     pub KI: i32,
    //     /// The derivative of error coefficient.
    //     pub KD: i32,
    //     // 10 ^ 1
    //     /// log 10 of scale.
    //     pub SCALE: u32,
    //
    //     /// Sample time.
    //     pub TS: i32,
    //     /// Fraction of seconds, so uS => 10^6.
    //     pub TIMESCALE: i32,
    // }
    //
    // /// The PID parameters for the ESC.
    // pub const ESC_PID_PARAMS: PidParams = PidParams {
    //     KP: 23,
    //     KI: 30,
    //     KD: 31,
    //     // 10^2
    //     SCALE: 2,
    //     TS: 50_000,
    //     TIMESCALE: 1_000_000,
    // };
    //
    // /// A wrapper around the [`Pid`] controller with predefined coefficients.
    // pub type MotorController = Pid<
    //     (),
    //     i32,
    //     f32,
    //     100,
    //     { ESC_PID_PARAMS.KP },
    //     { ESC_PID_PARAMS.KI },
    //     { ESC_PID_PARAMS.KD },
    //     { ESC_PID_PARAMS.TS },
    //     100,
    //     -100,
    //     { ESC_PID_PARAMS.TIMESCALE },
    //     { ESC_PID_PARAMS.SCALE },
    // >;
    //
    // #[test]
    // fn test_pid_esc() {
    //     let target = [100f32; 70].into_iter().chain([70f32; 30].into_iter());
    //     let mut pid: MotorController = Pid::new(0i32);
    //     pid.follow(target);
    //
    //     pid.register_measurement(1f32, 0);
    //     let mut counter = 30;
    //     while let Ok(actuation) = pid.actuate() {
    //         counter += 1;
    //         pid.register_measurement(counter as f32, 0);
    //         println!("Actuation : {actuation:?}");
    //     }
    //     // assert!(actuation.actuation == 2 * (actuation.reference -
    // actuation.measured) + 5 / 2 + 5);
    //
    //     pid.register_measurement(2f32, 1);
    //     let actuation = pid.actuate().unwrap();
    //     println!("First actuation : {actuation:?}");
    //     // Accumulated sum + average from previous time step to the current time
    // step.     // assert!(
    //     //     actuation.actuation // == 2 * (actuation.reference -
    // actuation.measured) + (5 + 4) / 2 + 2 - 1     // );
    //     assert!(false);
    // }

    #[test]
    fn test_pid_no_diff() {
        let channel = ([0].into_iter(), 0);
        let target = [0, 0];
        let mut pid: Pid<(), _, i32, 2, 2, 1, 1, 1, 100, 0, 1, 0> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(0, 0);
        let actuation = pid.actuate().unwrap();
        assert!(actuation.actuation == 0);

        pid.register_measurement(0, 1);
        let actuation = pid.actuate().unwrap();
        // Accumulated sum + average from previous time step to the current time step.
        assert!(actuation.actuation == 0);
    }

    #[test]
    fn test_pid_fixed_point() {
        let channel = ([1, 2].into_iter(), 0);
        let target = [6, 6];
        let mut pid: Pid<(), _, i32, 2, 21, 21, 11, 1, 100, 0, 1, 1> = Pid::new(channel);
        pid.follow(target);

        pid.register_measurement(1, 0);
        let actuation = pid.actuate().unwrap();
        let expected =
            (21 * (actuation.reference - actuation.measured) + 21 * (5 / 2) + 11 * 5) / 10;
        println!("Actuation : {:?} == {expected:?}", actuation);
        assert!(actuation.actuation == expected);

        pid.register_measurement(2, 1);
        let actuation = pid.actuate().unwrap();
        let expected =
            (21 * (actuation.reference - actuation.measured) + 21 * ((5 + 4) / 2) + 11 * 2) / 10;
        println!("Actuation : {:?} == {expected:?}", actuation);
        // Accumulated sum + average from previous time step to the current time step.
        assert!(actuation.actuation == expected);
    }

    #[test]
    fn test_pid_float() {
        let channel = 0f32;
        let target = [6f32, 6f32];
        let mut pid = new_pid!(
            buffer_size:2,
            kp:2,
            ki:1,
            kd:1,
            sample_time:1,
            threshold_max:100,
            threshold_min:0,
            time_scale:1,
            fixed_point:0,
            output:channel
        );
        pid.follow(target);

        pid.register_measurement(1f32, 0);
        let actuation = pid.actuate().unwrap();
        let expected = 2f32 * (actuation.reference - actuation.measured) + 5f32 / 2f32 + 5f32;
        println!("Actuation : {:?} == {expected:?}", actuation);
        assert!(actuation.actuation == expected);

        pid.register_measurement(2f32, 1);
        let actuation = pid.actuate().unwrap();
        let expected =
            2f32 * (actuation.reference - actuation.measured) + (5f32 + 4f32) / 2f32 + 2.5f32
                - 1f32;
        println!("Actuation : {:?} == {expected:?}", actuation);
        // Accumulated sum + average from previous time step to the current time step.
        assert!(actuation.actuation == expected);
    }
}

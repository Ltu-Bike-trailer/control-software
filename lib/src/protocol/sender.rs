//! Provides a priority queue based `Channel` abstraction called [`Sender`].
//!
//! For usage examples please refer to the [`Sender`] examples.

use heapless::binary_heap::Max;

use super::{BatteryStatus, CanMessage, MessageType, MotorSubSystem, VelocityInfo};

/// A simple sender wrapper.
///
/// This is abstraction allows the can task to dequeue [`CanMessage`]s from the
/// buffer. And allows any sender to enqueue [`MessageType`]s in the buffer,
/// thus making the abstraction quite simple to use.
///
///
/// ## Short hand example
///
/// ```
/// use embedded_can::Frame;
/// use lib::protocol::{
///     MessageType,
///     sender::Sender,
///     CanMessage
/// };
///
/// let mut buffer = Sender::<10>::new();
///
/// // Note that these will be prioritized in and dequeued in order of significance.
/// buffer.set_status_left_motor(10.).expect("Insufficient buffer size");
/// buffer.set_status_left_motor(10.).expect("Insufficient buffer size");
/// buffer.set_status_battery(35.).expect("Insufficient buffer size");
/// buffer.set_left_motor(10.).expect("Insufficient buffer size");
/// buffer.set_right_motor(11.).expect("Insufficient buffer size");
/// buffer.set_alpha(0.0001).expect("Insufficient buffer size");
/// buffer.set_theta(0.0010).expect("Insufficient buffer size");
/// buffer.set_load_cell_front(10.).expect("Insufficient buffer size");
/// buffer.set_load_cell_bed(9.).expect("Insufficient buffer size");
///
/// let mut n = 0;
/// while let Some(msg) = buffer.dequeue() {
///     println!("Expected {:?}", msg.data());
///     let serialzed: CanMessage = msg;
///     let de: MessageType = MessageType::try_from(&serialzed).unwrap();
///     println!("Expected {:?}", de);
///     n+=1;
/// }
/// assert!(n == 9);
/// ```
///
/// ## Example
///
/// ```
/// use embedded_can::Frame;
/// use lib::protocol::{
///     BatteryStatus,
///     FixedLogType,
///     MessageType,
///     MotorSubSystem,
///     sender::Sender,
///     SensorSubSystem,
///     VelocityInfo,
///     WriteType,CanMessage
/// };
///
/// let to_test = [
///     MessageType::FixedLog(FixedLogType::BatteryStatus(BatteryStatus(129.))),
///     MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
///         129.,
///     )))),
///     MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
///         129.,
///     )))),
///     MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))),
///     MessageType::Write(WriteType::Motor(MotorSubSystem::Right(10.))),
///     MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(10.))),
///     MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(10.))),
///     MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellBed(10.))),
///     MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(10.))),
/// ];
/// let mut buffer = Sender::<10>::new();
/// to_test
///     .iter()
///     .for_each(|el| buffer.enqueue(el.clone()).unwrap());
///
/// while let Some(msg) = buffer.dequeue() {
///     println!("Expected {:?}", msg.data());
///     let serialzed: CanMessage = msg;
///     let de: MessageType = MessageType::try_from(&serialzed).unwrap();
///     println!("Expected {:?}", de);
/// }
/// ```
pub struct Sender<const N: usize> {
    buffer: heapless::BinaryHeap<MessageType, Max, N>,
}

#[derive(Clone, Copy, Debug)]
/// The buffer is full. This should not happen.
pub struct OOM();

impl<const N: usize> Sender<N> {
    #[must_use]
    /// Creates a new sender.
    pub const fn new() -> Self {
        Self {
            buffer: heapless::BinaryHeap::new(),
        }
    }

    /// Enqueues a new message in the buffer.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying buffer is full.
    pub fn enqueue(&mut self, msg: MessageType) -> Result<(), OOM> {
        self.buffer.push(msg).map_err(|_| OOM())
    }

    /// Dequeues  can frame from the buffer ready to send to the driver.
    pub fn dequeue(&mut self) -> Option<CanMessage> {
        Some(self.buffer.pop()?.into())
    }

    /// Enqueues a left motor write message.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_left_motor(&mut self, vel: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::Write(super::WriteType::Motor(
            MotorSubSystem::Left(vel),
        )))
    }

    /// Enqueues a left motor write message.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_right_motor(&mut self, vel: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::Write(super::WriteType::Motor(
            MotorSubSystem::Right(vel),
        )))
    }

    /// Enqueues a write to the alpha sensor value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_alpha(&mut self, value: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::Write(super::WriteType::Sensor(
            super::SensorSubSystem::AlphaSensor(value),
        )))
    }

    /// Enqueues a write to the Theta sensor value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_theta(&mut self, value: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::Write(super::WriteType::Sensor(
            super::SensorSubSystem::ThetaSensor(value),
        )))
    }

    /// Enqueues a write to the Load cell front sensor value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_load_cell_front(&mut self, value: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::Write(super::WriteType::Sensor(
            super::SensorSubSystem::LoadCellFront(value),
        )))
    }

    /// Enqueues a write to the load cell bed sensor value.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_load_cell_bed(&mut self, value: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::Write(super::WriteType::Sensor(
            super::SensorSubSystem::LoadCellBed(value),
        )))
    }

    /// Informs the rest of the system about the status of the left motor.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_status_left_motor(&mut self, vel: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::FixedLog(super::FixedLogType::Velocity(
            VelocityInfo(MotorSubSystem::Left(vel)),
        )))
    }

    /// Informs the rest of the system about the status of the right motor.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_status_right_motor(&mut self, vel: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::FixedLog(super::FixedLogType::Velocity(
            VelocityInfo(MotorSubSystem::Right(vel)),
        )))
    }

    /// Informs the rest of the system about the status of the battery.
    ///
    /// # Errors
    ///
    /// This function returns an error if the underlying message queue is full.
    pub fn set_status_battery(&mut self, voltage: f32) -> Result<(), OOM> {
        self.enqueue(MessageType::FixedLog(super::FixedLogType::BatteryStatus(
            BatteryStatus(voltage),
        )))
    }
}

impl<const N: usize> Default for Sender<N> {
    fn default() -> Self {
        Self::new()
    }
}

//! Defines the protocol used to communicate over can bus.
//!
//! This is a dead simple protocol a typical message is defined by a
//! [`Message identifier`](constants::Message) and a single f32.
pub mod constants;

use constants::{InvalidMessageId, Message::*};
#[allow(clippy::wildcard_imports)]
use embedded_can::{Frame, Id};
use heapless::binary_heap::Max;

/// Denotes all of the supported message types.
#[derive(Clone, Debug)]
pub enum MessageType {
    /// Writes to a subsystem.
    ///
    /// These are reserved for things like sensor readings and motor velocity.
    Write(WriteType),
    /// A fixed log.
    ///
    /// These are reserved for system critical data such as battery voltage and
    /// velocity.
    FixedLog(FixedLogType),
    // Omitted as this is currently handled by the logger.
    //GeneralLog(LogLevel, HashOrString<NameIter>, ArgsIter),
}

/// Denotes all of the write operations possible with the given system.
#[derive(Clone, PartialEq, Debug)]
pub enum WriteType {
    /// The message relates to a sensor subsystem.
    Sensor(SensorSubSystem),
    /// The message relates to motor subsystem.
    Motor(MotorSubSystem),
}

/// Denotes a log for a specific subsystem.
#[derive(Clone, PartialEq, Debug)]
pub enum FixedLogType {
    /// The status of the battery.
    BatteryStatus(BatteryStatus),
    /// The velocity of that given motor subsystem.
    Velocity(VelocityInfo),
}

/// The velocity of the given motor.
#[derive(Clone, PartialEq, Debug)]
pub struct VelocityInfo(
    /// The motor syb system the message is refering to.
    pub MotorSubSystem,
);

/// The battery status at the time of logging.
#[derive(Clone, PartialEq, Debug)]
pub struct BatteryStatus(
    /// The voltage over the battery at the time of logging.
    pub f32,
);

/// Denotes all of our sensor sub systems.
#[derive(Clone, PartialEq, Debug)]
#[non_exhaustive]
pub enum SensorSubSystem {
    /// Denotes the sensor subsystem for our main control input.
    LoadCellFront(f32),
    /// The theta angle.
    ThetaSensor(f32),
    /// The alpha angle.
    AlphaSensor(f32),
    /// The load cell on the bed.
    LoadCellBed(f32),
}

/// Denotes the motors connected to the cart and their target velocities.
#[derive(Clone, PartialEq, Debug)]
pub enum MotorSubSystem {
    /// The target velocity for the left motor.
    Left(f32),
    /// The target velocity for the right motor.
    Right(f32),
}

/// Denotes our different log levels.
#[derive(Clone, PartialEq, Debug)]
pub enum GeneralLogType {}

#[derive(Clone, Debug)]
/// The message could not be propperly parsed.
pub enum ParsingError {
    /// The message id was not valid.
    IncorrectId,
    /// The buffer did not contain sufficient bytes.
    ///
    /// This can typically be resolved by reading more bytes in to the buffer.
    InsufficientBytes,
}

impl From<InvalidMessageId> for ParsingError {
    fn from(_value: InvalidMessageId) -> Self {
        Self::IncorrectId
    }
}

/// Denotes that a message can be used as a can message.
pub trait Message<F: embedded_can::Frame>:
    Into<F> + for<'a> TryFrom<&'a F, Error = ParsingError>
{
}

impl Into<CanMessage> for WriteType {
    fn into(self) -> CanMessage {
        match self {
            Self::Motor(m) => m.into(),
            Self::Sensor(s) => s.into(),
        }
    }
}

impl TryFrom<&CanMessage> for WriteType {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        Ok(match value.id() {
            Id::Standard(id) => match constants::Message::try_from(id)? {
                LeftMotor | RightMotor => WriteType::Motor(MotorSubSystem::try_from(value)?),
                SensorAlpha | SensorTheta | SensorLBed | SensorLFront => {
                    WriteType::Sensor(SensorSubSystem::try_from(value)?)
                }
                _ => return Err(ParsingError::IncorrectId),
            },
            _ => return Err(ParsingError::IncorrectId),
        })
    }
}

impl Message<CanMessage> for WriteType {}

impl Into<CanMessage> for MotorSubSystem {
    fn into(self) -> CanMessage {
        match self {
            Self::Left(val) => {
                CanMessage::new(LeftMotor, &val.to_le_bytes()).expect("Invalid parser")
            }
            Self::Right(val) => {
                CanMessage::new(RightMotor, &val.to_le_bytes()).expect("Invalid parser")
            }
        }
    }
}

impl TryFrom<&CanMessage> for MotorSubSystem {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        let left = match value.id() {
            Id::Standard(id) => match id.try_into()? {
                LeftMotor => true,
                RightMotor => false,
                _ => return Err(ParsingError::IncorrectId),
            },
            _ => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let buff = [
            match data.get(0) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(1) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(2) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(3) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
        ];
        Ok(match left {
            true => Self::Left(f32::from_le_bytes(buff)),
            false => Self::Right(f32::from_le_bytes(buff)),
        })
    }
}

impl Message<CanMessage> for MotorSubSystem {}

impl Into<CanMessage> for SensorSubSystem {
    fn into(self) -> CanMessage {
        match self {
            Self::AlphaSensor(val) => {
                CanMessage::new(SensorAlpha, &val.to_le_bytes()).expect("Invalid parser")
            }
            Self::ThetaSensor(val) => {
                CanMessage::new(SensorTheta, &val.to_le_bytes()).expect("Invalid parser")
            }
            Self::LoadCellFront(val) => {
                CanMessage::new(SensorLFront, &val.to_le_bytes()).expect("Invalid parser")
            }
            Self::LoadCellBed(val) => {
                CanMessage::new(SensorLBed, &val.to_le_bytes()).expect("Invalid parser")
            }
        }
    }
}

impl TryFrom<&CanMessage> for SensorSubSystem {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        let idx = match value.id() {
            Id::Standard(id) => match id.try_into()? {
                SensorAlpha => 0,
                SensorTheta => 1,
                SensorLBed => 2,
                SensorLFront => 3,
                _ => return Err(ParsingError::IncorrectId),
            },
            _ => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let buff = [
            match data.get(0) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(1) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(2) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(3) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
        ];
        Ok(match idx {
            0 => Self::AlphaSensor(f32::from_le_bytes(buff)),
            1 => Self::ThetaSensor(f32::from_le_bytes(buff)),
            2 => Self::LoadCellBed(f32::from_le_bytes(buff)),
            3 => Self::LoadCellFront(f32::from_le_bytes(buff)),
            _ => unimplemented!("Should not be needed."),
        })
    }
}

impl Message<CanMessage> for SensorSubSystem {}

impl Into<CanMessage> for FixedLogType {
    fn into(self) -> CanMessage {
        match self {
            Self::BatteryStatus(BatteryStatus(val)) => {
                CanMessage::new(BatteryDiag, &val.to_le_bytes()).expect("Invalid parser")
            }
            Self::Velocity(VelocityInfo(MotorSubSystem::Left(val))) => {
                CanMessage::new(MotorDiagLeft, &val.to_le_bytes()).expect("Invalid parser")
            }
            Self::Velocity(VelocityInfo(MotorSubSystem::Right(val))) => {
                CanMessage::new(MotorDiagRight, &val.to_le_bytes()).expect("Invalid parser")
            }
        }
    }
}
impl TryFrom<&CanMessage> for FixedLogType {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        let idx = match value.id() {
            Id::Standard(id) => match id.try_into()? {
                MotorDiagLeft => 0,
                MotorDiagRight => 1,
                BatteryDiag => 2,
                _ => return Err(ParsingError::IncorrectId),
            },
            _ => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let buff = [
            match data.get(0) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(1) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(2) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
            match data.get(3) {
                Some(val) => *val,
                None => return Err(ParsingError::InsufficientBytes),
            },
        ];
        Ok(match idx {
            0 => Self::Velocity(VelocityInfo(MotorSubSystem::Left(f32::from_le_bytes(buff)))),
            1 => Self::Velocity(VelocityInfo(MotorSubSystem::Right(f32::from_le_bytes(
                buff,
            )))),
            2 => Self::BatteryStatus(BatteryStatus(f32::from_le_bytes(buff))),
            _ => unimplemented!("Should not be needed."),
        })
    }
}

impl Message<CanMessage> for FixedLogType {}

impl Into<CanMessage> for MessageType {
    fn into(self) -> CanMessage {
        match self {
            Self::FixedLog(f) => f.into(),
            Self::Write(w) => w.into(),
        }
    }
}

impl TryFrom<&CanMessage> for MessageType {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        Ok(match value.id() {
            Id::Standard(id) => match id.try_into()? {
                LeftMotor | RightMotor | SensorAlpha | SensorTheta | SensorLBed | SensorLFront => {
                    Self::Write(WriteType::try_from(value)?)
                }
                MotorDiagLeft | MotorDiagRight | BatteryDiag => {
                    Self::FixedLog(FixedLogType::try_from(value)?)
                }
            },
            _ => return Err(ParsingError::IncorrectId),
        })
    }
}

impl Message<CanMessage> for MessageType {}

/// A simple dummy `CanMessage`.
pub struct CanMessage {
    id: embedded_can::Id,
    dlc: u8,       // Data length code = length of data field.
    data: [u8; 8], // D7 - D0
}

impl Frame for CanMessage {
    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
        let mut buffer = [0; 8];
        for (idx, el) in data.iter().enumerate() {
            debug_assert!(idx < 8);
            buffer[idx] = *el;
        }
        Some(CanMessage {
            id: id.into(),
            dlc: data.len() as u8,
            data: buffer,
        })
    }

    fn new_remote(_id: impl Into<embedded_can::Id>, _dlc: usize) -> Option<Self> {
        None
    }

    fn is_extended(&self) -> bool {
        false
    }

    fn is_remote_frame(&self) -> bool {
        false
    }

    fn id(&self) -> embedded_can::Id {
        self.id
    }

    fn dlc(&self) -> usize {
        self.dlc as usize
    }

    fn data(&self) -> &[u8] {
        &self.data
    }
}

/// A simple sender wrapper.
///
/// This is abstraction allows the can task to dequeue [`CanMessage`]s from the
/// buffer. And allows any sender to enqueue [`MessageType`]s in the buffer,
/// thus making the abstraction quite simple to use.
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
///     Sender,
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
    /// Creates a new sender.
    pub const fn new() -> Self {
        Self {
            buffer: heapless::BinaryHeap::new(),
        }
    }

    /// Enqueues a new message in the buffer.
    pub fn enqueue(&mut self, msg: MessageType) -> Result<(), OOM> {
        self.buffer.push(msg).map_err(|_| OOM())
    }

    /// Dequeues  can frame from the buffer ready to send to the driver.
    pub fn dequeue(&mut self) -> Option<CanMessage> {
        Some(self.buffer.pop()?.into())
    }
}

impl PartialOrd for MessageType {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(match (self, other) {
            (Self::Write(WriteType::Motor(_)), Self::Write(WriteType::Motor(_))) => {
                core::cmp::Ordering::Equal
            }
            (Self::Write(WriteType::Sensor(_)), Self::Write(WriteType::Sensor(_))) => {
                core::cmp::Ordering::Equal
            }
            (Self::FixedLog(_), Self::FixedLog(_)) => core::cmp::Ordering::Equal,
            (Self::Write(WriteType::Motor(_)), _) => core::cmp::Ordering::Greater,
            (Self::Write(WriteType::Sensor(_)), Self::Write(WriteType::Motor(_))) => {
                core::cmp::Ordering::Less
            }
            (Self::Write(WriteType::Sensor(_)), _) => core::cmp::Ordering::Greater,
            (Self::FixedLog(_), _) => core::cmp::Ordering::Less,
        })
    }
}

impl Ord for MessageType {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        match (self, other) {
            (Self::Write(WriteType::Motor(_)), Self::Write(WriteType::Motor(_))) => {
                core::cmp::Ordering::Equal
            }
            (Self::Write(WriteType::Sensor(_)), Self::Write(WriteType::Sensor(_))) => {
                core::cmp::Ordering::Equal
            }
            (Self::FixedLog(_), Self::FixedLog(_)) => core::cmp::Ordering::Equal,
            (Self::Write(WriteType::Motor(_)), _) => core::cmp::Ordering::Greater,
            (Self::Write(WriteType::Sensor(_)), Self::Write(WriteType::Motor(_))) => {
                core::cmp::Ordering::Less
            }
            (Self::Write(WriteType::Sensor(_)), _) => core::cmp::Ordering::Greater,
            (Self::FixedLog(_), _) => core::cmp::Ordering::Less,
        }
    }
}

impl core::cmp::PartialEq for MessageType {
    fn eq(&self, other: &Self) -> bool {
        match (self, other) {
            (Self::Write(l0), Self::Write(r0)) => l0 == r0,
            (Self::FixedLog(l0), Self::FixedLog(r0)) => l0 == r0,
            _ => false,
        }
    }
}

impl core::cmp::Eq for MessageType {}

#[cfg(test)]
mod test {
    use embedded_can::Frame;

    use super::{
        BatteryStatus,
        FixedLogType,
        MessageType,
        MotorSubSystem,
        Sender,
        SensorSubSystem,
        VelocityInfo,
        WriteType,
    };
    use crate::protocol::CanMessage;

    #[test]
    fn test_serialize() {
        let to_test = [
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))),
            MessageType::Write(WriteType::Motor(MotorSubSystem::Right(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellBed(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(10.))),
            MessageType::FixedLog(FixedLogType::BatteryStatus(BatteryStatus(129.))),
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
                129.,
            )))),
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
                129.,
            )))),
        ];

        for msg in to_test {
            let serialzed: CanMessage = msg.clone().into();
            println!("ID : {:?}", serialzed.id);
            let de: MessageType = MessageType::try_from(&serialzed).unwrap();
            println!("Expected {:?} got {:?}", msg, de);
            assert!(msg == de);
        }
    }
    #[test]
    fn test_serialize_queue() {
        let to_test = [
            MessageType::FixedLog(FixedLogType::BatteryStatus(BatteryStatus(129.))),
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
                129.,
            )))),
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
                129.,
            )))),
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))),
            MessageType::Write(WriteType::Motor(MotorSubSystem::Right(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellBed(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(10.))),
        ];
        let mut buffer = Sender::<10>::new();
        to_test
            .iter()
            .for_each(|el| buffer.enqueue(el.clone()).unwrap());

        while let Some(msg) = buffer.dequeue() {
            println!("Expected {:?}", msg.data());
            let serialzed: CanMessage = msg;
            let de: MessageType = MessageType::try_from(&serialzed).unwrap();
            println!("Expected {:?}", de);
        }
    }
}

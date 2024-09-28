use core::borrow::BorrowMut;

use embedded_can::{Frame, Id, StandardId};
use heapless::{
    binary_heap::{Max, Min},
    BinaryHeap,
};
//use logging::LogLevel;

const LEFT_MOTOR: StandardId = unsafe { StandardId::new_unchecked(0) };
const RIGHT_MOTOR: StandardId = unsafe { StandardId::new_unchecked(1) };
const SENSOR_THETA: StandardId = unsafe { StandardId::new_unchecked(10) };
const SENSOR_ALPHA: StandardId = unsafe { StandardId::new_unchecked(11) };
const SENSOR_L_FRONT: StandardId = unsafe { StandardId::new_unchecked(12) };
const SENSOR_L_BED: StandardId = unsafe { StandardId::new_unchecked(13) };

const MOTOR_DIAG_LEFT: StandardId = unsafe { StandardId::new_unchecked(30) };
const MOTOR_DIAG_RIGHT: StandardId = unsafe { StandardId::new_unchecked(31) };
const BATTERY_DIAG: StandardId = unsafe { StandardId::new_unchecked(32) };

const LOG_TRACE: StandardId = unsafe { StandardId::new_unchecked(100) };
const LOG_INFO: StandardId = unsafe { StandardId::new_unchecked(120) };
const LOG_DEBUG: StandardId = unsafe { StandardId::new_unchecked(130) };
const LOG_WARN: StandardId = unsafe { StandardId::new_unchecked(140) };
const LOG_ERROR: StandardId = unsafe { StandardId::new_unchecked(150) };

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
    Sensor(SensorSubSystem),
    Motor(MotorSubSystem),
}

/// Denotes a log for a specific subsystem.
#[derive(Clone, PartialEq, Debug)]
pub enum FixedLogType {
    /// The status of the battery.
    BatteryStatus(BatteryStatus),
    /// The velocity
    Velocity(VelocityInfo),
}

/// The velocity of the given motor.
#[derive(Clone, PartialEq, Debug)]
pub struct VelocityInfo(MotorSubSystem);

/// The battery status at the time of logging.
#[derive(Clone, PartialEq, Debug)]
pub struct BatteryStatus(
    /// The voltage over the battery at the time of logging.
    f32,
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
pub enum ParsingError {
    IncorrectId,
    InsufficientBytes,
}

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
            Id::Standard(LEFT_MOTOR) | Id::Standard(RIGHT_MOTOR) => {
                WriteType::Motor(MotorSubSystem::try_from(value)?)
            }
            Id::Standard(SENSOR_ALPHA)
            | Id::Standard(SENSOR_THETA)
            | Id::Standard(SENSOR_L_BED)
            | Id::Standard(SENSOR_L_FRONT) => WriteType::Sensor(SensorSubSystem::try_from(value)?),
            _ => return Err(ParsingError::IncorrectId),
        })
    }
}

impl Message<CanMessage> for WriteType {}

impl Into<CanMessage> for MotorSubSystem {
    fn into(self) -> CanMessage {
        match self {
            Self::Left(val) => CanMessage::new(Id::Standard(LEFT_MOTOR), &val.to_le_bytes())
                .expect("Invalid parser"),
            Self::Right(val) => CanMessage::new(Id::Standard(RIGHT_MOTOR), &val.to_le_bytes())
                .expect("Invalid parser"),
        }
    }
}

impl TryFrom<&CanMessage> for MotorSubSystem {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        let left = match value.id() {
            Id::Standard(LEFT_MOTOR) => true,
            Id::Standard(RIGHT_MOTOR) => false,
            _ => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let mut buff = [
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
                CanMessage::new(Id::Standard(SENSOR_ALPHA), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
            Self::ThetaSensor(val) => {
                CanMessage::new(Id::Standard(SENSOR_THETA), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
            Self::LoadCellFront(val) => {
                CanMessage::new(Id::Standard(SENSOR_L_FRONT), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
            Self::LoadCellBed(val) => {
                CanMessage::new(Id::Standard(SENSOR_L_BED), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
        }
    }
}

impl TryFrom<&CanMessage> for SensorSubSystem {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        let idx = match value.id() {
            Id::Standard(SENSOR_ALPHA) => 0,
            Id::Standard(SENSOR_THETA) => 1,
            Id::Standard(SENSOR_L_BED) => 2,
            Id::Standard(SENSOR_L_FRONT) => 3,
            _ => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let mut buff = [
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
                CanMessage::new(Id::Standard(BATTERY_DIAG), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
            Self::Velocity(VelocityInfo(MotorSubSystem::Left(val))) => {
                CanMessage::new(Id::Standard(MOTOR_DIAG_LEFT), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
            Self::Velocity(VelocityInfo(MotorSubSystem::Right(val))) => {
                CanMessage::new(Id::Standard(MOTOR_DIAG_RIGHT), &val.to_le_bytes())
                    .expect("Invalid parser")
            }
        }
    }
}
impl TryFrom<&CanMessage> for FixedLogType {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        let idx = match value.id() {
            Id::Standard(MOTOR_DIAG_LEFT) => 0,
            Id::Standard(MOTOR_DIAG_RIGHT) => 1,
            Id::Standard(BATTERY_DIAG) => 2,
            _ => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let mut buff = [
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
            Id::Standard(LEFT_MOTOR)
            | Id::Standard(RIGHT_MOTOR)
            | Id::Standard(SENSOR_ALPHA)
            | Id::Standard(SENSOR_THETA)
            | Id::Standard(SENSOR_L_BED)
            | Id::Standard(SENSOR_L_FRONT) => Self::Write(WriteType::try_from(value)?),
            Id::Standard(MOTOR_DIAG_LEFT)
            | Id::Standard(MOTOR_DIAG_RIGHT)
            | Id::Standard(BATTERY_DIAG) => Self::FixedLog(FixedLogType::try_from(value)?),
            _ => return Err(ParsingError::IncorrectId),
        })
    }
}

impl Message<CanMessage> for MessageType {}

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

    fn new_remote(id: impl Into<embedded_can::Id>, dlc: usize) -> Option<Self> {
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

pub struct Sender<const N: usize> {
    buffer: heapless::BinaryHeap<MessageType, Max, N>,
}

#[derive(Clone, Copy, Debug)]
pub struct OOM();

impl<const N: usize> Sender<N> {
    pub fn new() -> Self {
        Self {
            buffer: heapless::BinaryHeap::new(),
        }
    }

    pub fn enqueue(&mut self, msg: MessageType) -> Result<(), OOM> {
        self.buffer.push(msg).map_err(|_| OOM())
    }

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

    use crate::{CanMessage, MessageType, MotorSubSystem, Sender};

    #[test]
    fn test_serialize() {
        let to_test = [
            MessageType::Write(crate::WriteType::Motor(MotorSubSystem::Left(10.))),
            MessageType::Write(crate::WriteType::Motor(MotorSubSystem::Right(10.))),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::AlphaSensor(10.),
            )),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::ThetaSensor(10.),
            )),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::LoadCellBed(10.),
            )),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::LoadCellFront(10.),
            )),
            MessageType::FixedLog(crate::FixedLogType::BatteryStatus(crate::BatteryStatus(
                129.,
            ))),
            MessageType::FixedLog(crate::FixedLogType::Velocity(crate::VelocityInfo(
                MotorSubSystem::Left(129.),
            ))),
            MessageType::FixedLog(crate::FixedLogType::Velocity(crate::VelocityInfo(
                MotorSubSystem::Right(129.),
            ))),
        ];
        for msg in to_test {
            let serialzed: CanMessage = msg.clone().into();
            let de: MessageType = MessageType::try_from(&serialzed).unwrap();
            println!("Expected {:?} got {:?}", msg, de);
            assert!(msg == de);
        }
    }
    #[test]
    fn test_serialize_queue() {
        let to_test = [
            MessageType::FixedLog(crate::FixedLogType::BatteryStatus(crate::BatteryStatus(
                129.,
            ))),
            MessageType::FixedLog(crate::FixedLogType::Velocity(crate::VelocityInfo(
                MotorSubSystem::Left(129.),
            ))),
            MessageType::FixedLog(crate::FixedLogType::Velocity(crate::VelocityInfo(
                MotorSubSystem::Right(129.),
            ))),
            MessageType::Write(crate::WriteType::Motor(MotorSubSystem::Left(10.))),
            MessageType::Write(crate::WriteType::Motor(MotorSubSystem::Right(10.))),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::AlphaSensor(10.),
            )),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::ThetaSensor(10.),
            )),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::LoadCellBed(10.),
            )),
            MessageType::Write(crate::WriteType::Sensor(
                crate::SensorSubSystem::LoadCellFront(10.),
            )),
        ];
        let mut buffer = Sender::<10>::new();
        to_test
            .iter()
            .for_each(|el| buffer.enqueue(el.clone()).unwrap());

        while let Some(msg) = buffer.dequeue() {
            println!("Expected {:?}", msg.data());
            let serialzed: CanMessage = msg.into();
            let de: MessageType = MessageType::try_from(&serialzed).unwrap();
            println!("Expected {:?}", de);
        }
    }
}

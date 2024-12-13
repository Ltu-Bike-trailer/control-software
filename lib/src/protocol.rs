//! Defines the protocol used to communicate over can bus.
//!
//! This is a dead simple protocol a typical message is defined by a
//! [`Message identifier`](constants::Message) and a single f32.
pub mod constants;
//pub mod message;
pub mod sender;

#[allow(clippy::enum_glob_use)]
use constants::{InvalidMessageId, Message::*};
use embedded_can::{Frame, Id};
use can_mcp2515::drivers::message::CanMessage;

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
    /// Sets the control reference for both of the motors.
    MotorReference {
        /// The target value for the motor to achieve.
        target: f32,
        /// The time the motor should hold this reference value after which it
        /// returns to 0.
        deadline: u32,
    },
}

/// Denotes a log for a specific subsystem.
#[derive(Clone, PartialEq, Debug)]
pub enum FixedLogType {
    /// The status of the battery.
    BatteryStatus(BatteryStatus),
    /// The velocity of that given motor subsystem.
    Velocity(VelocityInfo),
    /// Logs the current torque for the motor.
    Torque(TorqueInfo),
}

/// The torque on a specific motor.
#[derive(Clone, PartialEq, Debug)]
pub struct TorqueInfo {
    /// The current acceleration
    pub acceleration: f32,
    /// The current velocity
    pub velocity: u16,
}

/// The velocity of the given motor.
#[derive(Clone, PartialEq, Debug)]
pub struct VelocityInfo(
    /// The motor sub system the message relates to.
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
    /// The current sensors on the left motor.
    CurrentSenseLeft(CurrentMeasurement),
    /// The current sensors on the right motor.
    CurrentSenseRight(CurrentMeasurement),
}

#[derive(Clone, PartialEq, Debug)]
/// A simple wrapper for the current measurements.
pub struct CurrentMeasurement {
    // This will wrap but that is fine. As long as we are aware of it.
    ts: u16,
    p1: f16,
    p2: f16,
    p3: f16,
}

impl From<(u64, [f32; 3])> for CurrentMeasurement {
    #[allow(clippy::cast_precision_loss)]
    fn from(value: (u64, [f32; 3])) -> Self {
        Self {
            ts: value.0 as u16,
            p1: value.1[0] as f16,
            p2: value.1[1] as f16,
            p3: value.1[2] as f16,
        }
    }
}
impl From<CurrentMeasurement> for (u64, [f32; 3]) {
    fn from(value: CurrentMeasurement) -> Self {
        //let from_u16 = |val: u16| -> f32 { val as f32 / ((1 << 16 - FLOAT_MAX) as f32
        // - 1.) };
        (u64::from(value.ts), [
            value.p1 as f32,
            value.p2 as f32,
            value.p3 as f32,
        ])
    }
}

impl From<CurrentMeasurement> for [u8; 8] {
    fn from(value: CurrentMeasurement) -> Self {
        let mut buffer = [0; 8];
        buffer[0..=1].copy_from_slice(&value.ts.to_le_bytes());
        buffer[2..=3].copy_from_slice(&value.p1.to_le_bytes());
        buffer[4..=5].copy_from_slice(&value.p2.to_le_bytes());
        buffer[6..=7].copy_from_slice(&value.p3.to_le_bytes());
        buffer
    }
}
impl From<[u8; 8]> for CurrentMeasurement {
    fn from(value: [u8; 8]) -> Self {
        let ts: u16 = u16::from_le_bytes([value[0], value[1]]);
        let p1: f16 = f16::from_le_bytes([value[2], value[3]]);
        let p2: f16 = f16::from_le_bytes([value[4], value[5]]);
        let p3: f16 = f16::from_le_bytes([value[6], value[7]]);
        Self { ts, p1, p2, p3 }
    }
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
#[derive(Clone, PartialEq, Eq, Debug)]
pub enum GeneralLogType {}

#[derive(Clone, Debug)]
/// The message could not be properly parsed.
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
            Self::MotorReference { target, deadline } => {
                let mut data = [0u8; 8];
                let output = target.to_le_bytes();
                data[0] = output[0];
                data[1] = output[1];
                data[2] = output[2];
                data[3] = output[3];
                let output = deadline.to_le_bytes();
                data[4] = output[0];
                data[5] = output[1];
                data[6] = output[2];
                data[7] = output[3];
                unsafe { CanMessage::new(SetMotorReference, &data).unwrap_unchecked() }
            }
        }
    }
}

impl TryFrom<&CanMessage> for WriteType {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        Ok(match value.id() {
            Id::Standard(id) => match constants::Message::try_from(id)? {
                LeftMotor | RightMotor => Self::Motor(MotorSubSystem::try_from(value)?),
                SensorAlpha | SensorTheta | SensorLBed | SensorLFront | SensorCurrentLeft
                | SensorCurrentRight => Self::Sensor(SensorSubSystem::try_from(value)?),
                _ => return Err(ParsingError::IncorrectId),
            },
            Id::Extended(_) => return Err(ParsingError::IncorrectId),
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
            Id::Extended(_) => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let buff = [
            match data.first() {
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
            Self::CurrentSenseLeft(mes) => {
                CanMessage::new(SensorCurrentLeft, &(<[u8; 8]>::from(mes))).expect("Invalid parser")
            }
            Self::CurrentSenseRight(mes) => {
                CanMessage::new(SensorCurrentRight, &(<[u8; 8]>::from(mes)))
                    .expect("Invalid parser")
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
                SensorCurrentLeft => {
                    if value.dlc != 8 {
                        return Err(ParsingError::InsufficientBytes);
                    }
                    return Ok(Self::CurrentSenseLeft(CurrentMeasurement::from(value.data)));
                }
                SensorCurrentRight => {
                    if value.dlc != 8 {
                        return Err(ParsingError::InsufficientBytes);
                    }
                    return Ok(Self::CurrentSenseRight(CurrentMeasurement::from(
                        value.data,
                    )));
                }
                _ => return Err(ParsingError::IncorrectId),
            },
            Id::Extended(_) => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let buff = [
            match data.first() {
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

impl From<FixedLogType> for CanMessage {
    fn from(value: FixedLogType) -> Self {
        match value {
            FixedLogType::BatteryStatus(BatteryStatus(val)) => {
                Self::new(BatteryDiag, &val.to_le_bytes()).expect("Invalid parser")
            }
            FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(val))) => {
                Self::new(MotorDiagLeft, &val.to_le_bytes()).expect("Invalid parser")
            }
            FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(val))) => {
                Self::new(MotorDiagRight, &val.to_le_bytes()).expect("Invalid parser")
            }
            FixedLogType::Torque(TorqueInfo {
                acceleration,
                velocity,
            }) => {
                let mut buffer = [0u8; 8];
                let data = acceleration.to_le_bytes();
                let data2 = velocity.to_le_bytes();
                buffer[0] = data[0];
                buffer[1] = data[2];
                buffer[2] = data[2];
                buffer[3] = data[3];
                buffer[4] = data2[0];
                buffer[5] = data2[1];

                Self::new(MotorDiagRight, &buffer).expect("Invalid parser")
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
            Id::Extended(_) => return Err(ParsingError::IncorrectId),
        };
        let data = value.data();
        let buff = [
            match data.first() {
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

impl From<MessageType> for CanMessage {
    fn from(value: MessageType) -> Self {
        match value {
            MessageType::FixedLog(f) => f.into(),
            MessageType::Write(w) => w.into(),
        }
    }
}

impl TryFrom<&CanMessage> for MessageType {
    type Error = ParsingError;

    fn try_from(value: &CanMessage) -> Result<Self, Self::Error> {
        Ok(match value.id() {
            Id::Standard(id) => match id.try_into()? {
                LeftMotor | RightMotor | SensorAlpha | SensorTheta | SensorLBed | SensorLFront
                | SensorCurrentLeft | SensorCurrentRight | SetMotorReference => {
                    Self::Write(WriteType::try_from(value)?)
                }
                MotorDiagLeft | MotorDiagRight | BatteryDiag | TorqueDiag => {
                    Self::FixedLog(FixedLogType::try_from(value)?)
                }
            },
            Id::Extended(_) => return Err(ParsingError::IncorrectId),
        })
    }
}

impl Message<CanMessage> for MessageType {}

///// A simple dummy `CanMessage`.
//pub struct CanMessage {
//    id: embedded_can::Id,
//    dlc: u8,       // Data length code = length of data field.
//    data: [u8; 8], // D7 - D0
//}

//impl Frame for CanMessage {
//    fn new(id: impl Into<embedded_can::Id>, data: &[u8]) -> Option<Self> {
//        let mut buffer = [0; 8];
//        for (idx, el) in data.iter().enumerate() {
//            debug_assert!(idx < 8);
//            buffer[idx] = *el;
//        }
//        Some(Self {
//            id: id.into(),
//            dlc: data.len() as u8,
//            data: buffer,
//        })
//    }
//
//    fn new_remote(_id: impl Into<embedded_can::Id>, _dlc: usize) ->
// Option<Self> {        None
//    }
//
//    fn is_extended(&self) -> bool {
//        false
//    }
//
//    fn is_remote_frame(&self) -> bool {
//        false
//    }
//
//    fn id(&self) -> embedded_can::Id {
//        self.id
//    }
//
//    fn dlc(&self) -> usize {
//        self.dlc as usize
//    }
//
//    fn data(&self) -> &[u8] {
//        &self.data
//    }
//}

impl PartialOrd for MessageType {
    fn partial_cmp(&self, other: &Self) -> Option<core::cmp::Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for MessageType {
    fn cmp(&self, other: &Self) -> core::cmp::Ordering {
        match (self, other) {
            (Self::Write(WriteType::Motor(_)), Self::Write(WriteType::Motor(_)))
            | (Self::Write(WriteType::Sensor(_)), Self::Write(WriteType::Sensor(_)))
            | (Self::FixedLog(_), Self::FixedLog(_)) => core::cmp::Ordering::Equal,
            (Self::Write(WriteType::Sensor(_)), Self::Write(WriteType::Motor(_)))
            | (Self::FixedLog(_), _) => core::cmp::Ordering::Less,
            (
                Self::Write(
                    WriteType::Sensor(_)
                    | WriteType::Motor(_)
                    | WriteType::MotorReference {
                        target: _,
                        deadline: _,
                    },
                ),
                _,
            ) => core::cmp::Ordering::Greater,
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
    use can_mcp2515::drivers::message::CanMessage;
    use super::{
        //message::CanMessage,
        sender::Sender,
        BatteryStatus,
        FixedLogType,
        MessageType,
        MotorSubSystem,
        SensorSubSystem,
        VelocityInfo,
        WriteType,
    };
    //use crate::protocol::CanMessage;

    #[test]
    fn test_serialize() {
        let to_test = [
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))),
            MessageType::Write(WriteType::Motor(MotorSubSystem::Right(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellBed(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(10.))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::CurrentSenseLeft(
                super::CurrentMeasurement::from((10, [10., 0., 9.])),
            ))),
            MessageType::Write(WriteType::Sensor(SensorSubSystem::CurrentSenseRight(
                super::CurrentMeasurement::from((10, [10., 0., 9.])),
            ))),
            MessageType::FixedLog(FixedLogType::BatteryStatus(BatteryStatus(129.))),
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
                129.,
            )))),
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
                129.,
            )))),
        ];

        for msg in to_test {
            println!("Testing {:?}", msg);
            let serialized: CanMessage = msg.clone().into();
            println!(
                "ID : {:?}, data : {:?} {}",
                serialized.id, serialized.data, serialized.dlc
            );
            let de: MessageType = MessageType::try_from(&serialized).unwrap();
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
            let serialized: CanMessage = msg;
            let de: MessageType = MessageType::try_from(&serialized).unwrap();
            println!("Expected {:?}", de);
        }
    }
}

//! Defines a few message identifiers as constants and a few operations on them.
//!
//! This is what we use for determining the message type during runtime.
use core::u16;

use embedded_can::StandardId;
use statics::Iter;

/// Defines the message enumerations.
///
/// Enumerates all of the message types.
#[repr(u16)]
#[derive(Clone, Iter, Debug)]
pub enum Message {
    /// Represents a write of the left motor state.
    LeftMotor = 0,

    /// Represents a write of the left motor state.
    RightMotor = 1,
    /// Represents a write of theta sensor value.
    SensorTheta = 10,
    /// Represents a write of alpha sensor value.
    SensorAlpha = 11,
    /// Represents a write of front loadcell value.
    SensorLFront = 12,
    /// Represents a write of bed loadcell value.
    SensorLBed = 13,

    /// Represents a status message from the left motor.
    ///
    /// The value paired with this is a f32 representing the velocity.
    MotorDiagLeft = 30,
    /// Represents a status message from the right motor.
    ///
    /// The value paired with this is a f32 representing the velocity.
    MotorDiagRight = 40,
    /// Represents a status message from the batery.
    ///
    /// The value paired with this is a f32 representing the battery voltage.
    BatteryDiag = 50,
}

impl Message {
    const fn to_standard_id(self) -> StandardId {
        let repr = self as u16;
        // This should be caught by unit tests.
        debug_assert!(repr < 1 << 11);
        unsafe { StandardId::new_unchecked(repr) }
    }
}
impl From<Message> for embedded_can::Id {
    fn from(value: Message) -> Self {
        embedded_can::Id::Standard(value.to_standard_id())
    }
}

impl PartialEq<u16> for Message {
    fn eq(&self, other: &u16) -> bool {
        self.clone() as u16 == *other
    }
}

impl PartialEq<StandardId> for Message {
    fn eq(&self, other: &StandardId) -> bool {
        *self == other.as_raw()
    }
}

#[derive(Debug)]
/// The message id was not in [`Message`].
pub struct InvalidMessageId;
impl TryFrom<StandardId> for Message {
    type Error = InvalidMessageId;

    fn try_from(value: StandardId) -> Result<Self, Self::Error> {
        for el in Self::iter() {
            if el == value {
                return Ok(el);
            }
        }
        Err(InvalidMessageId)
    }
}

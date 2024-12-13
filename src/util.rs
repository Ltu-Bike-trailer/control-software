//! Provides some simple utilities such as a ring buffer for [`ControlLog`]
//! messages.
use lib::protocol::TorqueInfo;

use crate::RingBuffer;

#[derive(Debug, Clone, Default)]
/// A single Control log node.
pub struct ControlLog {
    current: f32,
    angular_acceleration: f32,
    angular_velocity: f32,
    time: u64,
    reference: f32,
}

impl ControlLog {
    #[must_use]
    /// Creates a new controller log.
    pub const fn new(
        current: f32,
        angular_acceleration: f32,
        angular_velocity: f32,
        time: u64,
        reference: f32,
    ) -> Self {
        Self {
            current,
            angular_acceleration,
            angular_velocity,
            time,
            reference,
        }
    }

    const fn null() -> Self {
        Self::new(0., 0., 0., 0, 0.)
    }
}
impl defmt::Format for ControlLog {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        // Format as hexadecimal.
        defmt::write!(fmt, "\"global_current\",\"{}\",\"angular_velocity\",\"{}\",\"angular_acceleration\",\"{}\",\"target\",\"{}\",\"time\",\"{}\";",self.current,self.angular_velocity,self.angular_acceleration,self.reference,self.time);
    }
}

/// Wraps the control data telemetry signals.
///
/// This is a simple ring buffer that can dequeue in to a can message.
pub struct Data<const N: usize> {
    buffer: RingBuffer<ControlLog, N>,
}

impl<const N: usize> defmt::Format for Data<N> {
    fn format(&self, fmt: defmt::Formatter<'_>) {
        defmt::write!(fmt, "\n");
        defmt::info!("Writing buffer");
        for node in self.buffer.borrow_data() {
            defmt::write!(fmt, "{:?}\n", node);
        }
    }
}
impl<const N: usize> Data<N> {
    const fn null() -> Self {
        Self {
            buffer: RingBuffer::new([const { ControlLog::null() }; N]),
        }
    }

    /// Enqueues a new message in to the buffer.
    pub fn write(&mut self, data: ControlLog) {
        self.buffer.assign_next(data);
    }

    /// Gets the n latest control logs as can messages.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_sign_loss,
        clippy::cast_possible_truncation
    )]
    #[must_use]
    pub fn get_n_latest<const DEQUEUE: usize>(&self) -> lib::protocol::sender::Sender<DEQUEUE> {
        let data = self.buffer.borrow_data();
        let mut queue = lib::protocol::sender::Sender::new();
        for el in 0..DEQUEUE {
            let n = self.buffer.ptr.wrapping_sub(el).clamp(0, N - 1);
            let datapoint = &data[n];
            let _ = queue.enqueue(lib::protocol::MessageType::FixedLog(
                lib::protocol::FixedLogType::Torque(TorqueInfo {
                    acceleration: datapoint.angular_acceleration,
                    velocity: datapoint.angular_velocity as u16,
                }),
            ));
        }
        queue
    }
}

/// A simple ring buffer for control messages.
#[unsafe(link_section = "FLASH")]
pub static mut DATA: Data<1_000> = Data::null();

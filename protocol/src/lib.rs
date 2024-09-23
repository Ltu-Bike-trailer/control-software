//! Defines the protocol that will be used to send data back and forth inbetween
//! our subsystems.
#![cfg_attr(not(feature = "std"), no_std)]
use core::u16;

use logging::{prelude::*, LogLevel, Logger, Serializable};

pub enum HashOrString<I: Iterator<Item = u8>> {
    Hash(u32),
    String(I),
}

/// Denotes all of the supported message types.
pub enum MessageType<NameIter: Iterator<Item = u8>, ArgsIter: Iterator<Item = u8>> {
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
    GeneralLog(LogLevel, HashOrString<NameIter>, ArgsIter),
}

/// Denotes all of the write operations possible with the given system.
pub enum WriteType {
    Sensor(SensorSubSystem),
    Motor(MotorSubSystem),
}

/// Denotes a log for a specific subsystem.
pub enum FixedLogType {
    /// The status of the battery.
    BatteryStatus(BatteryStatus),
    /// The velocity
    Velocity(VelocityInfo),
}

/// The velocity of the given motor.
pub struct VelocityInfo(MotorSubSystem);

/// The battery status at the time of logging.
pub struct BatteryStatus(
    /// The voltage over the battery at the time of logging.
    f32,
);

/// Denotes all of our sensor sub systems.
#[non_exhaustive]
pub enum SensorSubSystem {
    /// Denotes the sensor subsystem for our main control input.
    LoadCellFront(f32),
    /// The theta angle.
    ThetaSensor(f32),
    /// The alpha angle.
    AlphaSensor(f32),
}

/// Denotes the motors connected to the cart and their target velocities.
pub enum MotorSubSystem {
    /// The target velocity for the left motor.
    Left(f32),
    /// The target velocity for the right motor.
    Right(f32),
}

/// Denotes our different log levels.
pub enum GeneralLogType {}

/// 8k of send buffer.
static mut TXQUEUE: [u8; u16::MAX as usize] = [0; u16::MAX as usize];
/// 8k of send buffer.
static mut TXBUILDERQUEUE: [u8; u16::MAX as usize] = [0; u16::MAX as usize];
/// 8k of receive buffer.
static mut RXQUEUE: [u8; u16::MAX as usize] = [0; u16::MAX as usize];

#[allow(dead_code)]
/// Acts as an allocator and parser.
pub struct ProtocolV1 {
    // /// Encodes (priority,start,end) where start and end are elements in the
    // /// TXQUEUE.
    // tx_priority: [(usize, usize, usize); u16::MAX as usize / 10],
    /// Number of elements currently in the transmit queue.
    tx_queue: usize,
    /// Ptr in to the TXQUEUE.
    tx_ptr: usize,
    /// Addresses that are availiable to reclaim.
    tx_reclaim: [(usize, usize); u16::MAX as usize / 10],
    /// The number of elements in the reclaim queue.
    tx_reclaim_queue: usize,
    /// Representest the treap as indexes in to lists.
    ///
    /// (parent (tx_treap) left child (tx_treap), current_node (tx_prio),right
    /// child (tx_treap))
    tx_treap: (
        usize,
        [(
            Option<usize>,
            Option<usize>,
            Option<usize>,
            (usize, usize, usize),
        ); u16::MAX as usize / 10],
    ),
    /// End of tx_treap.
    tx_treap_ptr: usize,
    /// Addresses that are availiable to reclaim in tx_treap.
    tx_treap_reclaim: [usize; u16::MAX as usize / 10],
    /// The number of elements in the treap reclaim queue.
    tx_treap_reclaim_queue: usize,
    /// Denotes the framing bit.
    latest: bool,

    /// Denotes the framing bit.
    latest_rx: bool,
    /// Carry bit from previous read.
    carry: (usize, u16),
    /// Receive buffer size.
    rx_ptr: usize,
}

pub struct Buffer<'a> {
    ptr: usize,
    end: usize,
    _data: &'a mut ProtocolV1,
}

impl<'a> Iterator for Buffer<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr == self.end {
            return None;
        }
        unsafe {
            if self.ptr >= TXQUEUE.len() {
                return None;
            }
        }
        unsafe {
            let ptr = self.ptr;
            self.ptr += 1;
            Some(TXQUEUE[ptr])
        }
    }
}

pub struct RxBuffer<'a> {
    ptr: usize,
    end: usize,
    _data: &'a mut ProtocolV1,
}

impl<'a> Iterator for RxBuffer<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr == self.end {
            return None;
        }
        unsafe {
            if self.ptr >= RXQUEUE.len() {
                return None;
            }
        }
        unsafe {
            let ptr = self.ptr;
            self.ptr += 1;
            let ret = RXQUEUE[ptr];
            println!("ret :{:#b}", ret);
            Some(ret)
        }
    }
}
pub struct BuildBuffer<'a> {
    ptr: usize,
    end: usize,
    _data: &'a mut ProtocolV1,
}

impl<'a> Iterator for BuildBuffer<'a> {
    type Item = u8;

    fn next(&mut self) -> Option<Self::Item> {
        if self.ptr == self.end {
            return None;
        }
        unsafe {
            if self.ptr >= RXQUEUE.len() {
                return None;
            }
        }
        unsafe {
            let ptr = self.ptr;
            self.ptr += 1;
            Some(RXQUEUE[ptr])
        }
    }
}

pub struct OutOfSpace {}
#[derive(Debug)]
pub enum ParsingError {
    VersionMismatch,
    InsufficientBytes,
    UnexpectedType,
}

impl<I1: Iterator<Item = u8>, I2: Iterator<Item = u8>> Logger<I1, I2> for ProtocolV1 {
    fn init() -> Self {
        Self::new()
    }

    fn combiner(&mut self, level: logging::LogLevel, hash: u32, name: I1, args: I2) {
        let mut peekable = name.peekable();
        // Nothing to do if we run out of space for messages.
        let _ = self.send(
            MessageType::GeneralLog(
                level,
                if peekable.peek().is_some() {
                    HashOrString::String(peekable)
                } else {
                    HashOrString::Hash(hash)
                },
                args,
            ),
            0,
        );
    }
}

trait ShiftRegister {
    /// Enqueues a new set of bits in the buffer.
    fn push<const N: usize>(&mut self, byte: u8);
    /// Pushes a dynamic set of bits.
    fn push_dyn(&mut self, byte: u8, n: usize);
    /// Returns true if the buffer contains atleast 7 elements
    fn can_dequeue<const N: usize>(&self) -> bool;
    /// Dequeues the first 7 bits in the buffer.
    fn dequeue<const N: usize>(&mut self) -> Option<u8>;

    /// Returns all of the bits in the buffer, up to 8 bits
    fn pop_all(&mut self) -> (usize, Option<u8>);
}

impl ShiftRegister for (usize, u16) {
    fn push<const N: usize>(&mut self, byte: u8) {
        assert!(N <= 8);
        //println!("Enqueueing {byte:#8b} for {N} bits");
        self.1 <<= N;
        self.1 |= u16::from(byte & ((1 << N) - 1));
        self.0 += N;
    }

    fn can_dequeue<const N: usize>(&self) -> bool {
        self.0 >= N
    }

    fn dequeue<const N: usize>(&mut self) -> Option<u8> {
        if self.0 <= N {
            return None;
        }

        // Number of bytes to keep in the register.
        let n: usize = self.0 - N;
        let intermediate: u16 = self.1 >> (self.0 - N);
        self.0 -= N;

        // Should equal 1 s repeating from the left and 0s for the remaining n bits.
        let mask = (1 << n) - 1;
        self.1 &= mask;

        Some(intermediate as u8)
    }

    fn push_dyn(&mut self, byte: u8, n: usize) {
        match n {
            0 => return,
            1 => self.push::<1>(byte),
            2 => self.push::<2>(byte),
            3 => self.push::<3>(byte),
            4 => self.push::<4>(byte),
            5 => self.push::<5>(byte),
            6 => self.push::<6>(byte),
            7 => self.push::<7>(byte),
            8 => self.push::<8>(byte),
            _ => panic!("Out of bounds"),
        }
    }

    fn pop_all(&mut self) -> (usize, Option<u8>) {
        match self.0 {
            1 => (1, self.dequeue::<1>()),
            2 => (2, self.dequeue::<2>()),
            3 => (3, self.dequeue::<3>()),
            4 => (4, self.dequeue::<4>()),
            5 => (5, self.dequeue::<5>()),
            6 => (6, self.dequeue::<6>()),
            7 => (7, self.dequeue::<7>()),
            8 => (8, self.dequeue::<8>()),
            _ => panic!("Out of bounds"),
        }
    }
}

impl ProtocolV1 {
    pub fn new() -> Self {
        Self {
            tx_queue: Default::default(),
            tx_ptr: Default::default(),
            tx_reclaim: [(0, 0); u16::MAX as usize / 10],
            tx_reclaim_queue: Default::default(),
            tx_treap: (0, [(None, None, None, (0, 0, 0)); u16::MAX as usize / 10]),
            tx_treap_ptr: Default::default(),
            tx_treap_reclaim: [0; u16::MAX as usize / 10],
            tx_treap_reclaim_queue: Default::default(),
            latest: false,
            latest_rx: true,
            carry: (0usize, 0u16),
            rx_ptr: 0,
        }
    }

    pub fn rx(
        &mut self,
        data: &[u8],
    ) -> Option<Result<MessageType<RxBuffer, RxBuffer>, ParsingError>> {
        for el in data {
            // println!("Parsing {el:#b} {}", self.latest_rx);
            let rx = (el & 0b1000_0000) == 0;
            if self.latest_rx != rx {
                let rx_buffer = RxBuffer {
                    ptr: 0,
                    end: self.rx_ptr - 1,
                    _data: self,
                };
                return Some(Self::parse(rx_buffer));
            }

            // This is too simple, we need to account for chaning amounts of overlap.

            self.carry.push::<7>(*el);
            while let Some(rxbyte) = self.carry.dequeue::<8>() {
                unsafe { RXQUEUE[self.rx_ptr] = rxbyte };
                println!("Dequeued {:#8b}", rxbyte);
                self.rx_ptr += 1;
            }
        }
        println!("Receiving {}", self.rx_ptr);
        let rx_buffer = RxBuffer {
            ptr: 0,
            end: self.rx_ptr + 1,
            _data: self,
        };
        let ret = Self::parse(rx_buffer);
        if let Err(e) = ret.as_ref() {
            println!("RET : {e:?}");
        }
        if ret.is_ok() {
            return Some(ret);
        }
        None
    }

    fn parse(rx_iter: RxBuffer<'_>) -> Result<MessageType<RxBuffer, RxBuffer>, ParsingError> {
        let mut peekable = rx_iter.peekable();

        let denom;
        match peekable.next() {
            Some(val) => {
                if (val & 0b1111_0000) != 0b0001_0000 {
                    return Err(ParsingError::VersionMismatch);
                }

                denom = val & 0b0000_1111;
            }
            None => return Err(ParsingError::InsufficientBytes),
        }
        println!("Denominator {denom:#8b}");
        if let Some(msg) = peekable.peek() {
            println!("typeid : {:#8b}", msg);
        }

        Ok(match denom {
            0b1100 => MessageType::Write(WriteType::Motor(MotorSubSystem::Left(
                f32::from_bytes(&mut peekable).map_err(|e| {
                    match e {
                        IntegerConversionError::InsufficientBytes => println!("Insufficient bytes"),
                        IntegerConversionError::InvalidTypeIdx => println!("Invalid type id"),
                    }
                    ParsingError::UnexpectedType
                })?,
            ))),
            0b1101 => MessageType::Write(WriteType::Motor(MotorSubSystem::Right(
                f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
            ))),
            0b1000 => MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(
                f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
            ))),
            0b1001 => MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(
                f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
            ))),
            0b1010 => MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(
                f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
            ))),
            0b0100 => MessageType::FixedLog(FixedLogType::BatteryStatus(BatteryStatus(
                f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
            ))),
            0b0101 => {
                MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
                    f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
                ))))
            }
            0b0110 => {
                MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
                    f32::from_bytes(&mut peekable).map_err(|_| ParsingError::UnexpectedType)?,
                ))))
            }
            _ => todo!(), /*
                          0b0000 => MessageType::GeneralLog(LogLevel::Trace, _id, _args),
                          0b0001 => MessageType::GeneralLog(LogLevel::Info, _id, _args),
                          0b0001 => MessageType::GeneralLog(LogLevel::Debug, _id, _args),
                          0b0010 => MessageType::GeneralLog(LogLevel::Warn, _id, _args),
                          0b0011 => MessageType::GeneralLog(LogLevel::Error, _id, _args)*/
        })
    }

    pub fn send<I1: Iterator<Item = u8>, I2: Iterator<Item = u8>>(
        &mut self,
        message: MessageType<I1, I2>,
        mut ptr: usize,
    ) -> Result<(), OutOfSpace> {
        let package_type: u8 = match &message {
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(_))) => 0b1100,
            MessageType::Write(WriteType::Motor(MotorSubSystem::Right(_))) => 0b1101,
            MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(_))) => 0b1000,
            MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(_))) => 0b1001,
            MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(_))) => 0b1010,
            MessageType::FixedLog(FixedLogType::BatteryStatus(_)) => 0b0100,
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
                _,
            )))) => 0b0101,
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
                _,
            )))) => 0b0110,
            MessageType::GeneralLog(LogLevel::Trace, _id, _args) => 0b0000,
            MessageType::GeneralLog(LogLevel::Info, _id, _args) => 0b0001,
            MessageType::GeneralLog(LogLevel::Debug, _id, _args) => 0b0001,
            MessageType::GeneralLog(LogLevel::Warn, _id, _args) => 0b0010,
            MessageType::GeneralLog(LogLevel::Error, _id, _args) => 0b0011,
        };
        let first_byte = 0b0001_0000 | package_type;
        let mut append = |byte: u8| {
            unsafe { TXBUILDERQUEUE[ptr] = byte };
            ptr += 1;
        };
        append(first_byte);

        match message {
            // Type fully distinguishable from message priority.
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(val)))
            | MessageType::Write(WriteType::Motor(MotorSubSystem::Right(val)))
            | MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(val)))
            | MessageType::Write(WriteType::Sensor(SensorSubSystem::ThetaSensor(val)))
            | MessageType::Write(WriteType::Sensor(SensorSubSystem::LoadCellFront(val))) => {
                val.into_bytes(&mut append);
            }
            // Type fully distinguishable from message priority.
            MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Left(
                val,
            ))))
            | MessageType::FixedLog(FixedLogType::Velocity(VelocityInfo(MotorSubSystem::Right(
                val,
            ))))
            | MessageType::FixedLog(FixedLogType::BatteryStatus(BatteryStatus(val))) => {
                val.into_bytes(&mut append);
            }
            MessageType::GeneralLog(_, id, args) => {
                match id {
                    HashOrString::Hash(h) => {
                        let bytes = h.to_le_bytes();
                        // Force first bit high.
                        append(bytes[0] | 0b1000_0000u8);
                        append(bytes[1]);
                        append(bytes[2]);
                        append(bytes[3]);
                    }
                    HashOrString::String(i) => {
                        let mut first = true;
                        i.for_each(|el| {
                            if first {
                                append(el & !0b1000_0000u8);
                                first = false;
                            }
                            append(el)
                        })
                    }
                }
                args.for_each(|el: u8| append(el));
            }
        }
        self.enqueue(first_byte.into(), unsafe { &mut TXBUILDERQUEUE[0..ptr] })
    }

    fn enqueue(&mut self, priority: usize, message: &mut [u8]) -> Result<(), OutOfSpace> {
        let req = (message.len() * 901) / 800;

        // Scan for open regions in the buffer.
        //
        // This is easiest to do

        // Assume that we have to fit the buffer to the end of the frame.
        let mut bounds = (self.tx_ptr, self.tx_ptr + req);
        if self.tx_reclaim_queue != 0 {
            'outer: {
                let mut shuffle = false;
                for el in 0..self.tx_reclaim_queue {
                    if shuffle {
                        // This is fine as shuffle can only be set after one itteration.
                        self.tx_reclaim[el - 1] = self.tx_reclaim[el];
                        continue;
                    }
                    let el = &mut self.tx_reclaim[el];
                    // Only claim areas with
                    if el.1 - el.0 > req {
                        bounds = (el.0, el.0 + req);
                        *el = (el.0 + req, el.1);
                        break 'outer;
                    } else if el.1 - el.0 == req {
                        shuffle = true;
                        bounds = (el.0, el.0 + req);
                        *el = (el.0 + req, el.1);
                        self.tx_reclaim_queue -= 1;
                    }
                }

                if shuffle {
                    break 'outer;
                }
                // If we could not find any reclaimable space and we cannot fit at the end of
                // the buffer we are OOM.
                unsafe {
                    if self.tx_ptr + 1 < TXQUEUE.len() {
                        self.tx_ptr += req;
                        break 'outer;
                    }
                }

                // If the buffer is full we need to overwrite previous messages.
                // Look for the longest message with the lowest priority and take that one.
                let mut root = self.tx_treap.0;
                let (parent, left, right, (prio, start, end)) = self.tx_treap.1[root];

                if prio < priority && end - start > req {
                    bounds = (start, start + req);
                    self.tx_treap.1[root] = (parent, left, right, (priority, start, start + req));
                    self.rotate(root);
                    break 'outer;
                } else if end - start > req {
                    return Err(OutOfSpace {});
                }

                while let Some(el) = self.left_child(root) {
                    root = el;
                    let (parent, left, right, (prio, start, end)) = self.tx_treap.1[root];
                    if prio < priority && end - start > req {
                        bounds = (start, start + req);
                        self.tx_treap.1[root] =
                            (parent, left, right, (priority, start, start + req));
                        self.rotate(root);
                        break 'outer;
                    } else if end - start > req {
                        return Err(OutOfSpace {});
                    }
                }
                return Err(OutOfSpace {});
            }
        }

        // TODO: Validate this assumption.
        //
        // We assume that the buffer will never actually be full.
        // self.tx_priority[self.tx_queue + 1] = (priority, bounds.0, bounds.1);
        // self.tx_queue += 1;

        // TODO: Manage framing. 1st bit should indicat buffer byte.
        let mut queue: (usize, u16) = (0, 0);
        let mut mask = 1;
        let mut queue_len = 0;
        let repl = (self.latest as u8) << 7;
        let mut lower_bound = bounds.0;
        // I really dislike this framing part.
        message.iter_mut().for_each(|el| {
            println!("Sending  {:#8b}", el);
            let to_push = queue.0 % 8;

            if to_push == 0 {
                queue.push::<1>(self.latest as u8);
                queue.push::<7>(*el >> 1);
                queue.push::<1>(self.latest as u8);
                queue.push::<1>(*el >> 7);
                //println!("Queue {:?} bin : {:#16b} FIRST", queue,queue.1);
            } else {
                let n = 8 - to_push;
                // Extract lower part of message
                let lower_part = *el & ((1 << n) - 1);
                let upper_part = *el >> n;
                queue.push_dyn(upper_part, to_push);
                queue.push::<1>(self.latest as u8);
                queue.push_dyn(lower_part, n);
                println!("Buffer({})  {n} {}",queue.0, queue.1);
            }
            //println!("Queue {:?} bin : {:#16b}", queue,queue.1);

            while let Some(msg) = queue.dequeue::<8>() {
                unsafe {
                    TXQUEUE[lower_bound] = msg;
                    println!("Pushed {:#8b}",msg);
                }
                lower_bound += 1;
            }

            /*

            //println!("Managing message {el:#b}");
            // If we have a another byte, insert that first.
            if queue_len == 7 {
                //println!("Enqueuing1 {:#b}", queue | repl);
                unsafe {
                    TXQUEUE[lower_bound] = queue | repl;
                }
                lower_bound += 1;
                queue = 0;
                queue_len = 0;
                mask = 1;
            }

            // Extract lsb and place it at the start of
            let old_queue = queue;
            queue = (*el & (mask)) << (7 - queue_len);
            /*println!(
                "queue {queue:#b} (*el & (mask)) : {:#b}, (7 - queue_len) : {}",
                ((*el & (mask))),(7 - queue_len)
            );*/
            println!(
                " Source {el:#b} Enqueuing2 {:#b} repl : {:#b} mask : {:#b}, queue : {:#b}, queue len {queue_len}",
                ((*el & !mask) >> (queue_len+1)) | old_queue | repl,repl,mask,queue
            );
            unsafe {
                TXQUEUE[lower_bound] = ((*el & !mask) >> (queue_len + 1)) | old_queue | repl;
            }
            mask <<= 1;
            mask |= 0b1;
            queue_len += 1;
            lower_bound += 1;*/
        });
        while let (_n, Some(msg)) = queue.pop_all() {
            unsafe {
                TXQUEUE[lower_bound] = msg;
            }
            lower_bound += 1;
        }
        /*
        unsafe {
            TXQUEUE[lower_bound] = queue | repl;
        }*/
        //println!("QUeue done");
        self.latest = !self.latest;

        self.insert(priority, bounds.0, bounds.1);
        Ok(())
    }

    fn insert(&mut self, key: usize, start: usize, end: usize) {
        if self.tx_treap_ptr == 0 {
            self.tx_treap.1[0] = (None, None, None, (key, start, end));
            self.tx_treap_ptr += 1;
            return;
        }

        let idx = match &mut self.tx_treap_reclaim_queue {
            // Allocate :)
            0 => {
                let ret = self.tx_treap_ptr;
                self.tx_treap_ptr += 1;
                ret
            }
            n => {
                let ret = self.tx_treap_reclaim[*n];
                *n -= 1;
                ret
            }
        };

        let mut root: usize = self.tx_treap.0;

        loop {
            if self.get_key(root) >= key {
                // Left child
                if let Some(new_root) = self.left_child(root) {
                    root = new_root;
                    continue;
                }
                self.replace_left_child(root, idx);
                break;
            }
            if let Some(new_root) = self.right_child(root) {
                root = new_root;
                continue;
            }
            self.replace_right_child(root, idx);
            break;
        }
        self.tx_treap.1[idx] = (Some(root), None, None, (key, start, end));

        self.rotate(idx);
    }

    /// TODO: Remove nodes. This is done by getting the highest priority
    /// message.
    pub fn next<'a>(&'a mut self) -> Buffer<'a> {
        let mut root: usize = self.tx_treap.0;
        while let Some(new_node) = self.right_child(root) {
            root = new_node;
        }

        if let Some(left) = self.left_child(root) {
            if let Some(parent) = self.parent(root) {
                self.tx_treap.1[parent].2 = Some(left);
            } else {
                self.set_root(left);
            }
        } else {
            if let Some(parent) = self.parent(root) {
                self.tx_treap.1[parent].2 = None;
            }
        }
        let new_node = self.tx_treap.1[root];

        self.tx_treap_reclaim[self.tx_treap_ptr] = root;
        self.tx_reclaim[self.tx_reclaim_queue] = (new_node.3 .1, new_node.3 .2);
        Buffer {
            ptr: new_node.3 .1,
            end: new_node.3 .2,
            _data: self,
        }
    }

    // TODO! Handle when ancestor is none. i.e. parent is root.
    fn rotate(&mut self, mut current: usize) {
        loop {
            let parent: usize = match self.parent(current) {
                Some(parent) => parent,
                // We have found the root.
                None => {
                    self.set_root(current);
                    return;
                }
            };

            let should_rotate = self.get_priority(parent) < self.get_priority(current);
            if !should_rotate {
                return;
            }

            // Determine if we should rotate left or right.

            let direction = self.side(current, parent);

            if !direction {
                self.rotate_right(current, parent);
            } else {
                self.rotate_left(current, parent);
            }
            current = parent;
        }
    }

    fn rotate_left(&mut self, node: usize, parent: usize) {
        let ancestor = self.replace_parent(parent, node);

        if let Some(ancestor) = ancestor {
            let _old_parent = match self.side(parent, ancestor) {
                true => self.replace_right_child(ancestor, node),
                false => self.replace_left_child(ancestor, node),
            };
            assert!(_old_parent == Some(parent));
        }

        let old_left = self.replace_left_child(node, parent);
        if let Some(old_left) = old_left {
            // Discard it as this should be the same as node.
            let _old_right = self.replace_right_child(parent, old_left);
            assert!(_old_right == Some(node));
        }
    }

    fn rotate_right(&mut self, node: usize, parent: usize) {
        let ancestor = self.replace_parent(parent, node);

        if let Some(ancestor) = ancestor {
            let _old_parent = match self.side(parent, ancestor) {
                true => self.replace_right_child(ancestor, node),
                false => self.replace_left_child(ancestor, node),
            };
            assert!(_old_parent == Some(parent));
        }

        let old_right = self.replace_right_child(node, parent);
        if let Some(old_right) = old_right {
            // Discard it as this should be the same as node.
            let _old_left = self.replace_left_child(parent, old_right);
            assert!(_old_left == Some(node));
        }
    }

    #[inline(always)]
    fn set_root(&mut self, node: usize) {
        self.tx_treap.0 = node;
    }

    /// Returns true if right child false if left child.
    #[inline(always)]
    fn side(&mut self, node: usize, parent: usize) -> bool {
        if self.left_child(parent).is_some_and(|el| el == node) {
            return false;
        }

        if self.right_child(parent).is_some_and(|el| el == node) {
            return true;
        }

        panic!("Invalid treap parent was not parent");
    }

    #[inline(always)]
    fn replace_left_child(&mut self, node: usize, new: usize) -> Option<usize> {
        self.tx_treap.1[node].1.replace(new)
    }

    #[inline(always)]
    fn replace_right_child(&mut self, node: usize, new: usize) -> Option<usize> {
        self.tx_treap.1[node].2.replace(new)
    }

    #[inline(always)]
    fn replace_parent(&mut self, node: usize, new: usize) -> Option<usize> {
        self.tx_treap.1[node].0.replace(new)
    }

    #[inline(always)]
    fn left_child(&self, node: usize) -> Option<usize> {
        self.tx_treap.1[node].1
    }

    #[inline(always)]
    fn right_child(&self, node: usize) -> Option<usize> {
        self.tx_treap.1[node].2
    }

    #[inline(always)]
    fn parent(&self, node: usize) -> Option<usize> {
        self.tx_treap.1[node].0
    }

    #[inline(always)]
    fn get_key(&self, node: usize) -> usize {
        let node = self.tx_treap.1[node].3;
        node.0
    }

    /// We want the treap to be a max treap on the length of the message.
    ///
    /// Since we want to replace the longest one with the the lowest priority.
    #[inline(always)]
    fn get_priority(&self, node: usize) -> usize {
        let node = self.tx_treap.1[node].3;
        node.2
            .checked_sub(node.1)
            .expect("Message length out of bounds")
    }
}

#[cfg(all(test, feature = "std"))]
mod test {

    use super::*;
    #[test]
    fn test_building() {
        let mut prt = ProtocolV1::new();

        prt.send::<Buffer, Buffer>(
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))),
            0,
        );

        prt.send::<Buffer, Buffer>(
            MessageType::Write(WriteType::Sensor(SensorSubSystem::AlphaSensor(10.))),
            0,
        );

        prt.send::<Buffer, Buffer>(
            MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))),
            0,
        );

        let mut buffer = Vec::new();

        {
            let mut iter = prt.next();
            while let Some(el) = iter.next() {
                buffer.push(el);
            }
        };
        println!("{:?}", buffer);

        match prt.rx(&buffer) {
            Some(Ok(MessageType::Write(WriteType::Motor(MotorSubSystem::Left(10.))))) => {}
            Some(Ok(other)) => {
                panic!("Parsing error");
            }
            Some(Err(e)) => {
                panic!("Error {e:?}");
            }
            None => {
                panic!("Queue management error");
            }
        }
    }
}

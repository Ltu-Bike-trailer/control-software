//! Defines the protocol that will be used to send data back and forth inbetween
//! our subsystems.

use core::u16;

/// Denotes all of the supported message types.
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
    //GeneralLog(GeneralLogType<'a>),
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
pub struct VelocityInfo {
    _motor: MotorSubSystem,
}

/// The battery status at the time of logging.
pub struct BatteryStatus {
    /// The voltage over the battery at the time of logging.
    _voltage: f32,
}

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
#[link_section = "RAM2"]
static mut TXQUEUE: [u8; u16::MAX as usize] = [0; u16::MAX as usize];
/// 8k of receive buffer.
#[link_section = "RAM2"]
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
    /// Index in to the tx treap
    tx_in_progress: Option<usize>,
    /// A pointer in to the
    rx_ptr: usize,
    /// Denotes the framing bit.
    latest: bool,
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
pub struct OutOfSpace {}

impl ProtocolV1 {
    pub fn enqueue(&mut self, priority: usize, message: &mut [u8]) -> Result<(), OutOfSpace> {
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
                self.tx_ptr += req;
                // If we could not find any reclaimable space and we cannot fit at the end of
                // the buffer we are OOM.
                unsafe {
                    if self.tx_ptr >= TXQUEUE.len() {
                        return Err(OutOfSpace {});
                    }
                }
            }
        }

        // TODO: Validate this assumption.
        //
        // We assume that the buffer will never actually be full.
        // self.tx_priority[self.tx_queue + 1] = (priority, bounds.0, bounds.1);
        // self.tx_queue += 1;

        // TODO: Manage framing. 1st bit should indicat buffer byte.
        let mut queue: u8 = 0;
        let mut mask = 1;
        let mut queue_len = 0;
        let repl = (self.latest as u8) << 7;
        let mut lower_bound = bounds.0;
        // I really dislike this framing part.
        message.iter_mut().for_each(|el| {
            // If we have a nother byte, insert that first.
            if queue_len == 7 {
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
            mask <<= 1;
            mask |= 0b1;
            unsafe {
                TXQUEUE[lower_bound] = ((*el & mask) >> queue_len) | old_queue | repl;
            }
            queue_len += 1;
            lower_bound += 1;
        });
        unsafe {
            TXQUEUE[lower_bound] = queue | repl;
        }
        self.latest = !self.latest;

        self.insert(priority, bounds.0, bounds.1);
        Ok(())
    }

    pub fn insert(&mut self, key: usize, start: usize, end: usize) {
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
        let mut root = self.tx_treap.0;
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
            let parent = match self.parent(current) {
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

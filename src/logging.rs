#![allow(warnings)]

use core::{cell::UnsafeCell, mem::MaybeUninit};

pub struct BuffLogger {
    // We get a list of 100 u64s.
    format_hash: [u64; 100],
    draining: bool,
    filling: bool,
    data: [u8; 1024],
    buffer_count: usize,
}

static mut BUFF_LOGGER: MaybeUninit<BuffLogger> = MaybeUninit::uninit();

pub struct Logger {}

trait Lock {
    type Inner: Drop;
    fn lock<'a>(&'a mut self) -> &'a mut Self::Inner;
}

struct BuffMutator {}

#[cfg(any(not(feature = "cortex-m")))]
/// Waits for interrupts.
fn wfi() {
    panic!()
}
#[cfg(any(not(feature = "cortex-m")))]
/// Dissables interrupts.
fn dissable_interrupt() {
    panic!()
}
#[cfg(any(not(feature = "cortex-m")))]
/// Enables interrupts.
fn enable_interrupt() {
    panic!()
}

/// Waits for interrupts.
#[cfg(feature = "cortex-m")]
fn wfi() {
    cortex_m::asm::wfi();
}

#[cfg(feature = "cortex-m")]
/// Dissables interrupts.
fn dissable_interrupt() {
    cortex_m::interrupt::disable();
}
#[cfg(feature = "cortex-m")]
/// Dissables interrupts.
fn enable_interrupt() {
    cortex_m::interrupt::enable();
}

impl Lock for MaybeUninit<BuffLogger> {
    type Inner = BuffMutator;

    fn lock<'a>(&'a mut self) -> &'a mut Self::Inner {
        unsafe {
            let logger = self.assume_init_mut();
            loop {
                if logger.filling || logger.draining {
                    wfi();
                    continue;
                }
            }

            dissable_interrupt();
            logger.filling = true;
        }
        &mut BuffMutator {}
    }
}

impl Drop for BuffMutator {
    fn drop(&mut self) {
        unsafe {
            let logger = BUFF_LOGGER.assume_init_mut();
            dissable_interrupt();
            logger.filling = true;
        }
    }
}

impl BuffLogger {
    fn init() -> Logger {
        unsafe {
            BUFF_LOGGER = MaybeUninit::new(BuffLogger {
                format_hash: [0; 100],
                draining: false,
                filling: false,
                data: [0; 1024],
                buffer_count: todo!(),
            });
        }

        Logger {}
    }
}

pub trait Serializable {
    const BUFFER_SIZE: usize;
    type Error:Sized;
    /// Returns number of bytes used and buffer.
    fn into_bytes<'a>(&'a self) -> (usize,[u8; Self::BUFFER_SIZE]);
    fn from_bytes<'a>(data: &'a mut [u8]) -> Result<Self, Self::Error>
    where Self:Sized;
}



// Be a good citizen and pass the actual logger after locking it with rtic :)
// Things might break otherwise. I assumes that locking is done externally.
macro_rules! info {
    ($logger:ident, $fmt_str:literal, $($token:ident,)*) => {
        #[cfg(feature = "defmt")]
        {
            use defmt::info;
            info!($fmt_str,$($token,)*);
        }
        #[cfg(all(features = "info",any("can","ble")))]
        {
            use statics::hash;
            let hash = hash!($fmt_str);
            let arguments = [$($token::into(),)*];
            let mut ptr = 0;
            const BUFFER_LEN:usize = 200;
            let mut format_arg_buffer = [0;BUFFER_LEN];
            'outer:{
                for el in arguments.iter() {
                    let (nbytes,to_extend) = buffer.into_bytes();
                    for i in 0..(core::cmp::min(nbytes,el::BUFFER_SIZE)) {
                        if ptr < BUFFER_LEN {
                            format_arg_buffer[ptr] = to_extend[i];
                        }
                        else {
                            // HMMMMM
                            // WHAT TO DO NOW
                            break 'outer;
                        }
                        ptr += 1;
                    }
                }
            }

        }
    };
}

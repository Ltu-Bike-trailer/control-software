#![allow(warnings)]
#![allow(incomplete_features)]
#![feature(generic_const_exprs)]
#![cfg_attr(not(feature = "std"), no_std)]
#![cfg(feature = "std")]
extern crate std as core;

use core::{
    borrow::BorrowMut,
    cell::UnsafeCell,
    marker::PhantomData,
    mem::MaybeUninit,
    ops::FnMut,
    unimplemented,
};

pub const LOGGER_BUFFER_SIZE: usize = 1024;
pub enum LogLevel {
    Trace,
    Info,
    Debug,
    Warn,
    Error,
}

pub struct BuffLogger {
    // We get a list of 100 u64s.
    format_hash: [u64; 100],
    draining: bool,
    filling: bool,
    data: [u8; LOGGER_BUFFER_SIZE],
    hash_ref: [u8; 8],
    buffer_count: usize,
}

static mut BUFF_LOGGER: MaybeUninit<BuffLogger> = MaybeUninit::uninit();

pub struct Logger<
    'buffers,
    'ret,
    F: FnMut(LogLevel, &'buffers [u8], u64, &'buffers [u8]) -> (&'ret [u8],usize),
> {
    combiner: F,
    data: PhantomData<&'buffers PhantomData<()>>,
    retdata: PhantomData<&'ret PhantomData<()>>,
}

impl BuffLogger {
    fn init<
        'buffers,
        'ret,
        F: FnMut(LogLevel, &'buffers [u8], u64, &'buffers [u8]) -> (&'ret [u8],usize),
    >(
        combiner: F,
    ) -> Logger<'buffers, 'ret, F> {
        unsafe {
            BUFF_LOGGER = MaybeUninit::new(BuffLogger {
                format_hash: [0; 100],
                draining: false,
                filling: false,
                data: [0; 1024],
                hash_ref: [0; 8],
                buffer_count: 0,
            });
        }
        Logger {
            combiner,
            data: PhantomData,
            retdata: PhantomData,
        }
    }

    fn name_or_hash<'a>(&'a mut self, name: &'a [u8], hash: &'a u64, data: &mut &'a [u8]) ->usize {
        let mut last_idx = 0;
        for (idx, el) in self.format_hash.iter().enumerate() {
            if self.format_hash.contains(hash) {
                self.hash_ref = hash.to_le_bytes();

                *data = &self.hash_ref;
                return 8;
            }
            last_idx = idx;
        }
        self.format_hash[last_idx] = *hash;
        let n = name.len();
        *data = name;
        n
    }
}

impl BuffLogger {
    /// Coppies in to the buffer, if buffer is overrun it discards the packages.
    pub fn extend<'data>(&mut self, data: &'data [u8]) {
        for el in data.iter() {
            if self.buffer_count > LOGGER_BUFFER_SIZE {
                // HMMM
                return;
            }

            self.data[self.buffer_count] = *el;
            self.buffer_count += 1;
        }
    }

    /// This assumes thtat the data buffer is larger than [`LOGGER_BUFFER_SIZE`]
    pub fn drain<'drain_buffer>(&mut self, data: &'drain_buffer mut [u8]) -> usize {
        for (idx, el) in self.data[0..self.buffer_count].iter().enumerate() {
            data[idx] = *el;
        }
        let ret = self.buffer_count;
        self.buffer_count = 0;
        ret
    }
}
impl<'buffers, 'ret, F: FnMut(LogLevel, &'buffers [u8], u64, &'buffers [u8]) -> (&'ret [u8],usize)>
    Logger<'buffers, 'ret, F>
{
    /// This assumes thtat the data buffer is larger than [`LOGGER_BUFFER_SIZE`]
    pub fn drain<'drain_buffer>(&self, data: &'drain_buffer mut [u8]) -> usize {
        unsafe {
            let buffer = BUFF_LOGGER.borrow_mut().assume_init_mut();
            buffer.drain(data)
        }
    }
}

pub trait Serializable {
    const BUFFER_SIZE: usize;
    type Error: Sized;
    /// Returns number of bytes used and buffer.
    fn into_bytes<'a>(&'a self) -> (usize, [u8; Self::BUFFER_SIZE]);
    fn from_bytes<'a>(data: &'a mut [u8]) -> Result<Self, Self::Error>
    where
        Self: Sized;
}

impl Serializable for i32 {
    type Error = ();

    const BUFFER_SIZE: usize = 4;

    /// Returns number of bytes used and buffer.
    fn into_bytes<'a>(&'a self) -> (usize, [u8; Self::BUFFER_SIZE]) {
        (4, self.to_le_bytes())
    }

    fn from_bytes<'a>(data: &'a mut [u8]) -> Result<Self, Self::Error> {
        unimplemented!()
    }
}

// Be a good citizen and pass the actual logger after locking it with rtic :)
// Things might break otherwise. I assumes that locking is done externally.
#[macro_export]
macro_rules! info {
    ($logger:ident, $fmt_str:literal, $($token:ident),*) => {
        #[cfg(all(feature = "info",feature = "defmt"))]
        {
            use defmt::info;
            info!($fmt_str,$($token,)*);
        }
        #[cfg(all(feature = "info",any(feature="can",feature="ble")))]
        {
            log!($logger, $fmt_str,$($token),*;LogLevel::Info);
        }
    };
}

#[macro_export]
macro_rules! log {
    ($logger:ident, $fmt_str:literal, $($token:ident),*;$loglevel:expr) => {
        use statics::{hash,string_to_bytes};
        use crate::{BUFF_LOGGER,Serializable};
        let arguments = [$($token,)*];
        let mut ptr = 0;

        const BUFFER_LEN:usize = 200;
        let mut format_arg_buffer = [0u8;BUFFER_LEN];

        'outer:{
            for el in arguments.iter() {
                let (nbytes,to_extend) = el.into_bytes();
                for i in 0..(nbytes) {
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

        use core::borrow::BorrowMut;
        unsafe{
            let buffer = BUFF_LOGGER.borrow_mut().assume_init_mut();
            let no_name = [];
            let mut name = &no_name[..];
            let n = buffer.name_or_hash(&string_to_bytes!($fmt_str),&hash!($fmt_str),&mut name);
            let (data,n) = ($logger.combiner)($loglevel,&name[0..n], hash!($fmt_str),&format_arg_buffer[0..ptr]);
            buffer.extend(&data[0..n]);
        };
    }
}

#[cfg(all(test, feature = "std"))]
mod test {
    #[macro_use]
    use super::{info, BuffLogger, LogLevel, BUFF_LOGGER};
    static mut DATA: Vec<u8> = vec![];
    #[test]
    fn test_info() {
        fn combiner<'buffers, 'ret>(
            level: LogLevel,
            fmt_str: &'buffers [u8],
            hash: u64,
            args: &'buffers [u8],
        ) -> (&'ret [u8],usize) {
            unsafe {
                let _ = DATA.drain(..);
                DATA.resize(0,0);
                let n = fmt_str.len() + args.len();
                DATA.extend(fmt_str);
                DATA.extend(args);

                (&DATA,n)
            }
        }
        let logger = BuffLogger::init(combiner);
        let i: i32 = 1;
        let mut buffer = [0; 225];
        let n = logger.drain(&mut buffer);
        println!("Buffer {:?}", &buffer[0..n]);
        info!(logger, "Some formatting string {}", i);
        let mut buffer = [0; 225];
        let n = logger.drain(&mut buffer);
        println!("Buffer {:?}", &buffer[0..n]);
        info!(logger, "Some formatting string {}", i);
        let mut buffer = [0; 225];
        let n = logger.drain(&mut buffer);
        println!("Buffer {:?}", &buffer[0..n]);
        panic!()
    }
}

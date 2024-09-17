#[macro_use]
use logging::{info, prelude::*, BuffLogger, LogLevel, prelude::*};
use statics::Serializer;
static mut DATA: Vec<u8> = vec![];

#[derive(Serializer)]
struct SomeCoolStruct {
    field_a: u32,
    field_b: i32,
}

#[derive(Serializer)]
struct SomeCoolStrut2 {
    field_a: u32,
}

fn main() {
    fn combiner<'buffers, 'ret>(
        level: LogLevel,
        fmt_str: &'buffers [u8],
        hash: u32,
        args: &'buffers [u8],
    ) -> (&'ret [u8], usize) {
        unsafe {
            println!("Sending str : {fmt_str:?}");
            let _ = DATA.drain(..);
            DATA.resize(0, 0);
            let n = fmt_str.len() + args.len();
            DATA.extend(fmt_str);
            DATA.extend(args);

            (&DATA, n)
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
    let data = SomeCoolStruct {
        field_a: 0,
        field_b: 4,
    };
    info!(logger, "Some formatting string {}", data);
    let mut buffer = [0; 225];
    let n = logger.drain(&mut buffer);
    println!("Buffer {:?}", &buffer[0..n]);
    let data = SomeCoolStrut2 { field_a: 0 };
    info!(logger, "Some formatting string {}", data);
    let mut buffer = [0; 225];
    let n = logger.drain(&mut buffer);
    println!("Buffer {:?}", &buffer[0..n]);
    info!(logger, "SOME REALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLY long log message {}", i);
    let mut buffer = [0; 225];
    let n = logger.drain(&mut buffer);
    println!("Buffer {:?}", &buffer[0..n]);
    info!(logger, "SOME REALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLYREALLY long log message {}", i);
    let mut buffer = [0; 225];
    let n = logger.drain(&mut buffer);
    println!("Buffer {:?}", &buffer[0..n]);
}

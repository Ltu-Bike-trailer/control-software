const fn hash(data: &'static str) -> u64 {
    const RADIX: u64 = 10;
    let mut prev_sum = 0;
    data.chars()
        .map(|c| {
            let num = c.to_digit(RADIX).unwrap();
            prev_sum = prev_sum ^ num;
            prev_sum
        })
        .sum::<u64>()
}

pub struct BuffLogger<const BUFFER_SIZE: usize> {
    // We get a list of 100 u64s.
    format_hash: [u64; 100],
    draining: bool,
    filling: bool,
    data: [u8; BUFFER_SIZE],
}

impl<const BUFFER_SIZE: usize> BuffLogger<BUFFER_SIZE> {}

macro_rules! info {
    ($fmt_str:literal, $($token:ident,)*) => {};
}

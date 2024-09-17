pub mod ext {
    use crate::Serializable;
    pub enum IntegerConversionError {
        InsufficientBytes,
        InvalidTypeIdx,
    }
    macro_rules! extend {
    ($($type_idx:literal :$type_name:ty : $size:literal);*) => {
        $(
        impl Serializable for $type_name {
            type Error = IntegerConversionError;

            const BUFFER_SIZE: usize = {$size + 1};

            fn from_bytes<'a>(ptr: usize, input: &'a mut [u8]) -> Result<(usize, Self), Self::Error>
            where
                Self: Sized,
            {
                match input.get(0).ok_or(IntegerConversionError::InsufficientBytes)? {
                    $type_idx => {},
                    _ => {
                        return Err(IntegerConversionError::InvalidTypeIdx)
                    }
                }
                let mut data: [u8; $size] = [0; $size];
                for idx in 0..(Self::BUFFER_SIZE) {
                    data[idx] = input
                        .get(idx + 1)
                        .ok_or(IntegerConversionError::InsufficientBytes)?
                        .clone();
                }
                Ok((Self::BUFFER_SIZE, Self::from_le_bytes(data)))
            }

            fn into_bytes<'a>(&'a self) -> (usize, [u8; Self::BUFFER_SIZE]) {

                let mut data: [u8; Self::BUFFER_SIZE] = [$type_idx; Self::BUFFER_SIZE];
                let buffer = self.to_le_bytes();
                for idx in 0..($size) {
                    data[idx] = buffer
                        .get(idx).unwrap().clone()
                }
                (Self::BUFFER_SIZE, data)
            }
        })*
    };
}
    extend!(
        1 : u8 : 1;
        2 : u16 : 2;
        3 : u32 : 4;
        4 : u64 : 8;
        5 : i8 : 1;
        6 : i16 : 2;
        7 : i32 : 4;
        8 : i64 : 8
    );
}

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

            fn from_bytes<'a,I:Iterator<Item= u8>>(input: &'a mut I) -> Result<Self, Self::Error>
            where
                Self: Sized,
            {
                let denom = match input.next() {
                    Some(denom) => denom,
                    None => return Err(IntegerConversionError::InsufficientBytes)
                };
                match denom {
                    $type_idx => {},
                    _ => {
                        return Err(IntegerConversionError::InvalidTypeIdx)
                    }
                }
                let mut data: [u8; $size] = [0; $size];
                for idx in 0..(Self::BUFFER_SIZE) {
                    let next = match input.next() {
                        Some(val) => val,
                        None => return Err(IntegerConversionError::InsufficientBytes)
                    };
                    data[idx] = next;
                }
                Ok(Self::from_le_bytes(data))
            }

            fn into_bytes<'a,F:FnMut(u8)>(&'a self,mut write:&mut F) {

                write($type_idx);

                self.to_le_bytes().iter().for_each(|el| write(*el));
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
        8 : i64 : 8;
        9 : f32 : 4;
        10 : f64 : 8
    );
}

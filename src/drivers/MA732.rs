use embedded_hal::{digital::OutputPin, spi::SpiBus};
use rtic_monotonics::{fugit::ExtU64, Monotonic};
///Struct containing the needed pins to drive MA732 sensor.
pub struct MA732Driver</*SPI: SpiBus,*/ PIN: OutputPin> {
    /// Spi configured with MISO, MOSI and clk pins.
    // pub spi: SPI,
    /// Chip-Select Pin, for driving the pin from high to low
    pub cs: PIN,
}

#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
enum InstructionCommand {
    /// read angle command bits
    _ReadAngle = 0b000,
    /// read register command bits
    ReadRegister = 0b010,
    ///write register command bits
    WriteRegister = 0b100,
}

/// MA732 IC module register addresses
#[repr(u8)]
#[allow(clippy::upper_case_acronyms)]
pub enum MA732Register {
    /// First register address for Zero setting
    ZeroSetting1 = 0x0,
    /// Second register address for Zero setting
    ZeroSetting2 = 0x1,
    /// register address for bias current trimming
    BiasCurrentTrimming = 0x2,
    //0x3,
    //0x4,
    //0x5,
    //0x6,
    /// register address for rotation direction
    RotationDIrection = 0x9,
    //0xE,
    //0x10,
    /// register address for mgh & mgl flags
    MghMgl = 0x1B,
}

impl</*SPI: SpiBus,*/ PIN: OutputPin> MA732Driver<PIN> {
    /// Inits a new instance taking ownership of the needed pints
    /// TODO spi should be shared for multible instances, or chip select should
    /// maybe be passed to the read/write commands to make it able to point to
    /// multiple senores.
    pub fn new( cs: PIN) -> Self {
        Self { cs }
    }

    /// reads the angle from the MA732 sensor
    /// caution there needs to be at least 150ns between polls of this
    pub fn read_angle<SPI:SpiBus>(&mut self, spi: &mut SPI,) -> u16 {
        let mut angle: [u8; 2] = [0; 2];
        self.cs.set_low().ok();
        spi.read(&mut angle).ok();
        self.cs.set_high().ok();

        return u16::from_be_bytes(angle);
    }

    /// Reads a specified register
    pub async fn read_register<
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>>,
        SPI: SpiBus,
        const NOM: u32,
        const DENOM: u32,
    >(
        &mut self,
        register: MA732Register,
        spi: &mut SPI,
    ) -> Result<u8, ()> {
        //|Instruction Command 3 bits|Register 5 bits| new value 8 bits|
        // Shift and combine above data into 2 [u8;2]
        let write: [u8; 2] = [
            ((InstructionCommand::ReadRegister as u8 & 0b111) << 3) | register as u8 & 0b1_1111,
            0,
        ];

        // Ignore the read angle and send read register command
        self.cs.set_low().ok();
        spi.transfer(&mut [0; 2], &write).ok();
        self.cs.set_high().ok();

        // Read the specified register after delaying
        T::delay(20.millis()).await;
        let mut read: [u8; 2] = [0; 2];
        self.cs.set_low().ok();
        spi.read(&mut read).ok();
        self.cs.set_high().ok();

        //first 8 bits contains the register value
        return Ok(read[0]);
    }

    /// writes a new value to a specified register
    pub fn write_register<SPI:SpiBus>(&mut self, register: MA732Register, spi: &mut SPI, new_value: u8) {
        //|Instruction Command 3 bits|Register 5 bits| new value 8 bits|
        // Shift and combine above data into 2 [u8;2]
        let write: [u8; 2] = [
            ((InstructionCommand::WriteRegister as u8 & 0b111) << 3) | register as u8 & 0b1_1111,
            new_value,
        ];

        // Ignore the read angle and send write register command
        self.cs.set_low().ok();
        spi.transfer(&mut [0; 2], &write).ok();
        self.cs.set_high().ok();
    }
}

#![allow(clippy::future_not_send)]
use core::f32;
use embedded_hal::{digital::OutputPin, spi::SpiBus};
use rtic_monotonics::{fugit::ExtU64, Monotonic};

///Struct containing the needed pins to drive MA732 sensor.
pub struct Driver<PIN: OutputPin> {
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
pub enum Register {
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

/// default register values
const ZERO_SETTING1: u8 = 0b0000_0000;
const ZERO_SETTING2: u8 = 0b0000_0000;
const BIAS_CURRENT_TRIMMING: u8 = 0b0000_0000;
//0x3 = 0b0000_0000,
//0x4 = 0b1100_0000,
//0x5 = 0b1111_1111,
//0x6 = 0b0001_1100,
const ROTATION_DIRECTION: u8 = 0b0000_0000;
//0xE = 0b0111_0111,
//0x10= 0b1001_1100,

impl<PIN: OutputPin> Driver<PIN> {
    /// Inits a new instance taking ownership of the needed pints
    /// maybe be passed to the read/write commands to make it able to point to
    /// multiple senores.
    pub const fn new(cs: PIN) -> Self {
        Self { cs }
    }

    /// Reads the angle from the MA732 sensor.
    ///
    /// returns the angle in radians.
    ///
    /// ### note
    ///
    /// Caution there needs to be at least 150ns between polls of this.
    pub fn read_angle<SPI: SpiBus>(&mut self, spi: &mut SPI) -> f32 {
        let mut angle: [u8; 2] = [0; 2];
        self.cs.set_low().ok();
        spi.read(&mut angle).ok();
        self.cs.set_high().ok();

        u16_to_radians(u16::from_be_bytes(angle))
    }

    /// Reads a specified register
    pub async fn read_register<
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>>,
        SPI: SpiBus,
        const NOM: u32,
        const DENOM: u32,
    >(
        &mut self,
        register: Register,
        spi: &mut SPI,
    ) -> u8 {
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
        T::delay(1.micros()).await;
        let mut read: [u8; 2] = [0; 2];
        self.cs.set_low().ok();
        spi.read(&mut read).ok();
        self.cs.set_high().ok();

        //first 8 bits contains the register value
        read[0]
    }

    /// writes a new value to a specified register
    pub async fn write_register<
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>>,
        SPI: SpiBus,
        const NOM: u32,
        const DENOM: u32,
    >(
        &mut self,
        register: Register,
        spi: &mut SPI,
        new_value: u8,
    ) {
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

        //Delay to prevent issues from happen in the case of chained instructions
        T::delay(20.millis()).await;

        let _ = spi.transfer(&mut [0u8; 2], &[0u8; 2]);
    }

    // ============ Common register actions pre coded ============

    /// Sets new zero to given angle in radians
    pub async fn set_zero_angle<
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>>,
        SPI: SpiBus,
        const NOM: u32,
        const DENOM: u32,
    >(
        &mut self,
        angle_offset: f32,
        spi: &mut SPI,
    ) {
        //convert angle from radians to 16bit representation and write to the registers
        let zero_angle: [u8; 2] = radians_to_u16(angle_offset).to_be_bytes();

        self.write_register::<T, SPI, NOM, DENOM>(Register::ZeroSetting1, spi, zero_angle[0])
            .await;
        self.write_register::<T, SPI, NOM, DENOM>(Register::ZeroSetting2, spi, zero_angle[1])
            .await;
    }

    /// resets all registers to default.
    pub async fn reset_registers<
        T: Monotonic<Duration = rtic_monotonics::fugit::Duration<u64, NOM, DENOM>>,
        SPI: SpiBus,
        const NOM: u32,
        const DENOM: u32,
    >(
        &mut self,
        spi: &mut SPI,
    ) {
        self.write_register::<T, SPI, NOM, DENOM>(Register::ZeroSetting1, spi, ZERO_SETTING1)
            .await;
        self.write_register::<T, SPI, NOM, DENOM>(Register::ZeroSetting2, spi, ZERO_SETTING2)
            .await;
        self.write_register::<T, SPI, NOM, DENOM>(
            Register::BiasCurrentTrimming,
            spi,
            BIAS_CURRENT_TRIMMING,
        )
        .await;
        self.write_register::<T, SPI, NOM, DENOM>(
            Register::RotationDIrection,
            spi,
            ROTATION_DIRECTION,
        )
        .await;
    }
}

#[allow(clippy::cast_sign_loss)]
#[allow(clippy::cast_possible_truncation)]
const fn radians_to_u16(radians: f32) -> u16 {
    let radians = radians % f32::consts::TAU;
    ((radians / f32::consts::TAU) * u16::MAX as f32) as u16
}

#[allow(clippy::cast_possible_truncation)]
const fn u16_to_radians(angle: u16) -> f32 {
    angle as f32 / u16::MAX as f32 * f32::consts::TAU
}

//! Defines peripheral, Pin, GPIO and GPIOTE config for the
//! "High-Level-Controller.
#![deny(warnings, missing_docs)]
#![allow(unused_imports, dead_code)]
#![allow(clippy::type_complexity)]

use embedded_hal::spi::MODE_0;
use nrf52840_hal::{
<<<<<<< HEAD
    gpio::{self, p0, p1, Floating, Input, Level, Output, Pin, PullUp, PushPull},
    gpiote::Gpiote,
=======
    gpio::{self, Pin, *},
    gpiote::*,
>>>>>>> d3bcbce (rebasing with main)
    pac::GPIOTE,
    spi::{Instance, Spi},
};

/// HLC board pins for the SPI interfacing the CAN driver.
pub struct CanSpi {
<<<<<<< HEAD
    /// Pin for `sck`.
=======
    ///
>>>>>>> d3bcbce (rebasing with main)
    pub sck: Pin<Output<PushPull>>,
    /// Pin for `mosi`.
    pub mosi: Pin<Output<PushPull>>,
<<<<<<< HEAD
    /// Pin for `miso`.
=======
    ///
>>>>>>> d3bcbce (rebasing with main)
    pub miso: Pin<Input<Floating>>,
}

/// Can Pins.
pub struct CanManager {
    spi: CanSpi,
    cs: Pin<Output<PushPull>>,
    int: Pin<Input<PullUp>>,
}

impl CanManager {
    /// Creates and return the Can pin mapping
<<<<<<< HEAD
    #[must_use]
=======
>>>>>>> d3bcbce (rebasing with main)
    pub fn new(p0: p0::Parts, p1: p1::Parts) -> Self {
        Self {
            spi: CanSpi {
                sck: p0.p0_05.into_push_pull_output(Level::Low).degrade(),
                mosi: p1.p1_15.into_push_pull_output(Level::Low).degrade(),
                miso: p0.p0_02.into_floating_input().degrade(),
            },
            cs: p1.p1_08.into_push_pull_output(Level::High).degrade(),
            int: p0.p0_28.into_pullup_input().degrade(),
        }
    }

    /// Returns ownership of a tuple containing the cs pin, int pin and the
    /// `Spi` instance.
    pub fn peripheral_instances<SPI: Instance>(
        self,
        spi: SPI,
        gp: GPIOTE,
    ) -> (Pin<Output<PushPull>>, Pin<Input<PullUp>>, Spi<SPI>, Gpiote) {
        let pins = nrf52840_hal::spi::Pins {
            sck: Some(self.spi.sck),
            mosi: Some(self.spi.mosi),
            miso: Some(self.spi.miso),
        };

        let spi_device = Spi::new(spi, pins, nrf52840_hal::spi::Frequency::M2, MODE_0);
        let gpiote = Gpiote::new(gp);
        gpiote
            .channel0()
            .input_pin(&self.int)
            .hi_to_lo()
            .enable_interrupt();

        (self.cs, self.int, spi_device, gpiote)
    }
}

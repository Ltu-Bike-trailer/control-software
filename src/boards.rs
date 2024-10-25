//! Provides configurations for the cart.
pub mod event;
use event::Manager;
use nrf52840_hal::{
    gpio::{self, p0, p1, Floating, Input, Output, Pin, PushPull},
    gpiote::Gpiote,
    ppi::{self, ConfigurablePpi, Ppi},
};

/// Wraps the pins needed for the SPI.
pub struct SpiPins {
    /// Incomming packets from the slave device.
    pub mosi: Pin<Output<PushPull>>,
    /// Outgoing packets to slave device.
    pub miso: Pin<Input<Floating>>,
    /// Clock driven from us.
    pub sck: Pin<Output<PushPull>>,
}

/// The pin mapping used in the car.
pub struct PinMapping<const SPI_USED: bool, const EVENTS_CONFIGURED: bool> {
    /// The pins connected to the spi.
    can: Option<SpiPins>,
    can_cs: Pin<Output<PushPull>>,
    can_interrupt: Pin<Input<Floating>>,
}

impl PinMapping<false, false> {
    /// Instantiates a new pin mapping.
    #[must_use]
    pub fn new(p0: p0::Parts, p1: p1::Parts) -> Self {
        // TODO: Move these to other crate.

        // Communications.
        let sck = p0.p0_05.into_push_pull_output(gpio::Level::Low).degrade();
        let mosi = p0.p0_06.into_push_pull_output(gpio::Level::Low).degrade();
        let miso = p0.p0_07.into_floating_input().degrade();

        // Needs testing.
        let cs = p1.p1_08.into_push_pull_output(gpio::Level::High).degrade();
        let can_interrupt = p1.p1_06.into_floating_input().degrade();

        let spi_pins = SpiPins { mosi, miso, sck };

        Self {
            can: Some(spi_pins),
            can_cs: cs,
            can_interrupt,
        }
    }
}

impl<const EVENTS_CONFIGURED: bool> PinMapping<false, EVENTS_CONFIGURED> {
    /// Creates a valid can driver for the SPI2CAN modules.
    pub fn can<T: nrf52840_hal::spi::Instance>(
        mut self,
        device: T,
    ) -> (
        PinMapping<true, EVENTS_CONFIGURED>,
        // TODO: Make this in to DMA spi.
        nrf52840_hal::spi::Spi<T>,
    ) {
        // This is checked by the type system.
        let spis_pins = unsafe { self.can.take().unwrap_unchecked() }.into();
        let spi = nrf52840_hal::spi::Spi::new(
            device,
            spis_pins,
            nrf52840_hal::spi::Frequency::M8,
            embedded_hal::spi::MODE_0,
        );
        // // If we have any slave devices we need to do something like this.
        // spi.enable_interrupt(spis::SpisEvent::End);
        // let spis = spi.transfer(buffer).unwrap_or_else(|_| panic!());
        let new_self = PinMapping {
            can: None,
            can_cs: self.can_cs,
            can_interrupt: self.can_interrupt,
        };
        (new_self, spi)
    }
}

/// The final pin cluster retruned on consumption.
pub type FinalPins = (Pin<Output<PushPull>>, Pin<Input<Floating>>);

impl<const SPI_USED: bool> PinMapping<SPI_USED, true> {
    /// Consumes the type returning the raw pins.
    #[must_use]
    pub const fn consume(self) -> FinalPins {
        (self.can_cs, self.can_interrupt)
    }
}

impl<const SPI_USED: bool> PinMapping<SPI_USED, false> {
    /// Sets up the events for the inputs.
    #[must_use]
    pub fn configure_events(
        self,
        gpiote: Gpiote,
        ppi: ppi::Parts,
    ) -> (PinMapping<SPI_USED, true>, Manager) {
        gpiote
            .channel0()
            .input_pin(&self.can_interrupt)
            .hi_to_lo()
            .enable_interrupt();
        /*gpiote
            .channel1()
            .input_pin(&self.sonar_left.echo)
            .toggle()
            .enable_interrupt();
        gpiote
            .channel2()
            .input_pin(&self.sonar_left_2.echo)
            .toggle()
            .enable_interrupt();
        gpiote
            .channel3()
            .input_pin(&self.sonar_right.echo)
            .toggle()
            .toggle()
            .enable_interrupt();
        gpiote
            .channel4()
            .input_pin(&self.sonar_right_2.echo)
            .toggle()
            .toggle()
            .enable_interrupt();
        gpiote
            .channel5()
            .input_pin(&self.sonar_forward.echo)
            .toggle()
            .toggle()
            .enable_interrupt();*/

        gpiote.port().input_pin(&self.can_interrupt).high();

        /*gpiote.port().input_pin(&self.sonar_forward.echo).high();
        gpiote.port().input_pin(&self.sonar_left.echo).high();
        gpiote.port().input_pin(&self.sonar_right.echo).high();
        gpiote.port().input_pin(&self.sonar_left_2.echo).high();
        gpiote.port().input_pin(&self.sonar_right_2.echo).high();
        gpiote.port().input_pin(&self.sonar_forward.echo).high();*/

        gpiote.port().input_pin(&self.can_interrupt).low();
        /*gpiote.port().input_pin(&self.sonar_left.echo).low();
        gpiote.port().input_pin(&self.sonar_right.echo).low();
        gpiote.port().input_pin(&self.sonar_left_2.echo).low();
        gpiote.port().input_pin(&self.sonar_right_2.echo).low();
        gpiote.port().input_pin(&self.sonar_forward.echo).low();*/

        let mut ppi0 = ppi.ppi0;
        ppi0.set_event_endpoint(gpiote.channel0().event());
        ppi0.set_task_endpoint(gpiote.channel0().task_out());
        ppi0.enable();

        /*
        let mut ppi1 = ppi.ppi1;
        ppi1.set_event_endpoint(gpiote.channel1().event());
        ppi1.set_task_endpoint(gpiote.channel1().task_out());
        ppi1.enable();

        let mut ppi2 = ppi.ppi2;
        ppi2.set_event_endpoint(gpiote.channel2().event());
        ppi2.set_task_endpoint(gpiote.channel2().task_out());
        ppi2.enable();

        let mut ppi3 = ppi.ppi3;
        ppi3.set_event_endpoint(gpiote.channel3().event());
        ppi3.set_task_endpoint(gpiote.channel3().task_out());
        ppi3.enable();

        let mut ppi4 = ppi.ppi4;
        ppi4.set_event_endpoint(gpiote.channel4().event());
        ppi4.set_task_endpoint(gpiote.channel4().task_out());
        ppi4.enable();

        let mut ppi5 = ppi.ppi5;
        ppi5.set_event_endpoint(gpiote.channel5().event());
        ppi5.set_task_endpoint(gpiote.channel5().task_out());
        ppi5.enable();*/

        gpiote.port().enable_interrupt();
        let new_self = PinMapping {
            can: self.can,
            can_cs: self.can_cs,
            can_interrupt: self.can_interrupt,
        };
        (new_self, Manager::new(gpiote))
    }
}

impl From<SpiPins> for nrf52840_hal::spi::Pins {
    fn from(value: SpiPins) -> Self {
        Self {
            sck: Some(value.sck),
            mosi: Some(value.mosi),
            miso: Some(value.miso),
        }
    }
}

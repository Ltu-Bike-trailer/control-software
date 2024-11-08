#![no_std]
#![no_main]
#![allow(unused)]

use controller as _;
use controller::drivers::can::{Mcp2515Driver, Mcp2515Settings};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use panic_probe as _;
use rtic::app;

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC0])]
mod app {

    use controller::{
        boards::*,
        drivers::can::{
            AcceptanceFilterMask,
            Bitrate,
            Mcp2515Driver,
            Mcp2515Settings,
            McpClock,
            OperationTypes,
            ReceiveBufferMode,
            SettingsCanCtrl,
            CLKPRE,
            RXBN,
        },
    };
    use cortex_m::asm;
    use embedded_can::{blocking::Can, Frame, StandardId};
    use embedded_hal::{digital::OutputPin, spi::SpiBus};
    use lib::protocol::sender::Sender;
    use nrf52840_hal::{
        gpio::{self, Floating, Input, Level, Output, Pin, Port, PullUp, PushPull},
        gpiote::{Gpiote, GpioteInputPin},
        pac::{Interrupt, GPIOTE, SPI0, SPI1, SPI2, SPIM0, SPIM1, SPIM2},
        spi::{Frequency, Spi},
        spim,
        spim::*,
        Clocks,
    };
    use rtic_monotonics::nrf::{
        self,
        rtc::*,
        timer::{fugit::ExtU64, Timer0 as Mono},
    };

    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    #[local]
    struct Local {
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        sender: Sender<10>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);
        let t = nrf52840_hal::pac::Peripherals::take();
        let clk = Clocks::new(device.CLOCK).enable_ext_hfosc().start_lfclk();

        /* Disable shared peripheral addresses */
        device.SPIM0.enable.write(|w| w.enable().disabled());
        device.SPIS0.enable.write(|w| w.enable().disabled());
        device.TWIM0.enable.write(|w| w.enable().disabled());
        device.TWIS0.enable.write(|w| w.enable().disabled());
        device.TWI0.enable.write(|w| w.enable().disabled());

        defmt::println!("Initialize the SPI instance and CanDriver for Custom PCB!");
        let pins = nrf52840_hal::spi::Pins {
            sck: Some(port0.p0_23.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_22.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_21.into_floating_input().degrade()),
        };

        let cs_pin = port0.p0_20.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port0.p0_24.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::K125, MODE_0);
        let mut gpiote = Gpiote::new(device.GPIOTE);

        let dummy_data = "dummy".as_bytes();

        let settings = Mcp2515Settings::default(); // same as below

        const CLKEN: bool = true;
        const OSM: bool = false;
        const ABAT: bool = false;

        const MASK_RXN: u16 = 0b1111_1111_1110_0000;
        const FILTER_RX0: u16 = 0x0;
        const FILTER_RX1: u16 = 0x1;
        const DEFAULT_FILTER_MASK: u16 = Mcp2515Settings::DEFAULT_FILTER_MASK;

        let canctrl_settings = SettingsCanCtrl::new(
            OperationTypes::Configuration,
            CLKEN,
            CLKPRE::DIV1,
            ABAT,
            OSM,
        );

        let can_settings = Mcp2515Settings::new(
            canctrl_settings,
            McpClock::MCP8,
            Bitrate::CAN125,
            0u8,
            ReceiveBufferMode::OnlyStandardId,
            AcceptanceFilterMask::new(DEFAULT_FILTER_MASK, DEFAULT_FILTER_MASK),
            AcceptanceFilterMask::new(DEFAULT_FILTER_MASK, DEFAULT_FILTER_MASK),
        );

        let mut can_driver = Mcp2515Driver::init(spi, cs_pin, can_interrupt, can_settings);

        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        let dummy_id = StandardId::new(0x2).unwrap();
        //defmt::info!("dummy_data: {:?}", dummy_data.len());

        let mut sender = Sender::new();
        sender.set_left_motor(1.0).unwrap();
        
        let mut msg = sender.dequeue().unwrap();
        msg.print_frame();
        can_driver.transmit(&msg);

        (Shared { gpiote }, Local {
            candriver: can_driver,
            sender,
        })
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote], local = [candriver, sender])]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        let handle = cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred - Custom PCB Node [channel 0]");
                let interrupt_type = cx.local.candriver.interrupt_decode().unwrap();
                cx.local.candriver.handle_interrupt(interrupt_type);

                if (cx.local.candriver.interrupt_is_cleared()) {
                    defmt::info!("All CAN interrupt has been handled!");
                    gpiote.channel0().reset_events();
                    //gpiote.reset_events(); //Execute when all Events are
                    // handled.
                }
                defmt::println!("\n");
            }

            //gpiote.reset_events();
        });
    }
}

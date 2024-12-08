#![no_std]
#![no_main]
#![allow(unused)]

use controller as _;
use can_mcp2515::drivers::can::*;
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use panic_probe as _;
use rtic::app;
use rtic_monotonics::nrf_rtc0_monotonic; // global logger + panicking-behavior + memory layout
nrf_rtc0_monotonic!(Mono);

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC0])]
mod app {

    use can_mcp2515::drivers::can::*;
    use controller::{
        boards::*,
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

    #[shared]
    struct Shared {
        gpiote: Gpiote,
    }

    #[local]
    struct Local {
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        candriver_node: Mcp2515Driver<Spi<SPI2>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
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
        //device.SPI1.enable.write(|w| w.enable().disabled());
        device.SPIM1.enable.write(|w| w.enable().disabled());
        device.SPIS1.enable.write(|w| w.enable().disabled());
        device.TWIM1.enable.write(|w| w.enable().disabled());
        device.TWIS1.enable.write(|w| w.enable().disabled());
        device.TWI1.enable.write(|w| w.enable().disabled());

        device.SPIM0.enable.write(|w| w.enable().disabled());
        device.SPIS0.enable.write(|w| w.enable().disabled());
        device.TWIM0.enable.write(|w| w.enable().disabled());
        device.TWIS0.enable.write(|w| w.enable().disabled());
        device.TWI0.enable.write(|w| w.enable().disabled());

        //let pin_mapping = PinMapping::new(port0, port1);
        //let (pin_map, spi) =pin_mapping.can(device.SPIM1);

        defmt::println!("Initialize the SPI instance, and CanDriver");
        let pins = nrf52840_hal::spi::Pins {
            sck: Some(port0.p0_05.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_02.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_28.into_floating_input().degrade()),
        };

        let pins_node = nrf52840_hal::spi::Pins {
            sck: Some(port0.p0_04.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_11.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_08.into_floating_input().degrade()),
        };

        let cs_pin = port1.p1_02.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port1.p1_15.into_pullup_input().degrade();

        let cs_node_pin = port1.p1_08.into_push_pull_output(Level::High).degrade();
        let can_node_interrupt = port1.p1_07.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::K125, MODE_0);
        let mut spi_node = Spi::new(device.SPI2, pins_node, Frequency::K125, MODE_0);
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
        let mut can_node =
            Mcp2515Driver::init(spi_node, cs_node_pin, can_node_interrupt, can_settings);

        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        gpiote
            .channel1()
            .input_pin(&can_node.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        defmt::println!("After initializing Spi<SPI1>...");
        let dummy_id = StandardId::new(0x2).unwrap();
        defmt::info!("dummy_data: {:?}", dummy_data.len());

        let mut sender = Sender::new();
        sender.set_left_motor(1.0).unwrap();
        let mut msg = sender.dequeue().unwrap();
        msg.print_frame();
        can_driver.transmit(&msg);

        //let mut frame =
        //   CanMessage::new(embedded_can::Id::Standard(dummy_id), &[0x01, 0x02,
        // 0x03]).unwrap();

        //can_driver.loopback_test(frame);
        //can_driver.transmit(&frame);
        //can_node.transmit(&frame);
        //can_node.loopback_test(frame);
        (Shared { gpiote }, Local {
            candriver: can_driver,
            candriver_node: can_node,
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

    #[task(binds = GPIOTE, shared = [gpiote], local = [candriver, candriver_node, sender])]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        //let can_interrupt::LocalResources {candriver, ..} = cx.local;
        
        cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 0] - Can Master!");
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
            if (gpiote.channel1().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 1] - Can Node!");
                let interrupt_type = cx.local.candriver_node.interrupt_decode().unwrap();
                cx.local.candriver_node.handle_interrupt(interrupt_type);

                if (cx.local.candriver_node.interrupt_is_cleared()) {
                    gpiote.channel1().reset_events();
                }
                defmt::println!("\n");
            }

            //gpiote.reset_events();
        });
    }
}

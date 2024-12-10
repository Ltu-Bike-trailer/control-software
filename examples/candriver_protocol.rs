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
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC1, TIMER0, TIMER1])]
mod app {

    use controller::{
        boards::*,
        drivers::{can::{
            AcceptanceFilterMask,
            Bitrate,
            InterruptFlagCode,
            Mcp2515Driver,
            Mcp2515Settings,
            McpClock,
            OperationTypes,
            ReceiveBufferMode,
            SettingsCanCtrl,
            CLKPRE,
            RXBN,
        }, message::CanMessage},
    };
    use cortex_m::asm;
    use embedded_can::{blocking::Can, Frame, StandardId};
    use embedded_hal::{delay::DelayNs, digital::OutputPin, spi::SpiBus};
    use lib::protocol::{self, constants::*, sender::Sender};
    use nrf52840_hal::{
        gpio::{self, Floating, Input, Level, Output, Pin, Port, PullUp, PushPull},
        gpiote::{Gpiote, GpioteInputPin},
        pac::{Interrupt, GPIOTE, SPI0, SPI1, SPI2, SPIM0, SPIM1, SPIM2},
        spi::{Frequency, Spi},
        spim,
        spim::*,
        Clocks,
    };

    use rtic_monotonics::{fugit::{ExtU64, ExtU32}, Monotonic};

    use crate::Mono;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
        sender: Sender<20>,
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        
    }

    #[local]
    struct Local {
        //candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);
        let t = nrf52840_hal::pac::Peripherals::take();
        
        let clk = Clocks::new(device.CLOCK).enable_ext_hfosc().start_lfclk();
        Mono::start(device.RTC0);
        
        /* Disable shared peripheral addresses */

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

        let cs_pin = port1.p1_02.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port1.p1_15.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::M4, MODE_0);
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

        defmt::println!("After initializing Spi<SPI1>...");
        let dummy_id = StandardId::new(0x2).unwrap();
        defmt::info!("dummy_data: {:?}", dummy_data.len());

        let mut sender = Sender::new();
        sender.set_left_motor(1.0).unwrap();
        //sender.set_theta(value) 
        //let mut msg = sender.dequeue().unwrap();

        //msg.data = dummy_data;
        //msg.print_frame();
        //can_driver.transmit(&msg);
        
        send_updates::spawn().unwrap();
        fetch_data::spawn().unwrap();
        
        //let mut frame =
        //   CanMessage::new(embedded_can::Id::Standard(dummy_id), &[0x01, 0x02,
        // 0x03]).unwrap();

        //can_driver.loopback_test(frame);
       // can_driver.transmit(&frame);

        (Shared { gpiote, sender, candriver: can_driver }, Local {
            
        })
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
            //fetch_data::spawn().unwrap();
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote, sender, candriver], priority = 5)]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        let handle = cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 0] - Can Master!");
                
                cx.shared.candriver.lock(|can_driver|{
                    let interrupt_type = can_driver.interrupt_decode().unwrap();
                    if let Some(frame) = can_driver.handle_interrupt(interrupt_type) {
                        let msg_type = protocol::MessageType::try_from(&frame).unwrap();
                        defmt::info!("Received the frame message type: {:?}", msg_type);
                        //handle_can::spawn(frame).unwrap();
                        //fetch_data::spawn().unwrap();
                        //can_driver.transmit(&msg_frame);
                    }

                    if can_driver.interrupt_is_cleared() {
                        defmt::info!("All CAN interrupt has been handled!");
                        gpiote.channel0().reset_events();
                        
                    }
                });

                defmt::println!("\n");
            }
            if (gpiote.channel1().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 1] - Can Node!");
                defmt::println!("\n");
            }

            //gpiote.reset_events();
        });
    }

    #[task(priority = 1, shared = [candriver, sender])]
    async fn handle_can(mut cx: handle_can::Context, frame: lib::protocol::message::CanMessage){
        let mut msg_frame = frame;
        let msg_type = protocol::MessageType::try_from(&msg_frame).unwrap();
        defmt::info!("Received the frame message type: {:?}", msg_type);

        // This is just for testing and keep sending dummy data.
    }
    #[task(priority = 3, shared = [sender])]
    async fn fetch_data(mut cx: fetch_data::Context){
        // This is just for testing and keep sending dummy data.
        loop {
            cx.shared.sender.lock(|sender|{
                //sender.set_right_motor(1.0).unwrap();
                sender.set_left_motor(1.3).unwrap();
                sender.set_left_motor(1.3).unwrap();
                sender.set_left_motor(1.2).unwrap();
                sender.set_left_motor(1.1).unwrap();
                sender.set_left_motor(1.1).unwrap();


                sender.set_load_cell_front(1.6).unwrap();
                sender.set_load_cell_front(1.6).unwrap();
                sender.set_load_cell_front(1.5).unwrap();
                sender.set_load_cell_front(1.5).unwrap();


                sender.set_load_cell_bed(1.1);
                sender.set_load_cell_bed(1.1);
                sender.set_load_cell_bed(1.2);


                sender.set_theta(1.2_f32);
                //sender.enqueue(protocol::MessageType::FixedLog(protocol::FixedLogType::BatteryStatus(protocol::BatteryStatus(0.5))));
                //sender.set_theta(2.4_f32);
            });
            Mono::delay(400u32.millis().into()).await;            
            
        }
    }
    
    #[task(priority = 4, shared = [sender, candriver])]
    /// This should enqueue on the Shared `Sender` whenever a 
    /// sensor reads a new value.
    async fn send_updates(mut cx: send_updates::Context){
        // Loop and dequeue, and transmit on the can bus.
        loop {
            cx.shared.sender.lock(|sender|{
                if let Some(mut data) = sender.dequeue(){
                    // Value was found in buffer [TRANSMIT OVER CAN]:
                    cx.shared.candriver.lock(|can|{
                        data.print_frame();
                        can.transmit(&data);
                    });
                    
                } 

            });
            Mono::delay(100u32.millis().into()).await;            

        }
    }
}

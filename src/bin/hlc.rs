#![no_std]
#![no_main]
#![allow(unused)]

use controller as _;
use defmt_rtt as _;
use defmt::*;
use nrf52840_hal as _;
use panic_probe as _;
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);


#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC1, TIMER0, TIMER1])]
mod hlc {

    use can_mcp2515::drivers::{can::*, message::CanMessage};
    use controller::{
        boards::*,
    };
    use cortex_m::asm;
    use defmt::Debug2Format;
    use embedded_can::{blocking::Can, StandardId};
    use embedded_hal::*;
    use lib::protocol::{sender::Sender, constants::*};
    use nrf52840_hal::{
        gpio::{self, Input, Level, Output, Pin, PullUp, PushPull},
        gpiote::Gpiote,
        pac::SPI0,
        spi::{Frequency, Spi},
        spim::*,
        Clocks,
    };
   
    use rtic_monotonics::{fugit::ExtU32, Monotonic};
    use crate::Mono;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        sender: Sender<10>,
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

        let mut spi = Spi::new(device.SPI0, pins, Frequency::K125, MODE_0);
        let mut gpiote = Gpiote::new(device.GPIOTE);


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

        let can_settings = Mcp2515Settings::default();
        let mut can_driver = Mcp2515Driver::init(spi, cs_pin, can_interrupt, can_settings);

        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        defmt::println!("After initializing Spi<SPI1>...");
        let dummy_id = StandardId::new(0x2).unwrap();

        let mut sender = Sender::new();
        //sender.set_left_motor(1.0).unwrap();

        send_updates::spawn().unwrap();
        fetch_data::spawn().unwrap();

        //let mut frame =
        //   CanMessage::new(embedded_can::Id::Standard(dummy_id), &[0x01, 0x02,
        // 0x03]).unwrap();

        //can_driver.transmit(&frame);

        (Shared { gpiote, sender, candriver: can_driver }, Local {
            
        })
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
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
                        let mut msg_frame = frame;
                        let std_id = StandardId::new(msg_frame.id_raw()).unwrap();
                        let recieved_id = Message::try_from(std_id).unwrap();

                        defmt::info!("Received the frame message type: {:?}", Debug2Format(&recieved_id));
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
    async fn handle_can(mut cx: handle_can::Context, frame: CanMessage){
        let mut msg_frame = frame;
        let std_id = StandardId::new(msg_frame.id_raw()).unwrap();
        let recieved_id = Message::try_from(std_id).unwrap();

        defmt::info!("Received the frame message type: {:?}", Debug2Format(&recieved_id));
    }

    #[task(priority = 4, shared = [sender])]
    async fn fetch_data(mut cx: fetch_data::Context){
        // This is just for testing and keep sending dummy data.
        loop {
            cx.shared.sender.lock(|sender|{
                sender.set_theta(1.2_f32);
                sender.set_theta(2.4_f32);
            });
            Mono::delay(500u32.millis().into()).await;            
        }
    }
    
    #[task(priority = 3, shared = [sender, candriver])]
    /// This should enqueue on the Shared `Sender` whenever a 
    /// sensor reads a new value.
    async fn send_updates(mut cx: send_updates::Context){
        // Loop and dequeue, and transmit on the can bus.
        loop {
            cx.shared.sender.lock(|sender|{
                if let Some(data) = sender.dequeue(){
                    // Value was found in buffer [TRANSMIT OVER CAN]:
                    cx.shared.candriver.lock(|can|{
                        can.transmit(&data);
                    });
                } 
            });
            //Mono::delay(100u32.millis().into()).await;            

        }
    }
}


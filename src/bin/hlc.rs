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
        hlc_utils::{config::*, core::*, events::*},
    };
    use cortex_m::asm;
    use defmt::Debug2Format;
    use embedded_can::{blocking::Can, StandardId};
    use embedded_hal::*;
    use lib::protocol::{constants::*, sender::Sender, MessageType};
    use nrf52840_hal::{
        gpio::{self, Input, Level, Output, Pin, PullUp, PushPull},
        gpiote::Gpiote,
        pac::SPI0,
        spi::{Frequency, Spi},
        spim::*,
        Clocks,
    };
   
    use rtic_monotonics::{fugit::ExtU32, Monotonic};
    use rtic_sync::{channel::{self, *}, make_channel};
    use crate::Mono;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
        sender: Sender<10>,
    }

    #[local]
    struct Local {
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        can_sender: channel::Sender<'static, CanMessage, 10>,
        can_receiver: channel::Receiver<'static, CanMessage, 10>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);
        
        let clk = Clocks::new(device.CLOCK).enable_ext_hfosc().start_lfclk();
        Mono::start(device.RTC0);

        defmt::println!("Initialize the SPI instance, and CanDriver");

        let mut can_manager = CanManager::new(port0, port1);
        let (cs_pin, int_pin, spi, gpiote) = can_manager.peripheral_instances(device.SPI0, device.GPIOTE);

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
        let mut can_driver = Mcp2515Driver::init(spi, cs_pin, int_pin, can_settings);
        
        let (send, receive) = make_channel!(CanMessage, 10);
        let mut sender = Sender::new();

        (Shared { gpiote, sender }, Local {
            candriver: can_driver, can_sender: send, can_receiver: receive,
        })
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote, sender], local = [candriver, can_sender], priority = 5)]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        let can_interrupt::LocalResources {candriver, ..} = cx.local;
        let handle = cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 0] - Can Master!");
                
                let interrupt_type = candriver.interrupt_decode().unwrap();
                if let Some(frame) = candriver.handle_interrupt(interrupt_type) {
                    //let mut msg_frame = frame;
                    //let std_id = StandardId::new(msg_frame.id_raw()).unwrap();
                    //let recieved_id = Message::try_from(std_id).unwrap();
                    cx.local.can_sender.try_send(frame);

                   // defmt::info!("Received the frame message type: {:?}", Debug2Format(&recieved_id));
                }
                if candriver.interrupt_is_cleared() {
                    defmt::info!("All CAN interrupt has been handled!");
                    gpiote.channel0().reset_events();
                }
                defmt::println!("\n");
            }
            if (gpiote.channel1().is_event_triggered()) {
                
            }
        });
    }

    /// Should process the MessageType channel, and distribute to specific 
    /// task handlers, depending on the message type being read.  
    #[task(priority = 1, shared = [sender], local = [can_receiver])]
    async fn process_input(mut cx: process_input::Context){
        while let Ok(frame) = cx.local.can_receiver.recv().await {
            let msg_type = match MessageType::try_from(&frame){
                Ok(msg) => msg,
                Err(_) => continue,
            };
            // Distribute the received CAN frame / MessageTypes below:  
        }
        //defmt::info!("Received the frame message type: {:?}", Debug2Format(&recieved_id));
    }

    #[task(priority = 3, shared = [sender])]
    async fn send_updates(mut cx: send_updates::Context){
        Mono::delay(500u32.millis().into()).await;            
        
    } 
}


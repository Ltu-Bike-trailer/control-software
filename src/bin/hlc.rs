#![no_std]
#![no_main]
#![allow(unused)]
<<<<<<< HEAD
<<<<<<< HEAD
#![feature(generic_arg_infer)]
#![allow(clippy::field_reassign_with_default)]

use controller as _;
use defmt::*;
use defmt_rtt as _;
=======
=======
#![feature(generic_arg_infer)]
>>>>>>> d3bcbce (rebasing with main)

use controller as _;
use defmt::*;
<<<<<<< HEAD
>>>>>>> 86eed1f (rebasing with main)
=======
use defmt_rtt as _;
>>>>>>> d3bcbce (rebasing with main)
use nrf52840_hal as _;
use panic_probe as _;
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

<<<<<<< HEAD
<<<<<<< HEAD
=======

>>>>>>> 86eed1f (rebasing with main)
=======
>>>>>>> d3bcbce (rebasing with main)
#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC1, TIMER0, TIMER1])]
mod hlc {

    use can_mcp2515::drivers::{can::*, message::CanMessage};
    use controller::{
        boards::*,
<<<<<<< HEAD
<<<<<<< HEAD
        drivers::hx711::{Gain, Hx711Driver, ValidTimings},
<<<<<<< HEAD
        hlc_utils::{config::*, core::*, events::*, stype_calibration::*},
=======
        hlc_utils::{stype_calibration::*, core::*, config::*, events::*},
>>>>>>> 6f406d9 (rebasing with main)
    };
    use cortex_m::asm;
    use defmt::Debug2Format;
    use delay::DelayNs;
    use embedded_can::{blocking::Can, StandardId};
    use embedded_hal::*;
    use lib::protocol::{constants::*, sender::Sender, MessageType};
    use nrf52840_hal::{
        gpio::{
            self,
            p0::P0_04,
            Disconnected,
            Floating,
            Input,
            Level,
            Output,
            Pin,
            PullUp,
            PushPull,
        },
        gpiote::{self, Gpiote},
        pac::SPI0,
        pwm::{Channel, Pwm},
        saadc::{self, SaadcConfig, SaadcTask, Time},
        spi::{Frequency, Spi},
        spim::*,
        time::U32Ext,
        Clocks,
    };
    use rtic_monotonics::{
        fugit::{ExtU32, ExtU64},
        Monotonic,
    };
    use rtic_sync::{
        channel::{self, *},
        make_channel,
    };

=======
=======
        drivers::hx711::{Gain, Hx711Driver, ValidTimings},
        hlc_utils::stype_calibration::*,
>>>>>>> d3bcbce (rebasing with main)
    };
    use cortex_m::asm;
    use defmt::Debug2Format;
    use delay::DelayNs;
    use embedded_can::{blocking::Can, StandardId};
    use embedded_hal::*;
    use lib::protocol::{sender::Sender, constants::*};
    use nrf52840_hal::{
        gpio::{
            self,
            p0::P0_04,
            Disconnected,
            Floating,
            Input,
            Level,
            Output,
            Pin,
            PullUp,
            PushPull,
        },
        gpiote::{self, Gpiote},
        pac::SPI0,
        pwm::{Channel, Pwm},
        saadc::{self, SaadcConfig, SaadcTask, Time},
        spi::{Frequency, Spi},
        spim::*,
        time::U32Ext,
        Clocks,
    };
<<<<<<< HEAD
   
    use rtic_monotonics::{fugit::ExtU32, Monotonic};
>>>>>>> 86eed1f (rebasing with main)
=======
    use rtic_monotonics::{
        fugit::{ExtU32, ExtU64},
        Monotonic,
    };
    use rtic_sync::{
        channel::{self, *},
        make_channel,
    };

>>>>>>> d3bcbce (rebasing with main)
    use crate::Mono;

    #[shared]
    struct Shared {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> d3bcbce (rebasing with main)
        cargo_weight: f32,
        s_type_force: f32,
        gpiote: Gpiote,
=======
        gpiote: Gpiote,
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
>>>>>>> 86eed1f (rebasing with main)
        sender: Sender<10>,
    }

    #[local]
    struct Local {
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> d3bcbce (rebasing with main)
        stype: SaadcTask<1>,
        hx711: Hx711Driver<Pin<Output<PushPull>>, Pin<Input<Floating>>>,
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        can_sender: channel::Sender<'static, Option<CanMessage>, 10>,
        control_reference_sender: channel::Sender<'static, Option<CanMessage>, 10>,
        can_receiver: channel::Receiver<'static, Option<CanMessage>, 10>,
        hlc_controller: Controller,
=======
        //candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
>>>>>>> 86eed1f (rebasing with main)
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);
<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> d3bcbce (rebasing with main)

        let clk = Clocks::new(device.CLOCK).enable_ext_hfosc().start_lfclk();
        Mono::start(device.RTC0);
=======
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
>>>>>>> 86eed1f (rebasing with main)

        defmt::println!("Initialize the SPI instance, and CanDriver");
        let pins = nrf52840_hal::spi::Pins {
            sck: Some(port0.p0_05.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_02.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_28.into_floating_input().degrade()),
        };

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
        //let can_manager = CanManager::new(port0, port1);
        //let (cs_pin, int_pin, spi, gpiote) =
        // can_manager.peripheral_instances(device.SPI0, device.GPIOTE);
=======
        let cs_pin = port1.p1_02.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port1.p1_15.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::K125, MODE_0);
        let mut gpiote = Gpiote::new(device.GPIOTE);
>>>>>>> d3bcbce (rebasing with main)

        let cs_pin = port1.p1_02.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port1.p1_15.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::M1, MODE_0);
        let mut gpiote = Gpiote::new(device.GPIOTE);

=======
=======
        //let mut can_manager = CanManager::new(port0, port1);
        //let (cs_pin, int_pin, spi, gpiote) = can_manager.peripheral_instances(device.SPI0, device.GPIOTE);

>>>>>>> 6f406d9 (rebasing with main)
        let cs_pin = port1.p1_02.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port1.p1_15.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::M1, MODE_0);
        let mut gpiote = Gpiote::new(device.GPIOTE);


>>>>>>> 86eed1f (rebasing with main)
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

<<<<<<< HEAD
<<<<<<< HEAD
        let mut can_settings = Mcp2515Settings::default().enable_interrupts(&[
            CanInte::MERRE,
            CanInte::ERRIE,
            CanInte::TX2IE,
            CanInte::TX1IE,
            CanInte::TX0IE,
            CanInte::RX1IE,
            CanInte::RX0IE,
        ]);

=======
        let can_settings = Mcp2515Settings::default();
>>>>>>> 86eed1f (rebasing with main)
        let mut can_driver = Mcp2515Driver::init(spi, cs_pin, can_interrupt, can_settings);

        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

<<<<<<< HEAD
        let (send, receive) = make_channel!(Option<CanMessage>, 10);
        let control_reference_sender = send.clone();
        let mut sender = Sender::new();
        //sender.set_left_motor(1.0).unwrap();

=======
        let can_settings = Mcp2515Settings::default();
        let mut can_driver = Mcp2515Driver::init(spi, cs_pin, can_interrupt, can_settings);

        let (send, receive) = make_channel!(CanMessage, 10);
        let mut sender = Sender::new();
        //sender.set_left_motor(1.0).unwrap();

>>>>>>> d3bcbce (rebasing with main)
        // HX711
        // "9","P1.09/TRACEDATA3","HX711 Data"
        let dout_pin = port1.p1_09.into_floating_input().degrade();
        // "10","P0.11/TRACEDATA2","HX711 SCK"
        let pd_sck = port0.p0_11.into_push_pull_output(Level::Low).degrade();

        let mut hx711_instance =
            Hx711Driver::init(pd_sck, dout_pin, Gain::Apply128, ValidTimings::default());

        // S-Type Load cell
        // "4","P0.04/AIN2","S-Type Load Cell Amplifier"
        // ADC configuration.
        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_40US;
        cfg.gain = adc_gain();
        cfg.oversample = saadc::Oversample::OVER256X;
        cfg.reference = saadc::Reference::INTERNAL;

        // The input pin.
        let input = port0.p0_04;

        //This is probably wrong is some way taken directly from S-type calibration
        // and modified
        let mut adc: SaadcTask<1> = SaadcTask::new(
            device.SAADC,
            cfg,
            &[<P0_04<Disconnected> as saadc::Channel>::channel()],
            [0],
        );

<<<<<<< HEAD
        let control_timer = nrf52840_hal::timer::Timer::new(device.TIMER2);
        let mut control_timer = control_timer.into_periodic();
        control_timer.timeout::<1, 1000>(25u64.millis());
        //control_timer.start(25u32.millis());

        //Start sampeling the stype
        adc.start_sample();
        read_hx711::spawn();
=======
        defmt::println!("After initializing Spi<SPI1>...");
        let dummy_id = StandardId::new(0x2).unwrap();

        let mut sender = Sender::new();
        //sender.set_left_motor(1.0).unwrap();

        send_updates::spawn().unwrap();
        fetch_data::spawn().unwrap();
>>>>>>> 86eed1f (rebasing with main)
=======
        //Start sampeling the stype
        adc.start_sample();
        read_hx711::spawn();

        send_updates::spawn().unwrap();
>>>>>>> d3bcbce (rebasing with main)

        //let mut frame =
        //   CanMessage::new(embedded_can::Id::Standard(dummy_id), &[0x01, 0x02,
        // 0x03]).unwrap();

<<<<<<< HEAD
<<<<<<< HEAD
=======
>>>>>>> d3bcbce (rebasing with main)
        (
            Shared {
                gpiote,
                sender,
                cargo_weight: f32::NEG_INFINITY,
                s_type_force: f32::NEG_INFINITY,
            },
            Local {
                stype: adc,
                hx711: hx711_instance,
                candriver: can_driver,
                can_sender: send,
                can_receiver: receive,
<<<<<<< HEAD
                hlc_controller: Controller::new(),
                control_reference_sender,
            },
        )
=======
        //can_driver.transmit(&frame);

        (Shared { gpiote, sender, candriver: can_driver }, Local {
            
        })
>>>>>>> 86eed1f (rebasing with main)
=======
            },
        )
>>>>>>> d3bcbce (rebasing with main)
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }

<<<<<<< HEAD
    #[task(binds = GPIOTE, shared = [gpiote, sender], local = [can_sender], priority = 4)]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 0] - Can Master!");

                let _ = cx.local.can_sender.try_send(None);
                gpiote.channel0().reset_events();

                //if candriver.interrupt_is_cleared() {
                //    defmt::info!("All CAN interrupt has been handled!");
                //    gpiote.channel0().reset_events();
                //}
                defmt::println!("\n");
            }
            if (gpiote.channel1().is_event_triggered()) {}
        });
    }

    #[task(priority = 3, local = [candriver, can_receiver])]
    async fn handle_can(mut cx: handle_can::Context) {
        //let can_interrupt::LocalResources { candriver, .. } = cx.local;
        while let Ok(frame) = cx.local.can_receiver.recv().await {
            let mut manager = cx.local.candriver.interrupt_manager();
            let mut received_message = None;
            while let Some(event) = manager.next() {
                if let Some(frame) = event.handle() {
                    received_message = Some(frame);
                    let msg_type = match MessageType::try_from(&frame) {
                        Ok(msg) => msg,
                        Err(_) => continue,
                    };
                }
            }
            if let Some(frame) = frame {
                cx.local.candriver.transmit(&frame);
            }
        }
    }

    /// Should process the MessageType channel, and distribute to specific
    /// task handlers, depending on the message type being read.  
    //#[task(priority = 1, shared = [sender], local = [can_receiver])]
    //async fn process_input(mut cx: process_input::Context) {
    //    while let Ok(frame) = cx.local.can_receiver.recv().await {
    //        let msg_type = match MessageType::try_from(&frame) {
    //            Ok(msg) => msg,
    //            Err(_) => continue,
    //        };
    //        // Distribute the received CAN frame / MessageTypes below:
    //    }
    //    //defmt::info!("Received the frame message type: {:?}",
    //    // Debug2Format(&recieved_id));
    //}

    ///High priority constant polling of S-Type loadcell
    #[task(binds = SAADC, local=[stype], shared =[s_type_force], priority=2)]
    fn read_stype(mut cx: read_stype::Context) {
        let [sample] = cx.local.stype.complete_sample(conv);

        // Scale in between.
        const GAIN: f32 = 10.;
        let converted = GAIN * sample;

        cx.shared.s_type_force.lock(|f| *f = converted);
        defmt::trace!("Measured {}N", converted);

        cx.local.stype.start_sample();
    }

    ///Lowest priority task. Expected to be static during operation but needs
    /// to be polled occasionally to keep a consistent value
    #[task(local = [hx711], shared = [cargo_weight], priority = 1)]
    async fn read_hx711(mut cx: read_hx711::Context) {
        loop {
            let data = cx
                .local
                .hx711
                .read_full_period::<Mono, _, _>(Gain::Apply128)
                .await;
            let decoded_data = cx.local.hx711.decode_data(data);

            defmt::trace!("Decoded Data: {:?}", decoded_data.data_out);

            // Write the data to the shared variable
            cx.shared
                .cargo_weight
                .lock(|cw| *cw = decoded_data.data_out as f32);

            //Delay to only read every few seconds to allow for wfi();
            Mono::delay_ms(&mut Mono, 5000);
        }
    }

    // TODO: - Change to hardware timer interrupt, not async software!
    #[task(binds = TIMER2, local = [hlc_controller, control_reference_sender], shared = [s_type_force], priority = 5)]
    fn controller_output(mut cx: controller_output::Context) {
        cx.shared.s_type_force.lock(|input_force| {
            while let Some(actuate) = cx.local.hlc_controller.actuate(*input_force) {}
        });
    }
}
=======
    #[task(binds = GPIOTE, shared = [gpiote, sender, candriver], priority = 5)]
    fn can_interrupt(mut cx: can_interrupt::Context) {
<<<<<<< HEAD
=======
        let can_interrupt::LocalResources { candriver, .. } = cx.local;
>>>>>>> d3bcbce (rebasing with main)
        let handle = cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 0] - Can Master!");
<<<<<<< HEAD
                
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

>>>>>>> 86eed1f (rebasing with main)
=======

                let interrupt_type = candriver.interrupt_decode().unwrap();
                if let Some(frame) = candriver.handle_interrupt(interrupt_type) {
                    //let mut msg_frame = frame;
                    //let std_id = StandardId::new(msg_frame.id_raw()).unwrap();
                    //let recieved_id = Message::try_from(std_id).unwrap();
                    cx.local.can_sender.try_send(frame);

                    // defmt::info!("Received the frame message type: {:?}",
                    // Debug2Format(&recieved_id));
                }
                if candriver.interrupt_is_cleared() {
                    defmt::info!("All CAN interrupt has been handled!");
                    gpiote.channel0().reset_events();
                }
                defmt::println!("\n");
            }
            if (gpiote.channel1().is_event_triggered()) {}
        });
    }

    /// Should process the MessageType channel, and distribute to specific
    /// task handlers, depending on the message type being read.  
    #[task(priority = 1, shared = [sender], local = [can_receiver])]
    async fn process_input(mut cx: process_input::Context) {
        while let Ok(frame) = cx.local.can_receiver.recv().await {
            let msg_type = match MessageType::try_from(&frame) {
                Ok(msg) => msg,
                Err(_) => continue,
            };
            // Distribute the received CAN frame / MessageTypes below:
        }
        //defmt::info!("Received the frame message type: {:?}",
        // Debug2Format(&recieved_id));
    }

    #[task(priority = 3, shared = [sender])]
    async fn send_updates(mut cx: send_updates::Context) {
        Mono::delay(500u32.millis().into()).await;
    }

    ///High priority constant polling of S-Type loadcell
    #[task(binds = SAADC, local=[stype], shared =[s_type_force], priority=2)]
    fn read_stype(mut cx: read_stype::Context) {
        let [sample] = cx.local.stype.complete_sample(conv);

        // Scale in between.
        const GAIN: f32 = 10.;
        let converted = GAIN * sample;

        cx.shared.s_type_force.lock(|f| *f = converted);
        defmt::trace!("Measured {}N", converted);

        cx.local.stype.start_sample();
    }

    ///Lowest priority task. Expected to be static during operation but needs
    /// to be polled occasionally to keep a consistent value
    #[task(local = [hx711], shared = [cargo_weight], priority = 1)]
    async fn read_hx711(mut cx: read_hx711::Context) {
        loop {
            let data = cx
                .local
                .hx711
                .read_full_period::<Mono, _, _>(Gain::Apply128)
                .await;
            let decoded_data = cx.local.hx711.decode_data(data);

            defmt::trace!("Decoded Data: {:?}", decoded_data.data_out);

            // Write the data to the shared variable
            cx.shared
                .cargo_weight
                .lock(|cw| *cw = decoded_data.data_out as f32);

            //Delay to only read every few seconds to allow for wfi();
            Mono::delay_ms(&mut Mono, 5000);
        }
    }
}
>>>>>>> d3bcbce (rebasing with main)

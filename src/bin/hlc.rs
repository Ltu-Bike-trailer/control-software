#![no_std]
#![no_main]
#![allow(unused)]
#![feature(generic_arg_infer)]
#![allow(clippy::field_reassign_with_default)]

use controller as _;
use defmt_rtt as _;
use nrf52840_hal as _;
use panic_probe as _;
use rtic_monotonics::nrf::timer::prelude::*;
nrf_timer4_monotonic!(Mono, 16_000_000);

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [TIMER0, TIMER1, TIMER2])]
mod hlc {


    use can_mcp2515::drivers::{can::*, message::CanMessage};
    use controller::{
        boards::*,
        drivers::hx711::{Driver, Gain, ValidTimings},
        hlc_utils::{config::*, core::*, events::*, stype_calibration::*},
    };
    use cortex_m::asm;
    use defmt::{panic, Debug2Format};
    use delay::DelayNs;
    use embedded_can::{blocking::Can, StandardId};
    use embedded_hal::*;
    use lib::protocol::{constants::*, sender::Sender, MessageType, WriteType};
    use nrf52840_hal::{
        gpio::{
            self, p0::P0_04, Disconnected, Floating, Input, Level, Output, Pin, PullDown, PullUp, PushPull
        },
        gpiote::{self, Gpiote},
        pac::{RTC0, SPI0, TIMER2, TIMER3},
        pwm::{Channel, Prescaler, Pwm},
        rtc::{RtcCompareReg, RtcInterrupt},
        saadc::{self, SaadcConfig, SaadcTask, Time},
        spi::{Frequency, Spi},
        spim::*,
        time::U32Ext,
        timer::Periodic,
        Clocks,
        Rtc,
        Timer,
    };
    use rtic_monotonics::{
        fugit::{ExtU32, ExtU64},
        rtic_time::monotonic::TimerQueueBasedInstant,
        Monotonic,
    };
    use rtic_sync::{
        channel::{self, *},
        make_channel,
    };

    use crate::Mono;

    #[shared]
    struct Shared {
        cargo_weight: f32,
        s_type_force: f32,
        gpiote: Gpiote,
        sender: Sender<10>,
        stype: SaadcTask<1>,
        can_rx: bool,
    }

    #[local]
    struct Local {
        hx711: Driver<Pin<Output<PushPull>>, Pin<Input<Floating>>>,
        candriver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        can_sender: channel::Sender<'static, Option<CanMessage>, 100>,
        control_reference_sender: channel::Sender<'static, Option<CanMessage>, 100>,
        can_receiver: channel::Receiver<'static, Option<CanMessage>, 100>,
        hlc_controller: Controller,
        control_timer: Timer<TIMER3>,
        can_thing: Rtc<RTC0>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);

        //let t = nrf52840_hal::pac::Peripherals::take();
        let clk = Clocks::new(device.CLOCK).start_lfclk().enable_ext_hfosc();
        Mono::start(device.TIMER4);

        defmt::println!("Initialize the SPI instance, and CanDriver");
        let pins = nrf52840_hal::spi::Pins {
            sck: Some(port0.p0_05.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_02.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_20.into_floating_input().degrade()),
        };

        //let can_manager = CanManager::new(port0, port1);
        //let (cs_pin, int_pin, spi, gpiote) =
        // can_manager.peripheral_instances(device.SPI0, device.GPIOTE);
        let cs_pin = port1.p1_08.into_push_pull_output(Level::High).degrade();
        let can_interrupt = port0.p0_28.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI0, pins, Frequency::M2, MODE_0);
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

        
        let mut can_settings = Mcp2515Settings::default().enable_interrupts(&[
            CanInte::RX1IE,
            CanInte::RX0IE,
        ]);

        can_settings.filter_b0(DEFAULT_FILTER_MASK, DEFAULT_FILTER_MASK);
        can_settings.filter_b1(DEFAULT_FILTER_MASK, DEFAULT_FILTER_MASK);

        defmt::info!("Interrupts: {:#08b}", can_settings.interrupts);

        let mut can_driver = Mcp2515Driver::init(spi, cs_pin, can_interrupt, can_settings);
       
        
        if true {
        match can_driver.read_caninte() {
            Ok(val) => defmt::info!("Caninte {:#08b}",val),
            Err(_)=> panic!("Caninte read failed"),
        }
        }
        
        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        let (send, receive) = make_channel!(Option<CanMessage>, 100);
        let control_reference_sender = send.clone();
        let mut sender = Sender::new();

        // HX711
        // "9","P1.09/TRACEDATA3","HX711 Data"
        let dout_pin = port1.p1_09.into_floating_input().degrade();
        // "10","P0.11/TRACEDATA2","HX711 SCK"
        let pd_sck = port0.p0_11.into_push_pull_output(Level::Low).degrade();

        let mut hx711_instance =
            Driver::init(pd_sck, dout_pin, Gain::Apply128, ValidTimings::default());

        // S-Type Load cell
        // "4","P0.04/AIN2","S-Type Load Cell Amplifier"
        // ADC configuration.
        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_40US;
        cfg.gain = adc_gain();
        cfg.oversample = saadc::Oversample::OVER256X;
        cfg.reference = saadc::Reference::INTERNAL;

        // The input pin.
        //let input = port0.p0_04;

        //This is probably wrong is some way taken directly from S-type calibration
        // and modified
        let mut adc: SaadcTask<1> = SaadcTask::new(
            device.SAADC,
            cfg,
            &[<P0_04<Disconnected> as saadc::Channel>::channel()],
            [0],
        );

        let mut control_timer = nrf52840_hal::timer::Timer::new(device.TIMER3);
        control_timer.enable_interrupt();
        let mut control_timer = control_timer.into_oneshot();
        control_timer.timeout::<1, 1000>(20u64.millis());

        let mut can_thing = nrf52840_hal::rtc::Rtc::new(device.RTC0, 1).unwrap();
        can_thing.enable_event(RtcInterrupt::Compare0);
        can_thing.enable_interrupt(RtcInterrupt::Compare0, None);
        can_thing.set_compare(RtcCompareReg::Compare0, 100); //655);
        can_thing.enable_counter();
        //control_timer.start(25u32.millis());

        //Start sampeling the stype
        adc.start_sample();
        read_hx711::spawn();
        //handle_can::spawn();
        //controller_output::spawn();

        assert!(Mono::now().ticks() != 0);

        (
            Shared {
                gpiote,
                sender,
                cargo_weight: f32::NEG_INFINITY,
                s_type_force: f32::NEG_INFINITY,
                stype: adc,
                can_rx: false,
            },
            Local {
                hx711: hx711_instance,
                candriver: can_driver,
                can_sender: control_reference_sender,
                can_receiver: receive,
                hlc_controller: Controller::new(),
                control_reference_sender: send,
                control_timer,
                can_thing,
            },
        )
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote, sender,can_rx], local = [can_sender], priority = 4)]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        defmt::info!("before lock!");
        cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("\n");
                defmt::info!("GPIOTE interrupt occurred [channel 0] - Can Master!");
                cx.shared.can_rx.lock(|rx| *rx = true);
                let _ = cx.local.can_sender.try_send(None);
                gpiote.channel0().reset_events();

                defmt::println!("\n");
            }
        });
    }

    #[task(binds = RTC0, priority = 3, local = [candriver, can_receiver, can_thing], shared = [sender, can_rx])]
    fn handle_can(mut cx: handle_can::Context) {
        //defmt::warn!("Can frames???");
        cx.local.can_thing.clear_counter();
        cx.local.can_thing.reset_event(RtcInterrupt::Compare0);

        let msg = cx.shared.sender.lock(|sender| sender.dequeue());

        if let Some(msg) = msg {
            //defmt::warn!("SENDING CAN FRAME???");
            //cx.local.candriver.transmit(&msg);
        }

        if cx.shared.can_rx.lock(|rx| {
            let new_rx = *rx;
            *rx = false;
            new_rx
        }) {
        
        
         //if true {
         //       panic!("Can RX!")
         //   }
            defmt::info!("Can RX!");
            //let mut manager = cx.local.candriver.interrupt_manager();
            //let mut received_message = None;
            //while let Some(event) = manager.next() {
            while !cx.local.candriver.interrupt_is_cleared() {
                let interrupt_type = cx.local.candriver.interrupt_decode().unwrap();
                if let Some(frame) = cx.local.candriver.handle_interrupt(interrupt_type) {
                    //let msg_type = MessageType::try_from(&frame);
                    defmt::info!("Frame: {}", frame.clone().data);
                }
                //defmt::warn!("Got event! {:#04b} Some other cool text", event.event_code as u8);
                //if let Some(frame) = event.handle() {
                //    received_message = Some(frame);
                    //defmt::info!("")
                    /*
                    let msg_type = match MessageType::try_from(&frame) {
                        Ok(msg) =>{
                            received_message = Some(msg);
                            break;
                        },
                        Err(_) => continue,
                    };
                    */
                //}
            }
            if true {
                panic!(":()");
            }
        }

        cx.local.can_thing.clear_counter();

        //defmt::info!("hello");
        ////let can_interrupt::LocalResources { candriver, .. } = cx.local;
        //while let Ok(frame) = cx.local.can_receiver.recv().await {
        //    defmt::info!("handle_can entry");
        //    let mut manager = cx.local.candriver.interrupt_manager();
        //    let mut received_message = None;
        //    while let Some(event) = manager.next() {
        //        if let Some(frame) = event.handle() {
        //            received_message = Some(frame);
        //            let msg_type = match MessageType::try_from(&frame) {
        //                Ok(msg) => msg,
        //                Err(_) => continue,
        //            };
        //        }
        //    }
        //    if let Some(frame) = frame {
        //        //defmt::info!("Sending frame: {}", Debug2Format(&frame));
        //        defmt::info!("Sending frame :)");
        //        cx.local.candriver.transmit(&frame);
        //    }
        //}
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
    #[task(binds = SAADC, shared =[s_type_force,stype],local = [buffer:[f32;4] = [0.;4], ptr:usize = 0], priority=2)]
    fn read_stype(mut cx: read_stype::Context) {
        let [sample] = cx.shared.stype.lock(|s_type| s_type.complete_sample(conv));
        if *cx.local.ptr < 4 {
            cx.local.buffer[*cx.local.ptr] = sample;
            cx.shared.stype.lock(|s_type| s_type.start_sample());
            return;
        }
        // AVERAGE SAMLPES
         
        let mut avg_sample: f32 = 0.;
        //let _ = *cx.local.buffer.iter().map(|val| ;  

        //avg_sample = avg_sample / cx.local.buffer.len();
         

        // Scale in between.
        const GAIN: f32 = 50.;
        const OFFSET: f32 = 1.7700;
        const LOADCELL_GAIN: f32 = 0.019984;
        const VOLTAGE_DIV: f32 = 0.68;
        const K: f32 = GAIN * LOADCELL_GAIN * VOLTAGE_DIV;
        const K_NEWTON: f32 = K / (200.0 * 9.82);
        const FORCE_INV: f32 = (1.0 / K_NEWTON);
        let converted: f32 = FORCE_INV * sample - (OFFSET / K_NEWTON);
        let mut converted_avg: f32 = 0.0;
        //let converted = GAIN * sample;

        cx.shared.s_type_force.lock(|f| {
            //converted_avg = (converted + *f) / 2.0 ;
            //*f = (converted + *f ) / 2.0;
            *f = converted;
        });
        *cx.local.ptr = 0;
        defmt::trace!("Measured {}N", converted);
        //cx.shared.stype.lock(|stype| stype.start_sample());
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

    #[task(binds =TIMER3, local = [hlc_controller, control_reference_sender,control_timer], shared = [s_type_force,stype, sender], priority = 5)]
    fn controller_output(mut cx: controller_output::Context) {
        let mut time = Mono::now();
        cx.local.control_timer.reset_event();

        if let Some(actuate) = cx
            .shared
            .s_type_force
            .lock(|input_force| cx.local.hlc_controller.actuate(*input_force))
        {
            //defmt::info!("Actuate: {}", actuate);

            //defmt::info!("Actuate: {}", actuate);
            let actuate_frame = lib::protocol::MessageType::Write(WriteType::MotorReference {
                target: actuate,
                deadline: 30,
            });
            cx.shared
                .sender
                .lock(|sender| sender.enqueue(actuate_frame));
            //.try_send(Some(actuate_frame))
            //.unwrap();
            //defmt::info!("Should have sent now....");
        } else {
            defmt::warn!(":/");
        }

        //cx.shared.stype.lock(|stype| stype.start_sample());
        //Mono::delay_until(time + 20u64.millis()).await;
        //defmt::info!("Adc sample!");
        //Mono::delay(20u64.millis()).await;
        //Mono::delay_ms(&mut Mono, 20);
        cx.local
            .control_timer
            .timeout((time + 20u64.millis()) - Mono::now());
        cx.shared.stype.lock(|stype| stype.start_sample());

        //defmt::info!("After delay!");
        //time = Mono::now();
    }
}

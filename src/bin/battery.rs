#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]
#![feature(generic_arg_infer)]

use controller as _; // global logger + panicking-behavior + memory layout


#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = nrf52840_hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [RTC0]
)]

mod app {
    use defmt::info;
// use embedded_hal::digital::OutputPin;
   // use core::arch::asm;
   // use rtic_monotonics::systick::*;

    use controller::drivers::can::{
            Mcp2515Driver,
            Mcp2515Settings,
    };
    use embedded_can::blocking::Can;

    use lib::protocol::sender::Sender;
    use nrf52840_hal::{self as hal,
        gpio::*,
        gpiote::Gpiote,
        pac::SPI0,
        spi::Spi,
        pac::Peripherals,
        saadc::SaadcConfig,    
        //Clocks,
    };
   // use rtic_monotonics::Monotonic;

    //use crate::Mono;

    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
        

    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
        curr_soc: f32,
        saadc: hal::saadc::Saadc,
        saadc_pin: p0::P0_31<Input<Floating>>,
        old_soc: f32,
        can_driver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        sender: Sender<10>,

    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = Peripherals::take().unwrap();

        //let clk = Clocks::new(device.CLOCK).enable_ext_hfosc().start_lfclk();
        // Mono::start(device.RTC0);
        // set up pins
        let port0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let _port1 = hal::gpio::p1::Parts::new(cx.device.P1);

        /* 
        // setup monotonic systic for peroidic interupts 
        let sysclk = 16_000_000;
        let token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, sysclk, token);
        */

        // set up saadic(Successive approximation analog-to-digital converter)
        let saadc_config = SaadcConfig::default();
        let mut saadc = hal::saadc::Saadc::new(cx.device.SAADC, saadc_config);
        let mut saadc_pin: p0::P0_31<Input<Floating>> = port0.p0_31.into_floating_input();

        // Estimate the current state of charge in battery
        // TODO add logic to estimate charge using battery temp and voltage 
        let _saadc_result = saadc.read_channel(&mut saadc_pin).unwrap() as f32;
        

        // set up state of charge varibles
        let old_soc: f32 = 0.0;
        let curr_soc: f32 = 0.0;

        // set up CAN
        //CAN PINS
        let can_interrupt = port0.p0_24.into_pullup_input().degrade();
        let cs0: Pin<Output<PushPull>> = port0.p0_20.into_push_pull_output(Level::High).degrade();
        let sck: Pin<Output<PushPull>> = port0.p0_23.into_push_pull_output(Level::Low).degrade();
        let miso: Pin<Input<Floating>> = port0.p0_21.into_floating_input().degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_22.into_push_pull_output(Level::Low).degrade();

        let _spi_pins = hal::spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: Some(mosi),
        };

        // set up can setting
        let spi_can: Spi<SPI0> = unsafe { (0x0 as *mut Spi<SPI0>).read() };
        let settings = Mcp2515Settings::default();
        let can_driver = Mcp2515Driver::init(spi_can, cs0, can_interrupt, settings);

        // new interupt (should not be used as this firmware only sends data)
        let gpiote = Gpiote::new(device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        // create sender to be able to send data via can
        let sender: Sender<_> = Sender::new();


        analog_read::spawn().ok();
        (
            Shared {
                // Initialization of shared resources go here
                  
            },
            Local {
                // Initialization of local resources go here
                saadc, saadc_pin, old_soc, sender, can_driver, curr_soc

            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
           // analog_read::spawn().ok();
            continue;
        }
    }
    
    // TODO: Add tasks
    #[task(priority = 1, local=[saadc, saadc_pin, old_soc, curr_soc, can_driver, sender])]
    async fn analog_read(cx: analog_read::Context) {
         // set up varibles for easier readibility
        let saadc = cx.local.saadc;
        let saadc_pin = cx.local.saadc_pin;
        let old_soc:f32 = *cx.local.old_soc;
        let curr_soc:f32 = *cx.local.curr_soc;
        let can_driver = cx.local.can_driver;
        let sender = cx.local.sender;

        // read value from saadc_pin
        let mut saadc_result = saadc.read_channel(saadc_pin).unwrap() as f32;

        // remove negative analog results incase of calibration errors (should be handled better but need more time for that) 
        if saadc_result < 0.0 {
            saadc_result = 0.0; 
        }
    
        let curr_voltage: f32 = (saadc_result * 3.3)/(1<<14) as f32;
        info!("Saadc_result is {}, and curr voltage is {}, ", saadc_result, curr_voltage);
        // transform ADC to Voltage ((analog_sample * voltage_ref)/resulution)
        
        
        // info!("current_voltage is {}", curr_voltage);
        

        if curr_soc < (old_soc-0.01) {
            let _ = sender.set_status_battery(curr_soc);
            let mut msg = match sender.dequeue() {
                Some(m) => m,
                None => {return},
            };
            msg.print_frame();
            let _ = can_driver.transmit(&msg);

        }
    }


    /* 
    #[task(priority = 1, local=[], shared=[])]
    async fn send_can(cx: send_can::Context) {
        // set up varibles for easier readibility
        let can_driver = cx.local.can_driver;
        let sender = cx.local.sender;
        let current_state_of_charge: f32;
        // mutex on curr_soc varuble
        cx.shared.curr_soc.lock(|curr_soc|{
            let get_soc:f32 = *curr_soc;
            current_state_of_charge = get_soc;
            
        });

        sender.set_status_battery(current_state_of_charge).unwrap();
        
        let mut msg = sender.dequeue().unwrap();
        msg.print_frame();
        can_driver.transmit(&msg).unwrap();
    }
   */

}

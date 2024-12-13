#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]

use controller as _; // global logger + panicking-behavior + memory layout


#[rtic::app(
    // TODO: Replace `some_hal::pac` with the path to the PAC
    device = nrf52840_hal::pac,
    // TODO: Replace the `FreeInterrupt1, ...` with free interrupt vectors if software tasks are used
    // You can usually find the names of the interrupt vectors in the some_hal::pac::interrupt enum.
    dispatchers = [RTC0]
)]

mod app {
    use embedded_hal::digital::OutputPin;
        //use core::arch::asm;
    //use nrf52840_hal::gpio::{self, Floating, Input};
  //  use rtic_monotonics::systick::*;
    use nrf52840_hal::{self as hal, gpio::* /* , pac::p0::pin_cnf::INPUT_A*/};
    //use hal::gpio::{, Floating};
  //  use hal::saadc::{SaadcConfig, Saadc};


    // Shared resources go here
    #[shared]
    struct Shared {
        // TODO: Add resources
      //  curr_soc: f32,

    }

    // Local resources go here
    #[local]
    struct Local {
        // TODO: Add resources
      //  saadc: hal::saadc::Saadc,
      //  saadc_pin: hal::gpio::Pin<INPUT_A>,
      //  old_soc: f32,
        test_pin1: hal::gpio::Pin<hal::gpio::Output<PushPull>>,
        test_pin2: hal::gpio::Pin<hal::gpio::Output<PushPull>>,

    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");

        // set up pins
        let p0 = hal::gpio::p0::Parts::new(cx.device.P0);

        // set up 
        //let old_soc = 1.00; // place holder 
        //let curr_soc: f32;

        // nrf test
        let test_pin1 = p0.p0_07.into_push_pull_output(Level::High).degrade();
        let test_pin2 = p0.p0_08.into_push_pull_output(Level::High).degrade();
        /*
        // setup monotonic systic for peroidic interupts 
        let sysclk = 64_000_000;
        let token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, sysclk, token);
        
        // set up saadic(Successive approximation analog-to-digital converter)
        let saadc_config = SaadcConfig::default();
        let mut saadc = Saadc::new(cx.device.SAADC, saadc_config);
        let mut saadc_pin = p0.p0_31;


        let _saadc_result = saadc.read_channel(&mut saadc_pin).unwrap() as f32;
        */
        nrf_test::spawn().ok();
        (
            Shared {
                // Initialization of shared resources go here
                //  curr_soc
            },
            Local {
                // Initialization of local resources go here
               // saadc, saadc_pin, old_soc
               test_pin1, test_pin2
            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            continue;
        }
    }
    /* 
    // TODO: Add tasks
    #[task(priority = 1, local=[saadc, saadc_pin, old_soc], shared=[curr_soc])]
    async fn analog_read(mut cx: analog_read::Context) {
        defmt::info!("Start analog read");

        // read value from saadc_pin
        let saadc_result = cx.local.saadc.read_channel(&mut cx.local.saadc_pin).unwrap() as f32;

        // transform ADC to Voltage ((analog_sample * voltage_ref)/resulution)
        let curr_voltage = (saadc_result * 3.3)/(1<<14) as f32;

        // lock mutex for curr_soc
        cx.shared.curr_soc.lock(|curr_soc|{
            // calc new curr_soc
            *curr_soc = *cx.local.old_soc + (9.8*curr_voltage)/60.0; 
            
        });

    }

    */
    #[task(priority = 1, local=[test_pin1,test_pin2])]
    async fn nrf_test(cx: nrf_test::Context){
        defmt::info!("Start nrf_test");
        cx.local.test_pin1.set_high().ok();
        cx.local.test_pin2.set_high().ok();
    }
}

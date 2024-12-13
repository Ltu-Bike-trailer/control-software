#![no_main]
#![no_std]
#![feature(type_alias_impl_trait)]
#![deny(clippy::all)]
#![deny(warnings)]
#![feature(generic_arg_infer)]


use controller as _; // global logger + panicking-behavior + memory layout


#[rtic::app(
    device = nrf52840_hal::pac,
    dispatchers = [RTC0]
)]

mod app {

    use defmt::info;
    use controller::drivers::can::{
            Mcp2515Driver,
            Mcp2515Settings,
    };
    use embedded_can::blocking::Can;

    use lib::protocol::sender::Sender;
    use nrf52840_hal::{self as hal, gpio::*, gpiote::Gpiote, pac::SPI0, saadc::{Gain, Oversample, Reference, Resistor, Resolution, SaadcConfig, Time}, spi::{Frequency, Spi}  
    };
    use rtic_monotonics::systick::*;


    // Shared resources go here
    #[shared]
    struct Shared {
        gpiote: Gpiote,

    }

    // Local resources go here
    #[local]
    struct Local {
        saadc: hal::saadc::Saadc,
        saadc_pin: p0::P0_31<Input<Floating>>,
        old_soc: f32,
        can_driver: Mcp2515Driver<Spi<SPI0>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        sender: Sender<10>,

    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;

        // set up pins
        let port0 = hal::gpio::p0::Parts::new(device.P0);

        // setup monotonic systic for periodic interrupts 
        let sysclk = 64_000_000;
        let token = rtic_monotonics::create_systick_token!();
        Systick::start(cx.core.SYST, sysclk, token);

        // set up saadic(Successive approximation analog-to-digital converter)
        // set up setting for sampling analog signal
        let saadc_config = SaadcConfig {
            resolution: Resolution::_14BIT,
            oversample: Oversample::OVER256X,
            reference: Reference::INTERNAL,
            gain: Gain::GAIN1_6,
            resistor: Resistor::BYPASS,
            time: Time::_40US,
        };

        // create new saadc object
        let mut saadc = hal::saadc::Saadc::new(device.SAADC, saadc_config);

        // assign readable pins for saadc
        let saadc_pin: p0::P0_31<Input<Floating>> = port0.p0_31.into_floating_input();
        let mut saadc_pin_0: p0::P0_30<Input<Floating>> = port0.p0_30.into_floating_input();
        let mut saadc_pin_1: p0::P0_29<Input<Floating>> = port0.p0_29.into_floating_input();
        let mut saadc_pin_2: p0::P0_28<Input<Floating>> = port0.p0_28.into_floating_input();
        
        // Estimate the current state of charge in battery
        // read analog value from P0.28, P0.29 and P0.30
        let saadc_result_0 = saadc.read_channel(&mut saadc_pin_0).unwrap() as f32;
        let saadc_result_1 = saadc.read_channel(&mut saadc_pin_1).unwrap() as f32;
        let saadc_result_2 = saadc.read_channel(&mut saadc_pin_2).unwrap() as f32;

        // convert analog values to voltage 
        let saadc_to_volt_0 = (saadc_result_0 * 0.6)/((1<<14) as f32 * (1.0/6.0));
        let saadc_to_volt_1 = (saadc_result_1 * 0.6)/((1<<14) as f32 * (1.0/6.0));
        let saadc_to_volt_2 = (saadc_result_2 * 0.6)/((1<<14) as f32 * (1.0/6.0));

        // difference in voltage between P0.28 and P0.29
        let temp_volt = (saadc_to_volt_1 - saadc_to_volt_2).abs();

        // temperature = (-5.8151*current_voltage³ + 24.9708*current_voltage² - 61.8928*current_voltage + 48.8572)
        let temp = -5.8151*(temp_volt*temp_volt*temp_volt) + 24.9708*(temp_volt*temp_volt) - 61.8928*temp_volt + 48.8572;
        info!("The temperature is {}", temp);

        // battery voltage Vm = Vb*10k/(150k+10k)
        let bat_voltage = ((saadc_to_volt_0*(150000.0+10000.0))/10000.0)+1.3;

        // battery voltage loss by temp function (approximated from us gov study: https://www.osti.gov/servlets/purl/975252) 
        let bat_temp_ratio = 0.817 + 9.19*0.001*temp + -4.24*0.00001*temp*temp;

        // voltage normailized for 25 degrees
        let norm = bat_voltage/bat_temp_ratio;

        // calculate the first state of charge
        // State of charge estimation (linear estimation based on values form https://naturesgenerator.com/blogs/news/lead-acid-battery-voltage-chart?srsltid=AfmBOoqZi_WlVo8OX221jsoNeKqaDMWyDLfuOsrSwsqLOeA4DP-hocwW)
        let old_soc = 0.251*norm -8.7;
        info!("Old soc: {}", old_soc);


        // set up CAN
        // CAN PINS
        let can_interrupt = port0.p0_24.into_pullup_input().degrade();
        let cs0: Pin<Output<PushPull>> = port0.p0_20.into_push_pull_output(Level::High).degrade();
        let sck: Pin<Output<PushPull>> = port0.p0_23.into_push_pull_output(Level::Low).degrade();
        let miso: Pin<Input<Floating>> = port0.p0_21.into_floating_input().degrade();
        let mosi: Pin<Output<PushPull>> = port0.p0_22.into_push_pull_output(Level::Low).degrade();

        let spi_pins = hal::spi::Pins {
            sck: Some(sck),
            miso: Some(miso),
            mosi: Some(mosi),
        };

        let spi_battery: Spi<SPI0> = Spi::new(
            device.SPI0,
            spi_pins,
            Frequency::K250,
            embedded_hal::spi::MODE_0,
        );

        // set up can setting
        let settings = Mcp2515Settings::default();
        let can_driver = Mcp2515Driver::init(spi_battery, cs0, can_interrupt, settings);

        // new interrupt (should not be used as this firmware only sends data)
        let gpiote = Gpiote::new(device.GPIOTE);
        gpiote
            .channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        // create sender to be able to send data via can
        let sender: Sender<_> = Sender::new();

        (
            Shared {
                // Initialization of shared resources go here
                gpiote
            },
            Local {
                // Initialization of local resources go here
                saadc, saadc_pin, old_soc, sender, can_driver

            },
        )
    }

    // Optional idle, can be removed if not needed.
    #[idle]
    fn idle(_: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            analog_read::spawn().ok();
            continue;
        }
    }
    
    // TODO: Add tasks
    #[task(priority = 1, local=[saadc, saadc_pin, old_soc, /*curr_soc,*/ can_driver, sender])]
    async fn analog_read(cx: analog_read::Context) {
        info!("analog read task started");

        // set up variables for easier readability
        let saadc = cx.local.saadc;
        let saadc_pin = cx.local.saadc_pin;
        let old_soc = cx.local.old_soc;
        let can_driver = cx.local.can_driver;
        let sender = cx.local.sender;

        // read value from saadc_pin
        let mut saadc_result = match saadc.read_channel(saadc_pin) {
            Ok(m) => {m},
            Err(_) => {return},
        } as f32;

        // remove negative analog results in case of calibration errors (should be handled better but need more time for that) 
        if saadc_result < 0.0 {
            saadc_result = 0.0; 
        }

        // Transform ADC to Voltage ((analog_sample * voltage_ref)/resolution)
        let curr_voltage: f32 = ((saadc_result) * 0.6)/((1<<14) as f32 * (1.0/6.0));
        info!("Saadc_result is {}, and curr voltage is {}, ", saadc_result, curr_voltage);

        // voltage divider rule to get the volt in (volt_out = volt_in * (R2/(R1+R2)))
        let volt_in = curr_voltage*(4700.0+10000.0)/10000.0;
        info!("volt in is {}", volt_in);

        // Calculate current in shunt resistor(0.0335*x + -0.0798)
        let i_shunt = volt_in/(50.0 * 0.003);
        info!("test {}", i_shunt);

        // Calculate current state of charge(curr_soc = old_soc - (i_shunt*delta_t)/Q)
        let mut curr_soc = *old_soc - i_shunt/20.0 * 0.5;

        // normalize for temp variations battery state of charge
        curr_soc = curr_soc.clamp(0.0, 1.0);
        
        // if the old State of Charge(soc) has lowered by 1% send the new soc via CAN
        if curr_soc < (*old_soc-0.01) {
            let _ = sender.set_status_battery(curr_soc);
            let mut msg = match sender.dequeue() {
                Some(m) => m,
                None => {return},
            };
            msg.print_frame();
            let _ = can_driver.transmit(&msg);
        }

        // set current state of charge to the old state of carge
        *old_soc = curr_soc;
        info!("current SoC is {}", old_soc);

        Systick::delay(500.millis()).await;
    }

    #[task(binds = GPIOTE, shared = [gpiote])]
    fn can_interrupt(mut cx: can_interrupt::Context) {
        //Do nothing this board has a shared SPI bus so its dangerous to read whenever
        //also it has no reason to read any messages of the can bus.
        cx.shared.gpiote.lock(|gpiote| {
            gpiote.reset_events();
        });
    }

}

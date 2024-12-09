#![no_std]
#![no_main]
#![allow(unused)]
#![feature(generic_arg_infer)]
use controller as _;
use controller::drivers::{
    hx711::*,
};
use can_mcp2515::drivers::can::*;
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use panic_probe as _;
use rtic::app;
use rtic_monotonics::nrf::rtc::prelude::*;
nrf_rtc0_monotonic!(Mono);

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC1])]
mod app {
    
    use can_mcp2515::drivers::can::*;
    use controller::{
        boards::*,
        drivers::{
            hx711::*,
        },
    };
    use cortex_m::asm;
    use embedded_can::{blocking::Can, Frame, StandardId};
    use embedded_hal::{digital::OutputPin, spi::SpiBus};
    use lib::protocol::sender::Sender;
    use nrf52840_hal::{
        gpio::{self, Floating, Input, Level, Output, Pin, Port, PullUp, PushPull},
        gpiote::{Gpiote, GpioteInputPin},
        pac::{Interrupt, GPIOTE, PWM0, SPI0, SPI1, SPI2, SPIM0, SPIM1, SPIM2},
        pwm::{self, Channel, Pwm},
        spi::{Frequency, Spi},
        spim::{self, *},
        time::U32Ext,
        Clocks,
    };
    use rtic_monotonics::{
        fugit::{ExtU32, ExtU64},
        Monotonic,
    };

    use crate::Mono;

    #[shared]
    struct Shared {
        cargo_weight: f32,
        s_type_force: f32,
        gpiote: Gpiote,
        pwm: Pwm<PWM0>,
    }

    #[local]
    struct Local {
        stype: SAADC,
        hx711: Hx711Driver<Pin<Output<PushPull>>, Pin<Input<Floating>>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local) {
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);
        let t = nrf52840_hal::pac::Peripherals::take();
        let clk = Clocks::new(device.CLOCK).enable_ext_hfosc().start_lfclk();
        let systick = cx.core.SYST;
        let timer0 = device.TIMER0;

        Mono::start(device.RTC0);

        //let pin_mapping = PinMapping::new(port0, port1);
        //let (pin_map, spi) =pin_mapping.can(device.SPIM1);

        defmt::println!("Initialize the HX711 Driver!");

        //TODO Change these pins to be the proper ones
        // HX711
        let dout_pin = port0.p0_04.into_floating_input().degrade();
        let pd_sck = port0.p0_05.into_push_pull_output(Level::Low).degrade();
        let pwm_output_pin = port0.p0_07.into_push_pull_output(Level::High).degrade();

        let mut gpiote = Gpiote::new(device.GPIOTE);
        let mut pwm = Pwm::new(device.PWM0);
        let mut hx711_instance = Hx711Driver::init(pd_sck, dout_pin, Gain::Apply128,  ValidTimings::default());

        gpiote
            .channel0()
            .input_pin(&hx711_instance.dout)
            .hi_to_lo()
            .enable_interrupt();

        pwm.set_output_pin(Channel::C0, pwm_output_pin)
            .set_period(500u32.hz())
            .enable();

        // S-Type Load cell
        // TODO set the correct pin
        // ADC configuration.
        let mut cfg = SaadcConfig::default();
        cfg.time = Time::_40US;
        cfg.gain = adc_gain();
        cfg.oversample = saadc::Oversample::OVER256X;
        cfg.reference = saadc::Reference::INTERNAL;

        //TODO update to correct pin
        // The input pin.
        let input = p0.p0_29;

        //TODO update to correct pin
        let mut adc = SaadcTask::new(cx.device.SAADC, cfg, &[P0_29::<Disconnected>::channel()], [
            0,
        ]);


        //Start sampeling the stype
        adc.start_sample(); 
        read_hx711::spawn();
        (
        Shared {
            let cargo_weight:f32,
            let s_type_force:f32,
            gpiote,
            pwm
        },
        Local {
            stype: adc,
            hx711: hx711_instance,
        }
        )
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }


    ///High priority constant polling of S-Type loadcell
    #[task(binds = SAADC, local=[stype],shared =[s_type_force],priority=1)]
    fn read_stype(mut cx: read_stype::Context) {
        let sample = cx.local.stype.complete_sample();
        
        defmt::info!("Sample : {:?}", sample);

        // Scale in between.
        const GAIN: f32 = 10.;
        let converted = GAIN * measured;
        defmt::info!("Measured {}N", converted);

        cx.shared.s_type_force.lock(|f| {

            *f = converted;
        });
        cx.local.stype.start_sample();
    }

    ///Lowest priority task. Expected to be static during operation but needs to be polled occasionally to keep a consistent value
    #[task(local = [hx711], shared = [cargo_weight] priority = 1)]
    async fn read_hx711(cx: read_hx711::Context) {
        loop{
            let data = cx
                .local
                .hx711
                .read_full_period::<Mono, _, _>(Gain::Apply128)
                .await;
            let decoded_data = cx.local.hx711.decode_data(data);
            defmt::println!(
                "Data [24-bit]: {:024b}\n Data: {:?}\nData [32-bit]: {:032b}",
                data,
                data,
                data
            );
            defmt::println!("Decoded Data: {:?}", decoded_data.data_out);

            // Write the data to the shared variable
            cx.shared.cargo_weight.lock(|cw| cw = decoded_data);

            //Delay to only read every few seconds to allow for wfi();
            Mono::delay_s(&mut Mono, 5);
        }
    }
    
    use nrf52840_hal::saadc;
    /// Resistance to ground for the current sense pins.
    pub const R_PHASE_SENSE_TO_GROUND: f32 = 10_000.;

    /// Resistance to vin for the current sense pins.
    pub const R_PHASE_SENSE_TO_VIN: f32 = 4_700.;

    /// The shunt resistor that the current sense measures voltage across.
    ///
    /// The Sense resistor is 10 mOhm on the esc.
    pub const R_SHUNT: f32 = 0.01;

    /// The gain that the sensor applies to the differential measurement.
    pub const SENSE_GAIN: f32 = 50.;

    /// The system voltage.
    pub const V_REF: f32 = 0.6;

    /// The gain in the nrf chip.
    pub const ADC_BITS: usize = 14;

    /// The gain of the adc on the nrf chip.
    pub const ADC_GAIN: f32 = 1. / 6.;

    /// The voltage divider used for current sensing.
    /// $I_shunt = (2.5-V_sense *  ( R_5+R_4 ) / R_5) / ( R_shunt  * NågonGain )
    /// $
    ///
    ///
    /// RESULT = [V(P) – V(N)] * (GAIN/REFERENCE) * 1<<(RESOLUTION - m)
    /// V(P) - V(N) = RESULT/((GAIN/REFERENCE) * 1<<(RESOLUTION - m))
    pub const fn raw_adc_to_current_factor() -> f32 {
        //(ADC_GAIN / V_REF) * ((1 << ADC_BITS) as f32) * v_div()
        //(V_REF / ((1 << ADC_BITS) as f32)) / ADC_GAIN

        (V_REF / (ADC_GAIN * (1 << ADC_BITS) as f32) * v_div()) as f32
    }

    /// Returns the voltage divider
    pub const fn v_div() -> f32 {
        (R_PHASE_SENSE_TO_GROUND + R_PHASE_SENSE_TO_VIN) / R_PHASE_SENSE_TO_GROUND
    }

    /// Combines the gain and shunt value in one.
    ///
    /// This is multiplied by a factor of 1000
    pub const fn sensor_gain() -> f32 {
        SENSE_GAIN as f32
    }

    /// Converts the adc value in to mA.
    #[inline(always)]
    fn conv(val: u16) -> f32 {
        2.5 / sensor_gain() - f32::from(val) * raw_adc_to_current_factor() / sensor_gain()
    }

    /// Converts the adc gain to the type representation at compile time.
    pub const fn adc_gain() -> saadc::Gain {
        if ADC_GAIN == 1. / 6. {
            saadc::Gain::GAIN1_6
        } else if ADC_GAIN == 1. / 5. {
            saadc::Gain::GAIN1_5
        } else if ADC_GAIN == 1. / 4. {
            saadc::Gain::GAIN1_4
        } else if ADC_GAIN == 1. / 3. {
            saadc::Gain::GAIN1_3
        } else if ADC_GAIN == 1. / 2. {
            saadc::Gain::GAIN1_2
        } else if ADC_GAIN == 1. {
            saadc::Gain::GAIN1
        } else if ADC_GAIN == 2. {
            saadc::Gain::GAIN2
        } else {
            panic!("Invalid ADC_GAIN")
        }
    }
}
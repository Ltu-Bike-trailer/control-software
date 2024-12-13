#![no_std]
#![no_main]
#![allow(unused)]
#![feature(generic_arg_infer)]
<<<<<<< HEAD
use can_mcp2515::drivers::can::*;
=======
use can_mcp2515::drivers::can::{Mcp2515Driver, Mcp2515Settings};
>>>>>>> 522b09b (Esc v2 merge (#35))
use controller as _;
use controller::drivers::hx711::*;
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

<<<<<<< HEAD
    use can_mcp2515::drivers::can::*;
=======
    use can_mcp2515::drivers::can::{
        AcceptanceFilterMask,
        Bitrate,
        Mcp2515Driver,
        Mcp2515Settings,
        McpClock,
        OperationTypes,
        ReceiveBufferMode,
        SettingsCanCtrl,
        CLKPRE,
        RXBN,
    };
>>>>>>> 522b09b (Esc v2 merge (#35))
    use controller::{boards::*, drivers::hx711::*};
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
        gpiote: Gpiote,
        pwm: Pwm<PWM0>,
    }

    #[local]
    struct Local {
        hx711: Driver<Pin<Output<PushPull>>, Pin<Input<Floating>>>,
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

        let dout_pin = port0.p0_04.into_floating_input().degrade();
        let pd_sck = port0.p0_05.into_push_pull_output(Level::Low).degrade();
        let pwm_output_pin = port0.p0_07.into_push_pull_output(Level::High).degrade();

        let mut gpiote = Gpiote::new(device.GPIOTE);
        let mut pwm = Pwm::new(device.PWM0);

        let timing_delays = ValidTimings::default();
        let (t1_val, t2_val, t3_val, t4_val) = (100u64, 10u64, 300u64, 300u64); // Alt.
        let applied_timing = ValidTimings::new(t1_val, t2_val, t3_val, t4_val); // Alt.

        let mut hx711_instance = Driver::init(pd_sck, dout_pin, Gain::Apply128, timing_delays);

        gpiote
            .channel0()
            .input_pin(&hx711_instance.dout)
            .hi_to_lo()
            .enable_interrupt();

        pwm.set_output_pin(Channel::C0, pwm_output_pin)
            .set_period(500u32.hz())
            .enable();

        (Shared { gpiote, pwm }, Local {
            hx711: hx711_instance,
        })
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }

    #[task(local = [hx711], priority = 2)]
    async fn read_data(cx: read_data::Context) {
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
    }

    #[task(binds = GPIOTE, shared = [gpiote])]
    fn data_interrupt(mut cx: data_interrupt::Context) {
        cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::info!("Analog data received!");
                read_data::spawn();
                gpiote.channel0().reset_events();
            }
        });
    }
}

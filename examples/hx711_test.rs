#![no_std]
#![no_main]
#![allow(unused)]

use controller as _;
use controller::drivers::{
    can::{Mcp2515Driver, Mcp2515Settings},
};
use cortex_m::asm as _;
use cortex_m_rt::entry;
use defmt_rtt as _;
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use panic_probe as _;
use rtic::app;

#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC0])]
mod app {

    use controller::{
        boards::*,
        drivers::{
            can::{
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
            },
        },
    };
    use cortex_m::asm;
    use embedded_can::{blocking::Can, Frame, StandardId};
    use embedded_hal::{digital::OutputPin, spi::SpiBus};
    use nrf52840_hal::{
        gpio::{self, Floating, Input, Level, Output, Pin, Port, PullUp, PushPull}, gpiote::{Gpiote, GpioteInputPin}, pac::{Interrupt, GPIOTE, PWM0, SPI0, SPI1, SPI2, SPIM0, SPIM1, SPIM2}, pwm::{self, Channel, Pwm}, spi::{Frequency, Spi}, spim::{self, *}, time::U32Ext, Clocks
    };
    
    use lib::protocol::sender::Sender;
    use rtic_monotonics::Monotonic;
    use rtic_monotonics::nrf::{
        self,
        rtc::*,
        timer::{fugit::ExtU64, Timer0 as Mono},
    };

    

    #[shared]
    struct Shared {
        gpiote: Gpiote,
        pwm: Pwm<PWM0>, 
    }

    #[local]
    struct Local {
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
        
        //let pin_mapping = PinMapping::new(port0, port1);
        //let (pin_map, spi) =pin_mapping.can(device.SPIM1);

        defmt::println!("Initialize the SPI instance, and CanDriver");

        let dout_pin = port0.p0_02.into_floating_input().degrade();
        let pd_sck = port0.p0_03.into_push_pull_output(Level::Low).degrade();
        let pwm_output_pin = port0.p0_04.into_push_pull_output(Level::High).degrade();

        let mut gpiote = Gpiote::new(device.GPIOTE);
        let mut pwm = Pwm::new(device.PWM0);
        let interrupt_token = rtic_monotonics::create_nrf_timer0_monotonic_token!(); 
        Mono::start(timer0, interrupt_token); 

        gpiote
            .channel0()
            .input_pin(&dout_pin)
            .hi_to_lo()
            .enable_interrupt();

        pwm
            .set_output_pin(Channel::C0, pwm_output_pin)
            .set_period(500u32.hz())
            .enable();

        (Shared { gpiote, pwm}, Local {
            
        })
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");

        loop {
            asm::wfi();
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote])]
    fn data_interrupt(mut cx: data_interrupt::Context) {
        let handle = cx.shared.gpiote.lock(|gpiote| {
            if (gpiote.channel0().is_event_triggered()) {
                defmt::println!("Analog data received!");
                gpiote.channel0().reset_events();
            }

        });
    }
}

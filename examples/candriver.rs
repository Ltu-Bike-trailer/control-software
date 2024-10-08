#![no_std]
#![no_main]
#![allow(unused)]

use controller::drivers::{can::{CanDriver, CanSettings}, message::CanMessage};
use nrf52840_hal as _;
use nrf52840_hal::gpio::{Level, Port};
use cortex_m::asm as _;
use defmt_rtt as _;
use panic_probe as _;
use cortex_m_rt::entry;
use rtic::app;

use controller as _;


#[rtic::app(device = nrf52840_hal::pac, dispatchers = [RTC0])]
mod app {

    use controller::drivers::{can::{CanControllSettings, CanDriver, CanSettings, OperationTypes, CLKPRE, McpClock, CanBitrate}, message::CanMessage};
    use controller::boards::*;
    use cortex_m::asm;
    use embedded_can::{StandardId, Frame};
    use embedded_can::blocking::Can;
    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::SpiBus;
    use nrf52840_hal::{gpio::PullUp, gpiote::{Gpiote, GpioteInputPin}, pac::{Interrupt, GPIOTE, SPI1, SPI2, SPIM0, SPIM1, SPIM2}};
    use nrf52840_hal::{spim, Clocks};
    use nrf52840_hal::{pac::SPI0, spi::Spi, spi::Frequency, spim::*};
    use nrf52840_hal::gpio::{self, Level, Port, Pin, Output, PushPull, Input, Floating};
    use rtic_monotonics::nrf::timer::{fugit::ExtU64, Timer0 as Mono};
    use rtic_monotonics::nrf::{self, rtc::*};
     

    #[shared]
    struct Shared{
        gpiote: Gpiote,
    }

    #[local]
    struct Local {
        candriver: CanDriver<Spi<SPI1>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
        candriver_node: CanDriver<Spi<SPI2>, Pin<Output<PushPull>>, Pin<Input<PullUp>>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local){
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);
        let t = nrf52840_hal::pac::Peripherals::take();
        let clk = Clocks::new(device.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk();
      
        /* Disable shared peripheral addresses */
        //device.SPI1.enable.write(|w| w.enable().disabled());
        device.SPIM1.enable.write(|w| w.enable().disabled());        
        device.SPIS1.enable.write(|w| w.enable().disabled());
        device.TWIM1.enable.write(|w| w.enable().disabled());
        device.TWIS1.enable.write(|w| w.enable().disabled());
        device.TWI1.enable.write(|w| w.enable().disabled());
       
        //let pin_mapping = PinMapping::new(port0, port1);
        //let (pin_map, spi) =pin_mapping.can(device.SPIM1);

        defmt::println!("Initialize the SPI instance, and CanDriver");
        let pins = nrf52840_hal::spi::Pins{
            sck: Some(port0.p0_05.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_06.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_07.into_floating_input().degrade()),
        };

        let pins_node = nrf52840_hal::spi::Pins{
            sck: Some(port0.p0_04.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_03.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_10.into_floating_input().degrade()),
        };
        
        let cs_pin = port1.p1_02.into_push_pull_output(Level::High).degrade(); 
        let can_interrupt = port1.p1_01.into_pullup_input().degrade();

        let cs_node_pin = port1.p1_10.into_push_pull_output(Level::High).degrade();
        let can_node_interrupt = port1.p1_11.into_pullup_input().degrade();

        let mut spi = Spi::new(device.SPI1, pins, Frequency::M1, MODE_0);
        let mut spi_node = Spi::new(device.SPI2, pins_node, Frequency::M1, MODE_0);
        let mut gpiote = Gpiote::new(device.GPIOTE);
        
        let dummy_data = "dummy".as_bytes();

        let settings = CanSettings::default(); // same as below
        
        const CLKEN: bool = true;         
        const OSM: bool = false;
        const ABAT: bool = false; 

        let canctrl_settings = CanControllSettings::new(OperationTypes::Configuration, CLKEN, CLKPRE::DIV1, ABAT, OSM);
        
        let can_settings = CanSettings{
            canctrl: canctrl_settings,
            mcp_clk: McpClock::MCP8,
            can_bitrate: CanBitrate::CAN125,
            interrupts: 0u8,
        };
        
        let mut can_driver = CanDriver::init(spi, cs_pin, can_interrupt, can_settings);
        let mut can_node = CanDriver::init(spi_node, cs_node_pin, can_node_interrupt, can_settings);

         gpiote.channel0()
            .input_pin(&can_driver.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        gpiote.channel1()
            .input_pin(&can_node.interrupt_pin)
            .hi_to_lo()
            .enable_interrupt();

        defmt::println!("After initializing Spi<SPI1>...");
        let dummy_id = StandardId::new(0x1).unwrap();
        defmt::info!("dummy_data: {:?}", dummy_data.len());
        let mut frame = CanMessage::new(embedded_can::Id::Standard(StandardId::ZERO), &[0x01, 0x02, 0x03]).unwrap();
       
        //can_driver.loopback_test(frame);
        //can_driver.transmit(&frame);
        can_node.transmit(&frame);
        //can_node.loopback_test(frame);
        (
            Shared {gpiote},
            Local {candriver: can_driver, candriver_node: can_node},
        )
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");
        
        loop {
            asm::wfi(); 
        }
    }

    #[task(binds = GPIOTE, shared = [gpiote], local = [candriver, candriver_node, counter: u16 = 0])]
    fn can_interrupt(mut cx: can_interrupt::Context){
        let handle = cx.shared.gpiote.lock(|gpiote|{
           
            if (gpiote.channel0().is_event_triggered()){
                defmt::info!("GPIOTE interrupt occured, for Can Master at channel 0!");
                let interrupt_type = cx.local.candriver.interrupt_decode();
                cx.local.candriver.handle_interrupt(interrupt_type).unwrap();
           
                if (cx.local.candriver.interrupt_is_cleared()){
                    defmt::info!("All CAN interrupt has been handled!");
                    gpiote.channel0().reset_events();
                    //gpiote.reset_events(); //Execute when all Events are handled. 
                }
            }else if (gpiote.channel1().is_event_triggered()) {
                defmt::info!("GPIOTE interrupt occured, for Can Node at channel 1!");
                let interrupt_type = cx.local.candriver_node.interrupt_decode();
                cx.local.candriver_node.handle_interrupt(interrupt_type).unwrap(); 
                
                if (cx.local.candriver_node.interrupt_is_cleared()){
                    gpiote.channel1().reset_events();
                }                
            }
        });
    }
}

#![no_std]
#![no_main]
#![allow(unused)]

use controller::drivers::can::{CanDriver, CanSettings, CanMessage};
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
    use controller::drivers::can::{CanControllSettings, CanDriver, CanMessage, CanSettings, OperationTypes, CLKPRE};
    use cortex_m::asm;
    use embedded_hal::digital::OutputPin;
    use embedded_hal::spi::SpiBus;
    use nrf52840_hal::pac::{Interrupt, SPIM0, SPIM1, SPIM2};
    use nrf52840_hal::{spim, Clocks};
    use nrf52840_hal::{pac::SPI0, spi::Spi, spi::Frequency, spim::*};
    use nrf52840_hal::gpio::{self, Level, Port, Pin, Output, PushPull};
    use rtic_monotonics::nrf::timer::{fugit::ExtU64, Timer0 as Mono};
    use rtic_monotonics::nrf::{self, rtc::*};
    
  

    #[shared]
    struct Shared{}

    #[local]
    struct Local {
        candriver: CanDriver<Spim<SPIM1>, Pin<Output<PushPull>>>,
    }

    #[init]
    fn init(mut cx: init::Context) -> (Shared, Local){
        defmt::info!("init");
        let device = cx.device;
        let port0 = gpio::p0::Parts::new(device.P0);
        let port1 = gpio::p1::Parts::new(device.P1);

        let clk = Clocks::new(device.CLOCK)
            .enable_ext_hfosc()
            .start_lfclk();
      
        /* Disable shared peripheral addresses */
        device.SPI1.enable.write(|w| w.enable().disabled());
        device.SPIS1.enable.write(|w| w.enable().disabled());
        device.TWIM1.enable.write(|w| w.enable().disabled());
        device.TWIS1.enable.write(|w| w.enable().disabled());
        device.TWI1.enable.write(|w| w.enable().disabled());
       
        /* PINS: 0.17, 0.19, 0.20, 0.21, 0.22, 0.23 is for flash memory pins, need to cut it? */

        defmt::println!("Initialize the SPI instance, and CanDriver");
        let pins = nrf52840_hal::spim::Pins{
            sck: Some(port0.p0_05.into_push_pull_output(Level::Low).degrade()),
            mosi: Some(port0.p0_06.into_push_pull_output(Level::Low).degrade()),
            miso: Some(port0.p0_07.into_floating_input().degrade()),
        };
        
        let cs_pin = port1.p1_08.into_push_pull_output(Level::High).degrade(); 
         
        //let mut spi = Spi::new(device.SPI0, pins, Frequency::M1, MODE_0);
        let mut spim = Spim::new(device.SPIM1, pins, nrf52840_hal::spim::Frequency::M8, spim::MODE_0, 0);
         
        let dummy_data = b"dummy data test";
        let test_vec = *dummy_data;
        let mut byte_msg: [u8; 3] = [0x02, 0x0F, 0x00];
        let mut read_buf = [0; 255];

        const OSM: bool = false;
        const ABAT: bool = false; 

        let canctrl_settings = CanControllSettings::new(OperationTypes::Configuration, false, CLKPRE::DIV1, ABAT, OSM);
             
        let can_settings = CanSettings{
            canctrl: canctrl_settings,
            mcp_clk: 0,
            can_bitrate: 0, 
        };

        let mut can_driver = CanDriver::init(spim, cs_pin, can_settings);
        
        defmt::println!("After initializing Spim<SPIM1>...");
        can_driver.loopback_test();

        (
            Shared {},
            Local {candriver: can_driver},
        )
    }

    #[idle]
    fn idle(cx: idle::Context) -> ! {
        defmt::info!("idle");
        //buffer_readwrite::spawn().unwrap();
        
        loop {
            asm::wfi(); 
        }
    }

    #[task(local = [candriver], priority = 1)]
    async fn buffer_readwrite(cx: buffer_readwrite::Context){
        let dummy_data = b"dummy data test";
        let test_vec = *dummy_data;
        let mut byte_msg: [u8; 3] = [0x02, 0x0F, 0x00];
        let mut read_buf = [0; 255];

        cx.local.candriver.transfer(&mut read_buf, &byte_msg);
        cx.local.candriver.spi.transfer_split_uneven(&mut cx.local.candriver.cs, &test_vec, &mut read_buf);
    }
}

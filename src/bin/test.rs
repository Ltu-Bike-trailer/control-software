#![no_main]
#![no_std]
#![deny(unsafe_code)]
#![deny(warnings)]
#![deny(unused_imports)]


/*/
    TODO:
    Switch form systic to rtc
    get button to work
    potencometer to change hz of led
    bug fixes
*/
mod time_struct;

use {
    hal::gpio::{ Pin, PushPull}, 
    nrf52840_hal as hal,
    rtic_monotonics::systick::fugit::TimerInstantU64,
};

const TIMER_HZ: u32 = 4; // 4 Hz (250 ms granularity)
const TIME_0: Instant = TimerInstantU64::from_ticks(0); // Constant for time zero
type Constant = Pin<hal::gpio::Output<PushPull>>;
type Instant = TimerInstantU64<TIMER_HZ>;
//type Duration = MillisDurationU64;
//type Button = Pin<hal::gpio::Input<hal::gpio::PullUp>>;


#[rtic::app(device = nrf52840_hal::pac, dispatchers= [TIMER0, UARTE1])]
mod app {
    //use rtt_target::rprint;

    //use hal::pac::{NVIC, SAADC};

    use nrf52840_hal::{clocks::LFCLK_FREQ, gpio::{p0::P0_03, Floating, Input, Level}, gpiote::Gpiote, rtc::{RtcCompareReg, RtcInterrupt}, saadc::SaadcConfig, Clocks, Rtc};
    use rtic_monotonics::systick::Systick;

    use self::time_struct::TimeStruct;

    use super::*;

    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick;

    #[shared]
    struct Shared {
        gpiote: Gpiote,
        blink_flag: bool,
        wake_flag: bool,
        pot_var: f32,
        buzzer: Constant,
        led1: Constant,
        rtc: Rtc<RTC0>,
        time_container: TimeStruct,
        
    }

    #[local]
    struct Local {
        //btn1: Button,
        saadc: hal::saadc::Saadc,
        saadc_pin: P0_03<Input<Floating>>,
       // saadc_last_read: f32,
        //_pot_out: Constant,
        //led1: Constant,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local) {
        rtt_init_print!();
        rprintln!("\n---init---\n");


       // Initialize Clock
        let clocks = Clocks::new(cx.device.CLOCK);
        let _clocks = clocks.start_lfclk();

        // Initialize RTC0
        let mut rtc = Rtc::new(cx.device.RTC0, 0).unwrap();
        rtc.enable_event(RtcInterrupt::Compare0);
        rtc.enable_interrupt(RtcInterrupt::Compare0, None);
        rtc.set_compare(RtcCompareReg::Compare0, LFCLK_FREQ).unwrap(); 
        rtc.enable_counter();

        // set date
        let time_container = time_struct::TimeStruct::new(16,11,0);


        // Initialize the monotonic (core clock at 64 MHz)
        let mono = Systick::new(cx.core.SYST, 64_000_000);

        // Initialize Ports
        let p0 = hal::gpio::p0::Parts::new(cx.device.P0);
        let p1 = hal::gpio::p1::Parts::new(cx.device.P1);

        // Initialize Components
        // button
        
        let btn1 = p0.p0_18.into_pullup_input().degrade();

        // led
        let led1 = p0.p0_14.into_push_pull_output(Level::High).degrade();
        // buzzer
        let buzzer = p1.p1_09.into_push_pull_output(Level::Low).degrade();

        // potenciometer
        let _pot_out = p0.p0_31.into_push_pull_output(Level::High).degrade();
 

        // SAADC

        //let saadc_config = SaadcConfig::default();

        #[allow(unused_mut)]
        let mut saadc = hal::Saadc::new(cx.device.SAADC, SaadcConfig::default());

        #[allow(unused_mut)]
        let mut saadc_pin =  p0.p0_03.into_floating_input();
       // let channel_config = SaadcChannelConfig::single_ended(&saadc_pin);

        /* 
        let saadc_config = SaadcConfig::default();
        let saadc = Saadc::new(cx.device.SAADC, saadc_config);
        let mut saadc_pin = p0.p0_03.into_floating_input();
        //let saadc_last_read = saadc.read(&mut saadc_pin).unwrap() as f32;
        */
       
        // Interrupts
        let gpiote = Gpiote::new(cx.device.GPIOTE);
        gpiote.channel0().input_pin(&btn1).hi_to_lo().enable_interrupt();
       // gpiote.channel1().input_pin(&pot_out).toggle().enable_interrupt();
        gpiote.port().enable_interrupt();

        // set varibles
        let wake_flag = false;
        let blink_flag = true;
        let pot_var= 1.0;

        rprintln!("\n---init successful---\n");

        // Initiate periodic process
        //wake::spawn_at(next_instant, next_instant).unwrap();

        (Shared {
            gpiote, blink_flag, pot_var, wake_flag, buzzer, led1, rtc, time_container, 
        },
         Local { 
            saadc, saadc_pin
        }, 
         init::Monotonics(mono) )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        rprintln!("idle");
        loop {
            // Puts the device into sleep.
            // However Systick requires the core clock of the MCU to be active
            // Thus we will get about 1.5mA
            asm::wfi();
            //rprintln!("wake");
        }
    }

    #[task(priority=1, binds = RTC0, shared = [rtc, time_container])]
    fn time(mut cx: time::Context){
        rprintln!("The current time is: ");

        cx.shared.rtc.lock(|rtc|{
            cx.shared.time_container.lock(|time_container|{

                time_container.add_sec();
                let current_time = time_container.get_time();
                rprintln!("{} : {} : {}", 
                        current_time.0, current_time.1, current_time.2);
                

                saadc_reader::spawn_after(50.millis()).ok();

                // starts wake at 9 a clock
                if current_time == (16,11,5){
                    //rprintln!("rtc works!");
                    let next_instant = TIME_0;
                    wake::spawn_at(next_instant, next_instant).unwrap();
    
                }
                rtc.reset_event(RtcInterrupt::Compare0);
                rtc.clear_counter();
            })

        })
    }


    #[task(priority=1, local = [cnt: u32 = 0], shared = [pot_var, blink_flag, led1])]
    async fn blink(mut cx: blink::Context, instant: Instant) {
       
        
        cx.shared.led1.lock(|led1|{
            // turns led on and of depending on counter
            if *cx.local.cnt % 2 == 0 {
                led1.set_high().ok();
            } else {
                led1.set_low().ok();
            }
            *cx.local.cnt += 1;
            
        
            // spawns new blink if flag is true
            cx.shared.blink_flag.lock(|blink_flag|{
                let var = *blink_flag;
                if var == true {
                    let mut var_ptr = 0 as f32;
                    cx.shared.pot_var.lock(|pot_var|{
                        if *pot_var <= 25.0 {
                            var_ptr = 4.0;
                        }else if *pot_var <= 50.0 && *pot_var > 25.0 {
                            var_ptr = 2.0;
                        }else if *pot_var <= 75.0 && *pot_var > 50.0 {
                            var_ptr = 1.5;
                        } else {
                            var_ptr = 1.0;
                        }
                    }); 
                    let time_to_int =(1000.0/var_ptr) as u32;
                    let next_time = fugit::ExtU32::millis(time_to_int);
                    let next_instant = instant + next_time;

                    blink::spawn_at(next_instant, next_instant).unwrap();
                
                }else{
                    led1.set_high().ok();
                    *cx.local.cnt = 1;
                }
            });
        });
    }


    // wake
    #[task(priority=1, shared = [blink_flag, wake_flag,  buzzer])]
    async fn wake(mut cx: wake::Context, instant: Instant){
        rprintln!("starting WAKE");

        // set blink flag and next instant
        cx.shared.wake_flag.lock(|wake_flag|{
            if *wake_flag == false {
                *wake_flag = true;

                cx.shared.blink_flag.lock(|blink_flag|{
                    if *blink_flag == false {
                        *blink_flag = true;
                    }
                });
                
                // start buzzer
                cx.shared.buzzer.lock(|buzzer|{
                    buzzer.set_high().ok();
                });
                

                // spawn blink
                let next_instant = instant;
                blink::spawn_at(next_instant, next_instant).unwrap();    

            }
        });
         // spawn next instant of wake
      //   let next_instant = instant + 10.secs();
      //   wake::spawn_at(next_instant, next_instant).unwrap();
        
    }


    #[task(priority=1, binds = GPIOTE, shared = [gpiote])]
    fn on_gpiote(mut cx: on_gpiote::Context) {
        cx.shared.gpiote.lock(|gpiote| {
            if gpiote.channel0().is_event_triggered() {
                rprintln!("Interrupt from channel 0 event");
                // start 
                wake_off::spawn_after(50.millis()).ok();
                let _ = wake_off::spawn();
                //saadc_reader::spawn_after(50.millis()).ok();
            }
            if gpiote.port().is_event_triggered() {
                rprintln!("Interrupt from port event");
            }
            // Reset all events
            gpiote.reset_events();

            
        });
    }

    #[task(priority=1, local = [saadc, saadc_pin], shared = [gpiote, pot_var])]
    async fn saadc_reader(mut cx: saadc_reader::Context){
        cx.shared.gpiote.lock(|gpiote|{
            
            let current_read = cx.local.saadc.read(cx.local.saadc_pin).unwrap() as f32;
            let resolution = 16384.0 as f32; // 2^14
            let reference = 3.3*0.25 as f32; // vdd/4 
            let gain = 0.25 as f32; // 1/4 form default settings
            
            // calc voltage
            let g_r = gain/reference;
            let res_g_r = current_read/g_r;
            let res_g_r_res = res_g_r/resolution;

            let curr_volt = res_g_r_res;   
            
            let norm = 0.033 as f32; // theoredical max / 100 (3.3v/100)
            cx.shared.pot_var.lock(|pot_var|{

                *pot_var = curr_volt/norm;
            });
            rprintln!("Current Voltage: {} V, current read: {}, current pot: {}", curr_volt, current_read, curr_volt/norm);
            

            //*cx.local.saadc_last_read = current_read;

            gpiote.reset_events();
        });
    }

    #[task(priority=1,shared = [gpiote, blink_flag, wake_flag, buzzer, led1])]
    async fn wake_off(mut cx: wake_off::Context){
        cx.shared.gpiote.lock(|gpiote|{

            cx.shared.wake_flag.lock(|wake_flag|{
                if *wake_flag == true {
                    *wake_flag = false;

                    cx.shared.blink_flag.lock(|blink_flag|{
                        *blink_flag = false;

                    });
                    cx.shared.led1.lock(|led1|{
                        led1.set_high().ok()

                    });
                    // stop buzzer
                    cx.shared.buzzer.lock(|buzzer|{
                        buzzer.set_low().ok();
                    });
                    rprintln!("Wake off")
                }
            });
            gpiote.reset_events();
        });
        
    }

}

#[panic_handler]
fn panic(_info: &core::panic::PanicInfo) -> ! {
    loop {}
}

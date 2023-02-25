#![no_main]
#![no_std]

//use panic_halt as _;
use panic_persist as _;

#[rtic::app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [SWI0_EGU0])]
mod app {
    use embedded_hal::digital::v2::OutputPin;

    use nrf52840_hal as hal;
    use systick_monotonic::*;
    use hal::{gpio, gpiote};
    use hal::gpio::*;

    #[monotonic(binds = SysTick, default = true)]
    type Mono = Systick<1_000_000>;

    #[shared]
    struct Shared {
    }

    #[local]
    struct Local {
        redled: Pin<Output<PushPull>>,
        redled_state: bool,
        dotstar_clk: Pin<Output<PushPull>>,
        dotstar_dat: Pin<Output<PushPull>>
    }


    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let _clocks = hal::Clocks::new(ctx.device.CLOCK).enable_ext_hfosc();

        let port0 = gpio::p0::Parts::new(ctx.device.P0);
        let port1 = gpio::p1::Parts::new(ctx.device.P1);


        let redled_state = true;
        let redled = port0.p0_06.into_push_pull_output(Level::High).degrade();

        let dotstar_clk = port1.p1_09.into_push_pull_output(Level::Low).degrade();
        let dotstar_dat = port0.p0_08.into_push_pull_output(Level::Low).degrade();

        let mono = Mono::new(ctx.core.SYST, 64_000_000);

        (
            Shared { },
            Local { redled, redled_state, dotstar_clk, dotstar_dat },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            blink_redled::spawn().ok();  // error here just means we did it twice, no biggie
        }
    }

    #[task(local = [redled, redled_state], capacity = 1)]  // capacity = 1 to make sure it's only blinking once
    fn blink_redled(ctx: blink_redled::Context) {
        if *ctx.local.redled_state {
            ctx.local.redled.set_low().expect("led setting failed!");
            *ctx.local.redled_state = false;
        } else {
            ctx.local.redled.set_high().expect("led setting failed!");
            *ctx.local.redled_state = true;
            blink_redled::spawn_after(200.millis()).ok();
        }
    }

    #[task(local = [dotstar_clk, dotstar_dat])]
    fn set_dotstar_color(ctx: set_dotstar_color::Context,
                         rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8) {
        let firstbyte = brightness | 0b11100000;  // anything > 31 is effectively treated as max
        
        let txbuffer = [0, 0, 0, 0,
                        firstbyte, bbyte, gbyte, rbyte,
                        0xff, 0xff, 0xff, 0xff
                        ];

        ctx.local.dotstar_clk.set_high().ok();
        cortex_m::asm::delay(200_u32);


        for b in txbuffer.iter() {
            for i in 0..8 {
                ctx.local.dotstar_clk.set_low().ok();
                if 0b10000000 & (b << i) == 0 { 
                    ctx.local.dotstar_dat.set_low().ok();
                } else {
                    ctx.local.dotstar_dat.set_high().ok();
                }
                cortex_m::asm::delay(3_u32);
                ctx.local.dotstar_clk.set_high().ok();
                cortex_m::asm::delay(3_u32);
            }
        }
        ctx.local.dotstar_clk.set_low().ok();
    }
}

// use nrf52840_hal as hal; 

// // //use embedded_hal::blocking::spi::*;
// // //use embedded_hal::digital::v2::OutputPin;

// use panic_halt as _;
// //use panic_persist as _;

// use hal::prelude::*;

// use hal::pac::{Peripherals, CorePeripherals};
// use hal::gpio;
// use hal::gpio::*;
// use hal::gpio::p0::*;

// #[cortex_m_rt::entry]
// fn main() -> ! {
//     //defmt::info!("Hello, World!");

//     let core = CorePeripherals::take().unwrap();
//     let peripherals = Peripherals::take().unwrap();

//     let mut delay = hal::delay::Delay::new(core.SYST);

//     let port0 = p0::Parts::new(peripherals.P0);
//     let port1 = p1::Parts::new(peripherals.P1);

//     let mut redled: P0_06<gpio::Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::High);

//     let mut dotstar_clk = port1.p1_09.into_push_pull_output(Level::Low).degrade();
//     let mut dotstar_dat = port0.p0_08.into_push_pull_output(Level::Low).degrade();

//     set_dotstar_color(0, 0, 100u8, 255u8,
//                       &mut dotstar_dat, &mut dotstar_clk);

//     loop {
//         redled.set_high().expect("led set failed");
//         delay.delay_ms(100u16);
//         redled.set_low().expect("led set failed");
//         delay.delay_ms(100u16);
//     }
// // }

// fn set_dotstar_color(rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8, 
//     dat: &mut Pin<Output<PushPull>>, clk: &mut Pin<Output<PushPull>>) {
//     let firstbyte = brightness | 0b11100000;  // anything > 31 is effectively treated as max
    
//     let txbuffer = [0, 0, 0, 0,
//                     firstbyte, bbyte, gbyte, rbyte,
//                     0xff, 0xff, 0xff, 0xff
//                     ];

//     clk.set_high().ok();
//     cortex_m::asm::delay(200_u32);


//     for b in txbuffer.iter() {
//         for i in 0..8 {
//             clk.set_low().ok();
//             if 0b10000000 & (b << i) == 0 { 
//                 dat.set_low().ok();
//             } else {
//                 dat.set_high().ok();
//             }
//             cortex_m::asm::delay(3_u32);
//             clk.set_high().ok();
//             cortex_m::asm::delay(3_u32);
//         }
//     }
//     clk.set_low().ok();
// }

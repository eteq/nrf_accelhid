#![no_main]
#![no_std]

mod monotonic_nrf52;
use crate::monotonic_nrf52::MonoTimer;

//use panic_halt as _;
use panic_persist as _;
use panic_persist::get_panic_message_bytes;

use embedded_hal::digital::v2::OutputPin;

use nrf52840_hal as hal;
use hal::{gpio, uarte};
use hal::gpio::*;
use fugit::*;

use core::fmt::Write;


#[rtic::app(device = nrf52840_hal::pac, peripherals = true, dispatchers = [SWI0_EGU0, SWI1_EGU1])]
mod app {
    use super::*;

    // #[monotonic(binds = SysTick, default = true)]
    // type MyMono = Systick<1_000_000>;
    #[monotonic(binds = TIMER1, default = true)]
    type Tonic = MonoTimer<hal::pac::TIMER1>;

    #[shared]
    struct Shared {
        uart: uarte::Uarte<hal::pac::UARTE0>
    }

    #[local]
    struct Local {
        redled: Pin<Output<PushPull>>,
        redled_state: bool,
        dotstar_clk: Pin<Output<PushPull>>,
        dotstar_dat: Pin<Output<PushPull>>
    }


    #[init]
    fn init(mut ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        let clocks = hal::Clocks::new(ctx.device.CLOCK).set_lfclk_src_external(hal::clocks::LfOscConfiguration::NoExternalNoBypass);
        clocks.start_lfclk();

        // needed for the monotonic timer (CYCCNT) ?
        //ctx.core.DCB.enable_trace();
        //ctx.core.DWT.enable_cycle_counter();

        let port0 = gpio::p0::Parts::new(ctx.device.P0);
        let port1 = gpio::p1::Parts::new(ctx.device.P1);

        let mut uart = uarte::Uarte::new(
            ctx.device.UARTE0,
            uarte::Pins {
                txd: port0.p0_24.into_push_pull_output(gpio::Level::High).degrade(),
                rxd: port0.p0_25.into_floating_input().degrade(),
                cts: None,
                rts: None,
            },
            uarte::Parity::EXCLUDED,
            uarte::Baudrate::BAUD115200,
        );

        // Check if there was a panic message, if so, send to UART
        if let Some(msg) = get_panic_message_bytes() {
            write!(uart, "Panic message found:\r\n").unwrap();
            uart.write(msg).unwrap();
            write!(uart, "\r\n").unwrap();
        } else {

            write!(uart, "No Panic message on startup.\r\n").unwrap();
        }

        let mut redled_state = true;
        let mut redled = port0.p0_06.into_push_pull_output(Level::High).degrade();

        let mut dotstar_clk = port1.p1_09.into_push_pull_output(Level::Low).degrade();
        let mut dotstar_dat = port0.p0_08.into_push_pull_output(Level::Low).degrade();

        set_dotstar_colorf(0,0,200u8,200u8, &mut dotstar_dat, &mut dotstar_clk);

        //let mono = MyMono::new(ctx.core.SYST, 64_000_000);
        let mono = MonoTimer::new(ctx.device.TIMER1);
        testtask::spawn().unwrap();
        (
            Shared { uart },
            Local { redled, redled_state, dotstar_clk, dotstar_dat },
            init::Monotonics(mono),
        )
    }

    #[idle(shared=[uart])]
    fn idle(_ctx: idle::Context) -> ! {
        //set_dotstar_color::spawn_after(1500u32.millis(), 0, 200u8, 0, 200u8).ok();
        //testtask2::spawn_after(50u32.millis());
        loop {
            //blink_redled::spawn().ok();  // error here just means we did it twice, no biggie
            cortex_m::asm::wfi();
        }
    }
    #[task(local = [redled], priority = 3)]
    fn testtask(ctx: testtask::Context) {
        ctx.local.redled.set_low().expect("led setting failed!");
    }

    #[task]
    fn testtask2(_ctx: testtask2::Context) {
        cortex_m::asm::delay(50);
    }

    // #[task(local = [redled, redled_state], capacity = 3)]  // capacity = 1 to make sure it's only blinking once
    // fn blink_redled(ctx: blink_redled::Context) {
    //     if *ctx.local.redled_state {
    //         ctx.local.redled.set_low().expect("led setting failed!");
    //         *ctx.local.redled_state = false;
    //     } else {
    //         ctx.local.redled.set_high().expect("led setting failed!");
    //         *ctx.local.redled_state = true;
    //         blink_redled::spawn_after(200u32.millis()).ok();
    //     }
    // }

    // #[task(local = [dotstar_clk, dotstar_dat])]
    // fn set_dotstar_color(ctx: set_dotstar_color::Context,
    //                      rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8) {
    //     let firstbyte = brightness | 0b11100000;  // anything > 31 is effectively treated as max
        
    //     let txbuffer = [0, 0, 0, 0,
    //                     firstbyte, bbyte, gbyte, rbyte,
    //                     0xff, 0xff, 0xff, 0xff
    //                     ];

    //     ctx.local.dotstar_clk.set_high().ok();
    //     cortex_m::asm::delay(200_u32);


    //     for b in txbuffer.iter() {
    //         for i in 0..8 {
    //             ctx.local.dotstar_clk.set_low().ok();
    //             if 0b10000000 & (b << i) == 0 { 
    //                 ctx.local.dotstar_dat.set_low().ok();
    //             } else {
    //                 ctx.local.dotstar_dat.set_high().ok();
    //             }
    //             cortex_m::asm::delay(3_u32);
    //             ctx.local.dotstar_clk.set_high().ok();
    //             cortex_m::asm::delay(3_u32);
    //         }
    //     }
    //     ctx.local.dotstar_clk.set_low().ok();
    // }
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

fn set_dotstar_colorf(rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8, 
    dat: &mut Pin<Output<PushPull>>, clk: &mut Pin<Output<PushPull>>) {
    let firstbyte = brightness | 0b11100000;  // anything > 31 is effectively treated as max
    
    let txbuffer = [0, 0, 0, 0,
                    firstbyte, bbyte, gbyte, rbyte,
                    0xff, 0xff, 0xff, 0xff
                    ];

    clk.set_high().ok();
    cortex_m::asm::delay(200_u32);


    for b in txbuffer.iter() {
        for i in 0..8 {
            clk.set_low().ok();
            if 0b10000000 & (b << i) == 0 { 
                dat.set_low().ok();
            } else {
                dat.set_high().ok();
            }
            cortex_m::asm::delay(3_u32);
            clk.set_high().ok();
            cortex_m::asm::delay(3_u32);
        }
    }
    clk.set_low().ok();
}

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_nrf::gpio::{Level, Output, OutputDrive};
use embassy_nrf::peripherals::P0_06;
use embassy_time::{Duration, Timer};
use {defmt_rtt as _, panic_persist as _}; // global logger

#[embassy_executor::task]
async fn blinker(mut led: Output<'static, P0_06>, interval: Duration) {
    loop {
        led.set_high();
        Timer::after(interval).await;
        led.set_low();
        Timer::after(interval).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let led = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
    unwrap!(spawner.spawn(blinker(led, Duration::from_millis(300))));
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

// fn set_dotstar_colorf(rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8, 
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

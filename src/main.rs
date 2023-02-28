#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};

use embassy_nrf;
use embassy_nrf::gpio::*;
use embassy_nrf::peripherals::*;
use embassy_nrf::{interrupt, uarte};



use defmt_rtt as _; // global logger
use panic_persist;

#[embassy_executor::task]
async fn dotstar_toggler(mut dat: Output<'static, AnyPin>, mut clk: Output<'static, AnyPin>, interval: Duration) {
    loop {
        set_dotstar_color(0, 255u8, 0, 10u8, &mut dat, &mut clk);
        Timer::after(interval).await;
        set_dotstar_color(0, 0, 255u8, 10u8, &mut dat, &mut clk);
        Timer::after(interval).await;
    }
}

#[embassy_executor::task]
async fn blinker(mut redled: Output<'static, P0_06>, interval: Duration) {
    loop {
        redled.set_high();
        Timer::after(interval).await;
        redled.set_low();
        Timer::after(interval).await;
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let p = embassy_nrf::init(Default::default());

    let mut uart_config = uarte::Config::default();
    uart_config.parity = uarte::Parity::EXCLUDED;
    uart_config.baudrate = uarte::Baudrate::BAUD115200;
    let uart_irq = interrupt::take!(UARTE0_UART0);
    let mut uart = uarte::Uarte::new(p.UARTE0, uart_irq, p.P0_25, p.P0_24, uart_config);

    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        unwrap!(uart.write(b"Panic message on boot!:\r\n").await);
        unwrap!(uart.write(msg).await);
        unwrap!(uart.write(b"\r\n").await);
    } else {
        unwrap!(uart.write(b"No panic error message on boot.\r\n").await);
    }


    let dotstar_dat = Output::new(p.P0_08.degrade(), Level::Low, OutputDrive::Standard);
    let dotstar_clk = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
    unwrap!(spawner.spawn(dotstar_toggler(dotstar_dat, dotstar_clk, Duration::from_millis(300))));


    let redled = Output::new(p.P0_06, Level::Low, OutputDrive::Standard);
    unwrap!(spawner.spawn(blinker(redled, Duration::from_millis(300))));
    
}

fn set_dotstar_color(rbyte:u8, gbyte:u8, bbyte:u8, brightness:u8, 
    dat: &mut Output<'static, AnyPin>, clk: &mut Output<'static, AnyPin>) {
    let firstbyte = brightness | 0b11100000;  // anything > 31 is effectively treated as max
    
    let txbuffer = [0, 0, 0, 0,
                    firstbyte, bbyte, gbyte, rbyte,
                    0xff, 0xff, 0xff, 0xff
                    ];

    clk.set_high();
    cortex_m::asm::delay(200_u32);


    for b in txbuffer.iter() {
        for i in 0..8 {
            clk.set_low();
            if 0b10000000 & (b << i) == 0 { 
                dat.set_low();
            } else {
                dat.set_high();
            }
            cortex_m::asm::delay(3_u32);
            clk.set_high();
            cortex_m::asm::delay(3_u32);
        }
    }
    clk.set_low();
}

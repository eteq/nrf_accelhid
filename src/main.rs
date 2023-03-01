#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

use defmt::*;
use embassy_executor::Spawner;
//use embassy_time::{Duration, Timer};

use embassy_nrf;
use embassy_nrf::gpio::*;
use embassy_nrf::{interrupt, uarte, config};



use defmt_rtt as _; // global logger
use panic_persist;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut init_config = config::Config::default();
    init_config.lfclk_source = config::LfclkSource::ExternalXtal;
    let p = embassy_nrf::init(init_config);

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


    let mut dotstar_dat = Output::new(p.P0_08.degrade(), Level::Low, OutputDrive::Standard);
    let mut dotstar_clk = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
    set_dotstar_color(0, 0, 0, 0, &mut dotstar_dat, &mut dotstar_clk);
    let mut redled = Output::new(p.P0_06, Level::High, OutputDrive::Standard);

    // action

    redled.set_low();
    
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

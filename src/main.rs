#![no_main]
#![no_std]

use nrf52840_hal as hal; 

//use embedded_hal::blocking::spi::*;
//use embedded_hal::digital::v2::OutputPin;

use panic_halt as _;
//use panic_persist as _;

use hal::prelude::*;

use hal::pac::{Peripherals, CorePeripherals};
use hal::gpio;
use hal::gpio::*;
use hal::gpio::p0::*;

#[cortex_m_rt::entry]
fn main() -> ! {
    //defmt::info!("Hello, World!");

    let core = CorePeripherals::take().unwrap();
    let peripherals = Peripherals::take().unwrap();

    let mut delay = hal::delay::Delay::new(core.SYST);

    let port0 = p0::Parts::new(peripherals.P0);
    let port1 = p1::Parts::new(peripherals.P1);

    let mut redled: P0_06<gpio::Output<PushPull>> = port0.p0_06.into_push_pull_output(Level::High);

    let dotstar_clk = port1.p1_09.into_push_pull_output(Level::Low).degrade();
    let dotstar_mosi = port0.p0_08.into_push_pull_output(Level::Low).degrade();
    let dotstar_miso = port1.p1_08.into_floating_input().degrade();  // not actually needed


    let pins = hal::spim::Pins {
        sck: Some(dotstar_clk),
        miso: Some(dotstar_miso),
        mosi: Some(dotstar_mosi),
    };
    let mut dotstar_spi = hal::spim::Spim::new(
        peripherals.SPIM2,
        pins,
        hal::spim::Frequency::K500,
        hal::spim::MODE_0,
        0,
    );

    loop {
        redled.set_high();
        delay.delay_ms(100);
        redled.set_low();
        delay.delay_ms(100);
    }
}


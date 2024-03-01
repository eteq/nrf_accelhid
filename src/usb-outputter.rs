#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]


use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_nrf::interrupt::QSPI;
use embassy_nrf::pac::UART0;
use embassy_nrf::qspi::Qspi;
use embassy_nrf::uarte::{UarteTx, UarteRx};
use embassy_time::{Timer, Delay, Instant, Duration};

use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_nrf;
use embassy_nrf::gpio::*;
use embassy_nrf::{bind_interrupts, interrupt, uarte, config, qspi, peripherals, twim};

use numtoa::NumToA;

use panic_persist;

mod mpu6050;


bind_interrupts!(struct Irqs {
    SPIM0_SPIS0_TWIM0_TWIS0_SPI0_TWI0 => twim::InterruptHandler<peripherals::TWISPI0>;
    UARTE0_UART0 => uarte::InterruptHandler<peripherals::UARTE0>;
    QSPI => qspi::InterruptHandler<peripherals::QSPI>;
});

const FLASH_PAGE_SIZE: usize = 256;
// Workaround for alignment requirements in flash writing
#[repr(C, align(4))]
struct AlignedPageBuf([u8; FLASH_PAGE_SIZE]);

static BUTTON_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();
static DUMP_SIGNAL: Signal<CriticalSectionRawMutex, u32> = Signal::new();


#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut init_config = config::Config::default();
    init_config.lfclk_source = config::LfclkSource::ExternalXtal;
    init_config.hfclk_source = config::HfclkSource::Internal;
    // don't collide with softdevice
    init_config.gpiote_interrupt_priority = embassy_nrf::interrupt::Priority::P2;
    init_config.time_interrupt_priority = embassy_nrf::interrupt::Priority::P3;
    let p = embassy_nrf::init(init_config);

    let mut redled = Output::new(p.P0_06, Level::High, OutputDrive::Standard);

    Timer::after(Duration::from_millis(100)).await;
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
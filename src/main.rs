#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![allow(unused_imports)]
#![allow(dead_code)]

use embassy_executor::Spawner;
use embassy_time::{Timer, Instant, Duration};

use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_nrf;
use embassy_nrf::gpio::*;
use embassy_nrf::{interrupt, uarte, config, qspi, peripherals};

use numtoa::NumToA;

use panic_persist;

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

    let mut uart_config = uarte::Config::default();
    uart_config.parity = uarte::Parity::EXCLUDED;
    uart_config.baudrate = uarte::Baudrate::BAUD115200;
    let uart_irq = interrupt::take!(UARTE0_UART0);
    let mut uart = uarte::Uarte::new(p.UARTE0, uart_irq, p.P0_25, p.P0_24, uart_config);

    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        uart.write(b"Panic message on boot!:\r\n").await.unwrap();
        uart.write(msg).await.unwrap();
        uart.write(b"\r\n").await.unwrap();
    } else {
        uart.write(b"No panic error message on boot.\r\n").await.unwrap();
    }
    let (mut tx_uart, rx_uart) = uart.split();

    let mut dotstar_dat = Output::new(p.P0_08.degrade(), Level::Low, OutputDrive::Standard);
    let mut dotstar_clk = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
    set_dotstar_color(0, 0, 0, 0, &mut dotstar_dat, &mut dotstar_clk);

    let mut config = qspi::Config::default();
    config.read_opcode = qspi::ReadOpcode::READ4O;
    config.write_opcode = qspi::WriteOpcode::PP4O;
    config.write_page_size = qspi::WritePageSize::_256BYTES;
    config.frequency = qspi::Frequency::M16;
    config.address_mode = qspi::AddressMode::_24BIT;

    let qspi_irq = interrupt::take!(QSPI);
    let mut qspi: qspi::Qspi<_, 67108864> = qspi::Qspi::new(
        p.QSPI, qspi_irq, 
        p.P0_19, p.P0_23, //sck, cs
        p.P0_21, p.P0_22, p.P1_00, p.P0_17, //io 0,1,2,3
        config
    );

    let mut mid = [1 ; 3];
    qspi.blocking_custom_instruction(0x9F, &[], &mut mid).unwrap();
    if mid[0] != 0xc8 || mid[1] != 0x40 || mid[2] != 0x15 {
        panic!("Flash ID is incorrect!");
    }

    // example for writing code
    // let mut buf = AlignedPageBuf([0u8; FLASH_PAGE_SIZE]);
    // qspi.blocking_erase(0);
    // qspi.blocking_write(0, &buf.0);
    // let mut rbuf = [1 ; 4];
    // qspi.blocking_read(0, &mut rbuf).unwrap();

    // Goal interface: 
    // * dotstar starts off, redled on until init finishes
    // * dotstar green while waiting for input
    // * press button: erase (dotar red), record data (dotstar blue), next button press signals stop recording (dotstar back to green)
    // * UART receives "d" : ignore if recording, otherwise dump flash (dotstar yellow)

    let button = Input::new(p.P0_29.degrade(), Pull::Up);
    BUTTON_SIGNAL.reset();
    spawner.spawn(button_task(button)).unwrap();

    DUMP_SIGNAL.reset();
    spawner.spawn(read_dump(rx_uart)).unwrap();

    redled.set_low();

    loop {
        set_dotstar_color(0, 255, 0, 5, &mut dotstar_dat, &mut dotstar_clk); //green -> waiting for input

        if DUMP_SIGNAL.signaled() {
            set_dotstar_color(255, 200, 0, 5, &mut dotstar_dat, &mut dotstar_clk); 

            let mut numtoa_buffer = [0u8; 40];
            let mut flash_data = [0u8; 4];

            qspi.blocking_read(0, &mut flash_data).unwrap();
            let ndata = usize::from_ne_bytes(flash_data);

            for i in 0..ndata {
                qspi.read(FLASH_PAGE_SIZE + i*4, &mut flash_data).await.unwrap();
                let towrite_bytes = u32::from_ne_bytes(flash_data).numtoa(10, &mut numtoa_buffer);
                tx_uart.blocking_write(towrite_bytes).unwrap();
                tx_uart.blocking_write(b"\r\n").unwrap();
            }

            DUMP_SIGNAL.reset();
        }
        if BUTTON_SIGNAL.signaled() {
            let emptybuf = [0u8 ;0];
            let mut emptymbuf = [0u8 ;0];
            let mut status = [0u8 ;1];

            qspi.blocking_custom_instruction(0x05, &emptybuf, &mut status).unwrap();  //reads the fist status register - bit 0 is WIP
            while status[0] & 1 == 1 {  // if WIP, keep reading until its not
                qspi.blocking_custom_instruction(0x05, &emptybuf, &mut status).unwrap();
            }
            

            tx_uart.blocking_write(b"Erasing flash\r\n").unwrap();
            set_dotstar_color(255, 0, 0, 5, &mut dotstar_dat, &mut dotstar_clk); 
            qspi.blocking_custom_instruction(0x60, &emptybuf, &mut emptymbuf).unwrap();  //chip erase - WEN is set automatically by blocking_custom_instruction
            
            // wait for chip erase to finish...
            qspi.blocking_custom_instruction(0x05, &emptybuf, &mut status).unwrap();  //reads the fist status register - bit 0 is WIP
            while status[0] & 1 == 1 {
                qspi.blocking_custom_instruction(0x05, &emptybuf, &mut status).unwrap();
            }
            tx_uart.blocking_write(b"Flash erased!\r\n").unwrap();
            
            set_dotstar_color(255, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 
            // erase finished, set up recording
            // TODO

            // start recording
            // TODO

            // record length
            let lenbuf: [u8; 4] = 234_u32.to_ne_bytes();  // note this is a count of *bytes* not bits
            qspi.blocking_write(0, &lenbuf).unwrap(); // first page is always dedicated to just the length

            set_dotstar_color(0, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 

            BUTTON_SIGNAL.reset(); // serves to ignore any presses during the operation
        }
        Timer::at(Instant::from_ticks(0)).await;
    }
    
}


#[embassy_executor::task]
async fn button_task(mut pin: Input<'static, AnyPin>) {
    loop {
        pin.wait_for_low().await;
        Timer::after(Duration::from_millis(5)).await;  // debounce period
        pin.wait_for_high().await;

        BUTTON_SIGNAL.signal(1);

        Timer::after(Duration::from_millis(5)).await;  // debounce period
    }
}

#[embassy_executor::task]
async fn read_dump(mut rx: uarte::UarteRx<'static, peripherals::UARTE0>) {
    let mut buf = [0; 1];
    loop {
        rx.read(&mut buf).await.unwrap();
        if buf[0] == b'd' {
            DUMP_SIGNAL.signal(1);
        }
    }
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

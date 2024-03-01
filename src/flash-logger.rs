#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]


// These are here to allow easy swaping between the recording and reporting loop options
#![allow(unused_imports)]
#![allow(dead_code)]

use embassy_executor::Spawner;
use embassy_nrf::qspi::Qspi;
use embassy_nrf::uarte::UarteTx;
use embassy_time::{Timer, Instant, Duration};

use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_nrf;
use embassy_nrf::gpio::*;
use embassy_nrf::{bind_interrupts, uarte, config, qspi, peripherals, twim};

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

    let mut uart_config = uarte::Config::default();
    uart_config.parity = uarte::Parity::EXCLUDED;
    uart_config.baudrate = uarte::Baudrate::BAUD115200;
    let mut uart = uarte::Uarte::new(p.UARTE0, Irqs, p.P0_25, p.P0_24, uart_config);

    if let Some(msg) = panic_persist::get_panic_message_bytes() {
        uart.write(b"Panic message on boot!:\r\n").await.unwrap();
        uart.write(msg).await.unwrap();
        uart.write(b"\r\n").await.unwrap();
    } else {
        uart.write(b"No panic error message on boot.\r\n").await.unwrap();
    }
    let (tx_uart, rx_uart) = uart.split();

    let mut dotstar_dat = Output::new(p.P0_08.degrade(), Level::Low, OutputDrive::Standard);
    let mut dotstar_clk = Output::new(p.P1_09.degrade(), Level::Low, OutputDrive::Standard);
    set_dotstar_color(0, 0, 0, 0, &mut dotstar_dat, &mut dotstar_clk);

    let mut config = qspi::Config::default();
    config.capacity = 2 * 1024 * 1024; // 2 MB
    config.read_opcode = qspi::ReadOpcode::READ4O;
    config.write_opcode = qspi::WriteOpcode::PP4O;
    config.write_page_size = qspi::WritePageSize::_256BYTES;
    config.frequency = qspi::Frequency::M16;
    config.address_mode = qspi::AddressMode::_24BIT;

    let mut qspi = qspi::Qspi::new(
        p.QSPI, Irqs, 
        p.P0_19, p.P0_23, //sck, cs
        p.P0_21, p.P0_22, p.P1_00, p.P0_17, //io 0,1,2,3
        config
    );

    let mut mid = [1 ; 3];
    qspi.blocking_custom_instruction(0x9F, &[], &mut mid).unwrap();
    if mid[0] != 0xc8 || mid[1] != 0x40 || mid[2] != 0x15 {
        panic!("Flash ID is incorrect!");
    }


    let mut twim_config = twim::Config::default();
    twim_config.frequency = twim::Frequency::K400;
    let mut twim0: twim::Twim<'_, peripherals::TWISPI0> = twim::Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_14, twim_config);
    mpu6050::setup(&mut twim0, 0x68);
    mpu6050::setup(&mut twim0, 0x69);

    let button = Input::new(p.P0_29.degrade(), Pull::Up);
    BUTTON_SIGNAL.reset();
    spawner.spawn(button_task(button)).unwrap();

    DUMP_SIGNAL.reset();
    spawner.spawn(read_dump(rx_uart)).unwrap();

    redled.set_low();

    
    reporting_loop(dotstar_dat, dotstar_clk, tx_uart, qspi, twim0).await;
    //recording_loop(dotstar_dat, dotstar_clk, tx_uart, qspi, twim0).await;
    
}

async fn reporting_loop(mut dotstar_dat: Output<'static, AnyPin>, 
                        mut dotstar_clk: Output<'static, AnyPin>, 
                        mut tx_uart: UarteTx<'static, peripherals::UARTE0>, 
                        mut _qspi: Qspi<'static, peripherals::QSPI>, 
                        mut twim0: twim::Twim<'static, peripherals::TWISPI0>) {
    // Goal interface: 
    // * dotstar starts off, redled on until init finishes
    // * blue when recording, flash green when writing to uart
    
    loop {
        //recording loop
        set_dotstar_color(0, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk);
        
        let data1 = mpu6050::read_latest(&mut twim0, 0x68).await;
        let data2 = mpu6050::read_latest(&mut twim0, 0x69).await;
        
        set_dotstar_color(0, 255, 0, 5, &mut dotstar_dat, &mut dotstar_clk);

        let mut data_bytes = [0u8; 2*4*2];
        let mut j = 0usize;
        for data in [data1, data2] {
            // extract just the quaternion components
            for i in 0..4 {
                data_bytes[i*2 + j*8] = data[i*2];
                data_bytes[i*2 + 1 + j*8] = data[i*2 + 1];
            }
            j += 1;
        }
        tx_uart.blocking_write(&data_bytes).expect("writing failed, cannot continue!");
        tx_uart.blocking_write(b"\r\n").expect("writing failed, cannot continue!");
    }
}

async fn recording_loop(mut dotstar_dat: Output<'static, AnyPin>, 
                        mut dotstar_clk: Output<'static, AnyPin>, 
                        mut tx_uart: UarteTx<'static, peripherals::UARTE0>, 
                        mut qspi: Qspi<'static, peripherals::QSPI>, 
                        mut twim0: twim::Twim<'static, peripherals::TWISPI0>) {
    // Goal interface: 
    // * dotstar starts off, redled on until init finishes
    // * dotstar green while waiting for input
    // * press button: erase (dotar red), record data (dotstar blue), next button press signals stop recording (dotstar back to green)
    // * UART receives "d" : ignore if recording, otherwise dump flash (dotstar yellow)

    loop {
        set_dotstar_color(0, 255, 0, 5, &mut dotstar_dat, &mut dotstar_clk); //green -> waiting for input

        if DUMP_SIGNAL.signaled() {
            set_dotstar_color(255, 200, 0, 5, &mut dotstar_dat, &mut dotstar_clk); 

            let mut numtoa_buffer = [0u8; 40];
            let mut ndata_buffer = [0u8; 4];
            let mut data_buffer = [0u8; 8];

            qspi.blocking_read(0, &mut ndata_buffer).unwrap();
            let ndata = u32::from_ne_bytes(ndata_buffer);

            for i in (0..ndata).step_by(8) {
                qspi.read(i + FLASH_PAGE_SIZE as u32, &mut data_buffer).await.unwrap();
                for j in 0..4 {
                    let towrite_bytes = i16::from_be_bytes([data_buffer[2*j], data_buffer[2*j+1]]).numtoa(10, &mut numtoa_buffer);
                    tx_uart.blocking_write(towrite_bytes).unwrap();
                    if j < 3 { tx_uart.blocking_write(b",").unwrap(); }
                }
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
            
            // erase finished, recording data
            


            //TODO: recording loop
            set_dotstar_color(0, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 
            BUTTON_SIGNAL.reset();

            let mut pagenum: u32 = 1;
            let mut data_to_flash = [0; FLASH_PAGE_SIZE];  // important assumption: page size/8/2 is even!
            let mut bytes_in_page = 0;
            
            tx_uart.blocking_write(b"Starting data loop!\r\n").unwrap();
            loop {
                let data1 = mpu6050::read_latest(&mut twim0, 0x68).await;
                let data2 = mpu6050::read_latest(&mut twim0, 0x69).await;

                for data in [data1, data2] {
                    // extract just the quaternion components
                    for i in 0..4 {
                        data_to_flash[bytes_in_page] = data[i*4];
                        data_to_flash[bytes_in_page + 1] = data[i*4+1];
                        bytes_in_page += 2;
                    }
                }
                if bytes_in_page >= FLASH_PAGE_SIZE {
                    set_dotstar_color(255, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 
                    qspi.write(pagenum*FLASH_PAGE_SIZE as u32, &mut data_to_flash).await.unwrap();
                    set_dotstar_color(0, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 
                    pagenum += 1;
                    bytes_in_page = 0;
                }

                Timer::after(Duration::from_millis(10)).await;
                if BUTTON_SIGNAL.signaled() { break; }
            }

            // take care of the last page if we didn't end on a page boundary
            if bytes_in_page != 0 {
                for i in bytes_in_page..data_to_flash.len() {
                    data_to_flash[i] = 0;
                }

                set_dotstar_color(255, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 
                qspi.write(pagenum*FLASH_PAGE_SIZE as u32, &mut data_to_flash).await.unwrap();
                set_dotstar_color(0, 0, 255, 5, &mut dotstar_dat, &mut dotstar_clk); 
            }



            // record length
            let ndata = (pagenum as usize - 1)*FLASH_PAGE_SIZE+ bytes_in_page;
            let lenbuf: [u8; 4] = ndata.to_ne_bytes();  // note this is a count of *bytes* not bits
            qspi.blocking_write(0, &lenbuf).unwrap(); // first page is always dedicated to just the length

            tx_uart.blocking_write(b"Completed data recording!\r\n").unwrap();
            BUTTON_SIGNAL.reset(); // serves to ignore any presses during the above operation
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
        } else if buf[0] == b's' {
            BUTTON_SIGNAL.signal(1);
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

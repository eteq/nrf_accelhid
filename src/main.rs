#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]

#![allow(unused_imports)]
#![allow(dead_code)]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embassy_executor::Spawner;
use embassy_time::{Timer, Delay, Instant, Duration};

use embassy_sync::signal::Signal;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;

use embassy_nrf;
use embassy_nrf::gpio::*;
use embassy_nrf::{bind_interrupts, interrupt, uarte, config, qspi, peripherals, twim};

use numtoa::NumToA;

use panic_persist;

mod dmp_firmware;


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
    let (mut tx_uart, rx_uart) = uart.split();

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
    let mut twim0 = twim::Twim::new(p.TWISPI0, Irqs, p.P0_16, p.P0_14, twim_config);
    mpu6050_setup(&mut twim0, 0x68);
    mpu6050_setup(&mut twim0, 0x69);

    mpu6050_test_panic(&mut twim0, 0x68, false);

    let button = Input::new(p.P0_29.degrade(), Pull::Up);
    BUTTON_SIGNAL.reset();
    spawner.spawn(button_task(button)).unwrap();

    DUMP_SIGNAL.reset();
    spawner.spawn(read_dump(rx_uart)).unwrap();

    redled.set_low();

    
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
            let mut flash_data = [0u8; 4];

            qspi.blocking_read(0, &mut flash_data).unwrap();
            let ndata = u32::from_ne_bytes(flash_data);

            for i in 0..ndata {
                qspi.read(i*4 + FLASH_PAGE_SIZE as u32, &mut flash_data).await.unwrap();
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
            let ndata = 234u32;

            // record length
            let lenbuf: [u8; 4] = ndata.to_ne_bytes();  // note this is a count of *bytes* not bits
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

fn mpu6050_setup<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8) {
    let mut reg_buffer = [0; 1];
    twim.blocking_write_read(address, &[0x75], &mut reg_buffer).unwrap();
    if reg_buffer[0] != 0x68 { panic!("no/invalid response from MPU6050! {}", reg_buffer[0]); }

    // device reset
    twim.blocking_write(address, &[107, 0b10000000]).unwrap();
    Delay.delay_ms(100u32);
    // singal path reset - not clear if this is necessary but maybe recommended in register map doc?
    twim.blocking_write(address, &[104, 0b00000111]).unwrap();
    Delay.delay_ms(100u32);

    twim.blocking_write(address, &[56,  0b00000000]).unwrap(); // no interrupts
    twim.blocking_write(address, &[35,  0b00000000]).unwrap(); // Turn off regular FIFO to use DMP FIFO instead
    twim.blocking_write(address, &[28,  0b00000000]).unwrap(); // 2g full scale accelerometer
    twim.blocking_write(address, &[55,  0b10010000]).unwrap(); //logic level for int pin low, and clear int status on any read MAY NOT BE NEEDED?
    twim.blocking_write(address, &[107, 0b00000001]).unwrap(); // Clock to PLL with X axis gyroscope reference, dont sleep
    twim.blocking_write(address, &[26,  0b00000001]).unwrap(); // no external synchronization, set the DLPF to 184/188 Hz bandwidth
    twim.blocking_write(address, &[25,  0b00000100]).unwrap(); // divides the sample rate - yields 200hz I think combined with above?
	
    mpu6050_write_firmware(twim, address, true);
    // set DMP Program Start Address 0x0400
    twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_DMP_PROG_START, 0x04, 0x00]).unwrap(); 
   
    twim.blocking_write(address, &[27,  0b00011000]).unwrap(); // set gyro to +2000 Deg/sec full range
    twim.blocking_write(address, &[106, 0b11001100]).unwrap(); // turn on the fifo and reset it, and also turn on and reset the DMP (bits 7,3, respectively)
    twim.blocking_write(address, &[56,  0b00000010]).unwrap(); // now turn on the RAW_DMP_INT_EN since the DMP is now set up
  
    twim.blocking_write_read(address, &[0x75], &mut reg_buffer).unwrap();
    if reg_buffer[0] != 0x68 { panic!("no/invalid response from MPU6050 after reset/setup! {}", reg_buffer[0]); }
}

fn mpu6050_write_firmware<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8, verify: bool) {
    // Logic currently wrong!  Need to manually update the bank it turns out


    // set the bank to 0
    twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, 0]).unwrap();
    // and the start address
    twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, 0]).unwrap();

    let mut i = 0;
    let mut write_buffer = [dmp_firmware::MPU6050_REGADDR_MEM_R_W; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE+1];
    while i < dmp_firmware::DMP_FIRMWARE.len() {
        let mut chunk_size = dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE;

        // write the last chunk without extra bytes
        if (i + chunk_size) > dmp_firmware::DMP_FIRMWARE.len() { chunk_size = dmp_firmware::DMP_FIRMWARE.len() - i; }

        for j in 0..chunk_size { write_buffer[j+1] = dmp_firmware::DMP_FIRMWARE[i+j]; }
        twim.blocking_write(address, &write_buffer[0..(chunk_size+1)]).unwrap();
        
        i += chunk_size;
    }

    if verify {
        //reset to bank and start address of 0 
        twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, 0]).unwrap();
        twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, 0]).unwrap();
        
        let mut verify_buffer = [0; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE];
        i = 0;
        while i < dmp_firmware::DMP_FIRMWARE.len() {
            twim.blocking_write_read(address, &[dmp_firmware::MPU6050_REGADDR_MEM_R_W],  
                                    &mut verify_buffer).unwrap();

            for j in 0..verify_buffer.len() {
                if (i+j) < dmp_firmware::DMP_FIRMWARE.len() {
                    if dmp_firmware::DMP_FIRMWARE[i+j] != verify_buffer[j] {
                        panic!("verification of firmware failed at {}! {} vs {}", i + j, dmp_firmware::DMP_FIRMWARE[i+j], verify_buffer[j]);
                    }
                }
            }
            i += verify_buffer.len();

        }
    }
}

fn mpu6050_reset_fifo<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8) {
    let mut reg_buffer = [0; 1];
    // first read the USER_CTRL register
    twim.blocking_write_read(address, &[106], &mut reg_buffer).unwrap();
    twim.blocking_write(address, &[106, reg_buffer[0] | 0b00000100]).unwrap();  // write back the same thing but with the fifo reset bit set
}

fn mpu6050_get_fifo_count<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8) -> u16 {
    let mut rd_buffer = [0; 2];
    // first read the USER_CTRL register
    twim.blocking_write_read(address, &[114], &mut rd_buffer).unwrap();
    ((rd_buffer[0] as u16) << 8) | (rd_buffer[1] as u16)
}

// entirely for testing that it actually produces a good fifo output
fn mpu6050_test_panic<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8, basic_fifo: bool) {
    
    if basic_fifo {
        twim.blocking_write(address, &[35,  0b01111000]).unwrap(); //regular fifo on for accel/gyro
    }

    mpu6050_reset_fifo(twim, address);
    let mut count = mpu6050_get_fifo_count(twim, address);
    let mut nzeros: usize = 0;
    while count == 0 {
        count = mpu6050_get_fifo_count(twim, address);
        //panic!("in 0 count");
        nzeros += 1;
    }

    panic!("reached non-zero count {}, after {} zeros", count, nzeros);
    
    //end test
}



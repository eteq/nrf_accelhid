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
            let mut ndata_buffer = [0u8; 4];
            let mut data_buffer = [0u8; 8];

            qspi.blocking_read(0, &mut ndata_buffer).unwrap();
            let ndata = u32::from_ne_bytes(ndata_buffer);

            for i in (0..ndata).step_by(8) {
                qspi.read(i + FLASH_PAGE_SIZE as u32, &mut data_buffer).await.unwrap();
                for j in 0..4 {
                    let towrite_bytes = u16::from_be_bytes([data_buffer[2*j], data_buffer[2*j+1]]).numtoa(10, &mut numtoa_buffer);
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
                let data1 = mpu6050_read_latest(&mut twim0, 0x68).await;
                let data2 = mpu6050_read_latest(&mut twim0, 0x69).await;

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
    let nbanks = dmp_firmware::DMP_FIRMWARE.len() / dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE + 
        if dmp_firmware::DMP_FIRMWARE.len() % dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE == 0 {0} else {1};

    for b in 0..nbanks {
        let bank_size = if b == (nbanks - 1) {
            // last block
            dmp_firmware::DMP_FIRMWARE.len() - (nbanks-1) * dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE
        } else {
            dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE
        };

        // set the bank
        twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, b as u8]).unwrap();

        // now write the bank one chunk at a time
        for i in (0..bank_size).step_by(dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE) {
            let chunk_size = if (i + dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE) > bank_size {
                    bank_size - i
                } else {
                    dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE
                };
            let mut write_buffer = [dmp_firmware::MPU6050_REGADDR_MEM_R_W; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE+1];
            for j in 0..chunk_size {
                write_buffer[j + 1] = dmp_firmware::DMP_FIRMWARE[i + j + b*dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE];
            }
            // set the start address then write
            twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, i as u8]).unwrap();
            twim.blocking_write(address, &write_buffer[0..(chunk_size+1)]).unwrap();
        }

    }

    if verify {
        let mut verify_buffer = [0; dmp_firmware::MPU6050_DMP_MEMORY_CHUNK_SIZE];
        let mut i = 0;
        let mut last_bank = 255;
        while i < dmp_firmware::DMP_FIRMWARE.len() {
            let bank = (i / dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE) as u8;
            if bank != last_bank {
                //set to new bank 
                twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_BANK_SEL, bank]).unwrap();
            }

            twim.blocking_write(address, &[dmp_firmware::MPU6050_REGADDR_MEM_START_ADDR, (i % dmp_firmware::MPU6050_DMP_MEMORY_BANK_SIZE) as u8]).unwrap();
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
            last_bank = bank;
        }
    }
}

fn mpu6050_reset_fifo<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8) {
    let mut reg_buffer = [0; 1];
    // first read the USER_CTRL register
    twim.blocking_write_read(address, &[106], &mut reg_buffer).unwrap();
    twim.blocking_write(address, &[106, reg_buffer[0] | 0b00000100]).unwrap();  // write back the same thing but with the fifo reset bit set
}

async fn mpu6050_get_fifo_count<T: twim::Instance>(twim: &mut twim::Twim<'_, T>, address: u8) -> u16 {
    let mut rd_buffer = [0; 2];
    // first read the USER_CTRL register
    twim.write_read(address, &[114], &mut rd_buffer).await.unwrap();
    ((rd_buffer[0] as u16) << 8) | (rd_buffer[1] as u16)
}

// entirely for testing that it actually produces a good fifo output
fn mpu6050_test_panic<T: twim::Instance>(twim: &mut twim::Twim<T>, address: u8, basic_fifo: bool) {
    
    if basic_fifo {
        twim.blocking_write(address, &[35,  0b01111000]).unwrap(); //regular fifo on for accel/gyro
    }

    mpu6050_reset_fifo(twim, address);
    let mut count = 0;
    let mut nzeros: usize = 0;
    while count == 0 {
        let mut rd_buffer = [0; 2];
        // first read the USER_CTRL register
        twim.blocking_write_read(address, &[114], &mut rd_buffer).unwrap();
        count = ((rd_buffer[0] as u16) << 8) | (rd_buffer[1] as u16);
        //panic!("in 0 count");
        nzeros += 1;
    }

    if count >= 5 {
        let mut fifo = [0; 5];
        twim.blocking_write_read(address, &[116], &mut fifo).unwrap();
        panic!("reached non-zero count {}, after {} zeros. First 5: {},{},{},{},{}", count, nzeros, fifo[0], fifo[1], fifo[2], fifo[3], fifo[4]);
    } else {
        panic!("reached non-zero count {}, after {} zeros", count, nzeros);

    }
}


async fn mpu6050_read_latest<T: twim::Instance>(twim: &mut twim::Twim<'_, T>, address: u8) -> [u8; dmp_firmware::DMP_PACKET_SIZE] {
    mpu6050_reset_fifo(twim, address);

    let mut count = mpu6050_get_fifo_count(twim, address).await;
    while count < 28 {
        count = mpu6050_get_fifo_count(twim, address).await;
    }
    let mut fifo = [0; dmp_firmware::DMP_PACKET_SIZE];
    twim.write_read(address, &[116], &mut fifo).await.unwrap();
    fifo
}

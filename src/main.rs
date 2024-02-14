#![no_std]
#![no_main]
#![deny(clippy::pedantic)]
#![allow(unused_imports)]

// use crate::i2s_process::{I2cPlayer, TSamples};
use core::borrow::BorrowMut;
use core::fmt::Write;
use core::mem;
use core::ptr::slice_from_raw_parts;
use cortex_m::peripheral::NVIC;
use defmt::{error, info, println, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_rp::config::Config;
use embassy_rp::gpio::Output;
// use embassy_nrf::config::Config;
// use embassy_nrf::gpio::Output;
// use embassy_nrf::pac::USBD;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{self, Channel, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::delay::DelayNs;
use heapless::{String, Vec};

use {defmt_rtt as _, panic_probe as _};

use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;
use fixed::traits::ToFixed;

// mod i2s_process;

use embassy_rp::peripherals::{DMA_CH0, PIO0, USB};
use embassy_rp::pio::{
    Config as PIOConfig, FifoJoin, InterruptHandler as PioInterruptHandler, Pio, ShiftConfig,
    ShiftDirection,
};
use embassy_rp::usb::{Driver, Instance, InterruptHandler};
use embassy_rp::{bind_interrupts, peripherals, Peripheral as _, PeripheralRef};

bind_interrupts!(struct Irqs {
    PIO0_IRQ_0 => PioInterruptHandler<PIO0>;
    USBCTRL_IRQ => InterruptHandler<USB>;
});

use static_cell::StaticCell;
use usb_audio_class::{AudioClass, State};

mod audio;
mod usb_audio_class;

// const SAMPLES_1K_HZ: [u16; 48] = [
//     0x0000, 0x10b5, 0x2120, 0x30fb, 0x4000, 0x4deb, 0x5a82, 0x658c, 0x6ed9, 0x7641, 0x7ba3, 0x7ee7,
//     0x7fff, 0x7ee7, 0x7ba3, 0x7641, 0x6ed9, 0x658c, 0x5a82, 0x4deb, 0x3fff, 0x30fb, 0x2120, 0x10b5,
//     0x0000, 0xef4b, 0xdee0, 0xcf05, 0xc000, 0xb215, 0xa57e, 0x9a74, 0x9127, 0x89bf, 0x845d, 0x8119,
//     0x8000, 0x8119, 0x845d, 0x89bf, 0x9127, 0x9a74, 0xa57e, 0xb215, 0xc001, 0xcf05, 0xdee0, 0xef4b,
// ];

#[repr(align(8))]
struct SampleBuffer(pub [u32; 50]);

impl SampleBuffer {
    pub fn new() -> Self {
        Self([0; 50])
    }

    pub fn as_mut_ptr(&mut self) -> &mut [u8] {
        unsafe { self.0.align_to_mut::<u8>().1 }
    }
}

type Usb = Driver<'static, peripherals::USB>;

#[embassy_executor::task]
async fn usb_runner(mut usb: embassy_usb::UsbDevice<'static, Usb>) {
    loop {
        info!("init usb runner");
        usb.run().await;
    }
}

// Create embassy-usb DeviceBuilder using the driver and config.
// It needs some buffers for building the descriptors.
// static EP_OUT_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
// static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static AUDIO_STATE: StaticCell<State> = StaticCell::new();
// static SANPLE_BUF: StaticCell<Channel<NoopRawMutex, TSamples, 10>> = StaticCell::new();

// const SAMPLES: usize = 48 * 2 * 2 / 2 * 4;

// static SAMPLE_DMABUF: StaticCell<[u16; SAMPLES]> = StaticCell::new();

#[derive(Debug, defmt::Format)]
enum I2SStatus {
    Waiting,
    Buffering(u8),
    Running,
}

const BUFFER_SIZE: usize = 48;
static DMA_BUFFER: StaticCell<[u32; BUFFER_SIZE * 2]> = StaticCell::new();

#[embassy_executor::task]
async fn usb_samples_task(
    mut uac: AudioClass<'static, Usb>,
    //samples: Sender<'static, NoopRawMutex, TSamples, 10>,
    // mut i2s: I2S<'static, peripherals::SPI3, peripherals::DMA1_CH7, u16>,
    mut pio: Pio<'static, PIO0>,
    mut dma_ref: PeripheralRef<'static, DMA_CH0>,
    mut status_pin: Output<'static>,
) {
    let mut status;
    let mut packet_buf = SampleBuffer::new();
    let mut cnt = 1;
    let mut total = 0;
    // let (mut tx, mut rx) = uac.split();

    let dma_buffer = DMA_BUFFER.init_with(|| [0u32; BUFFER_SIZE * 2]);
    let (mut back_buffer, mut front_buffer) = dma_buffer.split_at_mut(BUFFER_SIZE);

    // start pio state machine
    pio.sm0.set_enable(true);
    let tx = pio.sm0.tx();

    loop {
        info!("Wait for USB Audio samples");
        status = I2SStatus::Waiting;
        status_pin.set_high();
        // i2s.stop();
        // uac.write_packet(&[0x01, 0x2, 0x0]).await;
        uac.wait_connection().await;

        loop {
            // i2s.start();
            let dma_future = tx.dma_push(dma_ref.reborrow(), front_buffer);
            match uac.read_packet(packet_buf.as_mut_ptr()).await {
                Ok(n) => {
                    status_pin.toggle();
                    total += n;
                    let buf_orig = &packet_buf.0[0..n / 4];
                    cnt -= 1;
                    let mut rem = 0;

                    for (s, sample) in back_buffer.iter_mut().zip(buf_orig) {
                        *s = *sample;
                    }
                    //rem = i2s.write(buf_orig).await;
                    if cnt == 0 {
                        info!(
                            "GOT: {} BL {} S {} T {} R {}",
                            n,
                            buf_orig.len(),
                            status,
                            total,
                            rem
                        );
                        cnt = 1000;
                        total = 0;
                    }
                    // match &mut status {
                    //     I2SStatus::Waiting => status = I2SStatus::Buffering(1),
                    //     I2SStatus::Buffering(n) => {
                    //         *n -= 1;
                    //         if *n == 0 {
                    //             status = I2SStatus::Running;
                    //         }
                    //     }
                    //     I2SStatus::Running => (),
                    // }

                    // now await the dma future. once the dma finishes, the next buffer needs to be queued
                    // within DMA_DEPTH / SAMPLE_RATE = 8 / 48000 seconds = 166us
                    dma_future.await;
                    core::mem::swap(&mut back_buffer, &mut front_buffer);
                }
                Err(e) => {
                    error!("Read: {:?}", e);
                    break;
                }
            }
        }
    }
}

const SAMPLE_RATE: u32 = 48_000;

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    println!("init");
    let p = embassy_rp::init(Config::default());

    let mut led_status = Output::new(p.PIN_21, embassy_rp::gpio::Level::Low);

    let mut cnt = 0_u8;

    // loop {
    //     led_status.toggle();
    //     println!("loop {:02}", cnt);
    //     Timer::after_millis(1000).await;
    //     cnt += 1;
    // }

    // Setup pio state machine for i2s output
    let mut pio = Pio::new(p.PIO0, Irqs);

    #[rustfmt::skip]
     let pio_program = pio_proc::pio_asm!(
         ".side_set 2",
         "    set x, 14          side 0b01", // side 0bWB - W = Word Clock, B = Bit Clock
         "left_data:",
         "    out pins, 1        side 0b00",
         "    jmp x-- left_data  side 0b01",
         "    out pins 1         side 0b10",
         "    set x, 14          side 0b11",
         "right_data:",
         "    out pins 1         side 0b10",
         "    jmp x-- right_data side 0b11",
         "    out pins 1         side 0b00",
     );

    let bit_clock_pin = p.PIN_18;
    let left_right_clock_pin = p.PIN_19;
    let data_pin = p.PIN_20;

    let data_pin = pio.common.make_pio_pin(data_pin);
    let bit_clock_pin = pio.common.make_pio_pin(bit_clock_pin);
    let left_right_clock_pin = pio.common.make_pio_pin(left_right_clock_pin);

    let status_pin = Output::new(p.PIN_22, embassy_rp::gpio::Level::Low);

    let cfg = {
        let mut cfg = PIOConfig::default();
        cfg.use_program(
            &pio.common.load_program(&pio_program.program),
            &[&bit_clock_pin, &left_right_clock_pin],
        );
        cfg.set_out_pins(&[&data_pin]);
        const BIT_DEPTH: u32 = 16;
        const CHANNELS: u32 = 2;
        let clock_frequency = SAMPLE_RATE * BIT_DEPTH * CHANNELS;
        cfg.clock_divider = (125_000_000. / f64::from(clock_frequency) / 2.).to_fixed();
        cfg.shift_out = ShiftConfig {
            threshold: 32,
            direction: ShiftDirection::Left,
            auto_fill: true,
        };
        // join fifos to have twice the time to start the next dma transfer
        cfg.fifo_join = FifoJoin::TxOnly;
        cfg
    };
    pio.sm0.set_config(&cfg);
    pio.sm0.set_pin_dirs(
        embassy_rp::pio::Direction::Out,
        &[&data_pin, &left_right_clock_pin, &bit_clock_pin],
    );

    // Create the driver, from the HAL.
    let driver = Driver::new(p.USB, Irqs);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-Audio");
    config.serial_number = Some("12345678");
    config.max_power = 100;
    config.max_packet_size_0 = 64;

    // Required for windows compatibility.
    // https://developer.nordicsemi.com/nRF_Connect_SDK/doc/1.9.1/kconfig/CONFIG_CDC_ACM_IAD.html#help
    config.device_class = 0xEF;
    config.device_sub_class = 0x02;
    config.device_protocol = 0x01;
    config.composite_with_iads = false;

    let mut builder = Builder::new(
        driver,
        config,
        DEVICE_DESC.init([0; 256]),
        CONFIG_DESC.init([0; 256]),
        BOS_DESC.init([0; 256]),
        &mut [], // no msos descriptors
        CONTROL_BUF.init([0; 256]),
    );

    let class =
        usb_audio_class::AudioClass::new(&mut builder, AUDIO_STATE.init(State::default()), 200);
    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    unwrap!(spawner.spawn(usb_runner(usb)));
    info!("write done");

    Timer::after(Duration::from_millis(10)).await;

    let dma_ref = p.DMA_CH0.into_ref();

    // unwrap!(spawner.spawn(usb_samples_task(class, sample_chan.sender())));
    //unwrap!(spawner.spawn(usb_samples_task(class, i2s, status_pin)));
    unwrap!(spawner.spawn(usb_samples_task(class, pio, dma_ref, status_pin)));

    let mut cnt: u8 = 0;

    loop {
        led_status.toggle();
        println!("loop {:02x}", cnt);
        Timer::after_millis(1000).await;
        cnt = cnt.wrapping_add(1);
    }
}

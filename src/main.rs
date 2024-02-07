#![no_std]
#![no_main]
#![deny(clippy::pedantic)]
#![allow(unused_imports)]

// use crate::i2s_process::{I2cPlayer, TSamples};
use core::borrow::BorrowMut;
use core::fmt::Write;
use core::ptr::slice_from_raw_parts;
use cortex_m::peripheral::NVIC;
use defmt::{error, info, println, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::dma::word::Word;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{Config as I2CConfig, I2c};
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::spi::{Config as SPIConfig, Polarity, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CaptureCompare16bitInstance;
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{bind_interrupts, i2c, pac, peripherals, usb_otg, Config, Peripheral};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{self, Channel, Sender};
use embassy_sync::mutex::Mutex;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::delay::DelayNs;
use heapless::{String, Vec};

use {defmt_rtt as _, panic_probe as _};

use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

// mod i2s_process;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

use embassy_stm32::i2s::{
    ClockPolarity, Config as I2SConfig, Format, Function, Mode, Standard, I2S,
};
use static_cell::StaticCell;
use usb_audio_class::{AudioClass, State};

mod audio;
mod usb_audio_class;

const SAMPLES_1K_HZ: [u16; 48] = [
    0x0000, 0x10b5, 0x2120, 0x30fb, 0x4000, 0x4deb, 0x5a82, 0x658c, 0x6ed9, 0x7641, 0x7ba3, 0x7ee7,
    0x7fff, 0x7ee7, 0x7ba3, 0x7641, 0x6ed9, 0x658c, 0x5a82, 0x4deb, 0x3fff, 0x30fb, 0x2120, 0x10b5,
    0x0000, 0xef4b, 0xdee0, 0xcf05, 0xc000, 0xb215, 0xa57e, 0x9a74, 0x9127, 0x89bf, 0x845d, 0x8119,
    0x8000, 0x8119, 0x845d, 0x89bf, 0x9127, 0x9a74, 0xa57e, 0xb215, 0xc001, 0xcf05, 0xdee0, 0xef4b,
];

mod timer2;

// #[embassy_executor::task]
// async fn i2s_runner(
//     mut i2s: I2cPlayer<NoopRawMutex, peripherals::SPI3, NoDma, NoDma, peripherals::DMA1_CH7>,
// ) {
//     loop {
//         info!("init i2c runner");
//         i2s.run().await;
//     }
// }

#[repr(align(8))]
struct SampleBuffer(pub [u16; 100]);

impl SampleBuffer {
    pub fn new() -> Self {
        Self([0; 100])
    }

    pub fn as_mut_ptr(&mut self) -> &mut [u8] {
        unsafe { self.0.align_to_mut::<u8>().1 }
    }
}

#[embassy_executor::task]
async fn usb_runner(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB_OTG_FS>>) {
    loop {
        info!("init usb runner");
        usb.run().await;
    }
}

// Create embassy-usb DeviceBuilder using the driver and config.
// It needs some buffers for building the descriptors.
static EP_OUT_BUFFER: StaticCell<[u8; 512]> = StaticCell::new();
static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
// static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static AUDIO_STATE: StaticCell<State> = StaticCell::new();
// static SANPLE_BUF: StaticCell<Channel<NoopRawMutex, TSamples, 10>> = StaticCell::new();

const SAMPLES: usize = 48 * 2 * 2 / 2 * 8;

static SAMPLE_DMABUF: StaticCell<[u16; SAMPLES]> = StaticCell::new();

#[derive(Debug, defmt::Format)]
enum I2SStatus {
    Waiting,
    Buffering(u8),
    Running,
}

#[embassy_executor::task]
async fn usb_samples_task(
    mut uac: AudioClass<'static, Driver<'static, USB_OTG_FS>>,
    //samples: Sender<'static, NoopRawMutex, TSamples, 10>,
    mut i2s: I2S<'static, peripherals::SPI3, peripherals::DMA1_CH7, u16>,
    mut status_pin: Output<'static>,
) {
    let mut status;
    let mut packet_buf = SampleBuffer::new();
    let mut cnt = 1;
    let mut total = 0;

    loop {
        info!("Wait for USB Audio samples");
        status = I2SStatus::Waiting;
        status_pin.set_high();
        i2s.stop();
        uac.wait_connection().await;
        i2s.start();
        loop {
            match uac.read_packet(packet_buf.as_mut_ptr()).await {
                Ok(n) => {
                    status_pin.toggle();
                    total += n;
                    let buf_orig = &packet_buf.0[0..n / 2];
                    cnt -= 1;
                    let mut rem = 0;
                    // rem = i2s.write(buf_orig).await;
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
                }
                Err(e) => {
                    error!("Read: {:?}", e);
                    break;
                }
            }
        }
    }
}

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    let mut config = Config::default();
    {
        use embassy_stm32::rcc::{
            AHBPrescaler, APBPrescaler, Hse, HseMode, Pll, PllMul, PllPDiv, PllPreDiv, PllQDiv,
            PllRDiv, PllSource, Sysclk,
        };
        config.rcc.hse = Some(Hse {
            freq: Hertz(25_000_000),
            mode: HseMode::Oscillator,
        });
        config.rcc.pll_src = PllSource::HSE;
        config.rcc.pll = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL336,
            divp: Some(PllPDiv::DIV4), // 25mhz / 25 * 336 / 4 = 84Mhz.
            divq: Some(PllQDiv::DIV7), // 25mhz / 25 * 336 / 7 = 48Mhz.
            divr: None,
        });

        config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV25,
            // PLLI2SN
            mul: PllMul::MUL384,
            divp: None,
            divq: None,
            // PLLI2SR
            divr: Some(PllRDiv::DIV5),
        });

        config.rcc.ahb_pre = AHBPrescaler::DIV1;
        config.rcc.apb1_pre = APBPrescaler::DIV2;
        config.rcc.apb2_pre = APBPrescaler::DIV1;
        config.rcc.sys = Sysclk::PLL1_P;
        config.enable_debug_during_sleep = true;
    }

    let p = embassy_stm32::init(config);

    let tmr2 = timer2::CCTIM2::new(p.TIM2.into_ref(), timer2::ITR1_RMP::OTG_FS_SOF);

    let mut led_status = Output::new(
        p.PC14,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );

    let status_pin = Output::new(
        p.PC13,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::VeryHigh,
    );

    println!("Setup TIM2");

    info!("Setup I2S");

    let mut i2s_cfg = I2SConfig::default();
    i2s_cfg.format = Format::Data16Channel32;
    i2s_cfg.master_clock = false;
    i2s_cfg.clock_polarity = ClockPolarity::IdleLow;
    i2s_cfg.mode = Mode::Master;
    i2s_cfg.standard = Standard::Philips;
    i2s_cfg.function = Function::Transmit;

    let mut i2s = I2S::new_no_mck(
        p.SPI3,
        p.PB5,      // sd - DAta (Sample Data)
        p.PA15,     // ws - LRCK ( Data L/R )
        p.PB3,      // ck - SCK (Sample clock)
        p.DMA1_CH7, // Chab 0 SPI3_TX
        SAMPLE_DMABUF.init([0; SAMPLES]),
        Hertz(48_000),
        i2s_cfg,
    );

    // let sample_chan = SANPLE_BUF.init(Channel::new());

    // let i2c_play = I2cPlayer::new(i2s, sample_chan.receiver());

    // unwrap!(spawner.spawn(i2s_runner(i2c_play)));

    let mut write: Vec<u16, 96> = Vec::new();

    // Enable TIM2 interrupt
    unsafe { NVIC::unmask(pac::interrupt::TIM2) };
    // unsafe { NVIC::unmask(pac::interrupt::DMA1_STREAM5) };
    // unsafe { NVIC::unmask(pac::interrupt::DMA1_STREAM7) };
    // unsafe { NVIC::unmask(pac::interrupt::SPI3) };

    for sample in SAMPLES_1K_HZ {
        //let sample = sample.wrapping_add(0xEFFF);
        unwrap!(write.push(sample));
        unwrap!(write.push(sample));
    }

    let mut sm = write.iter().cycle();

    i2s.start();

    info!("write I2S");
    for _ in 0u32..1000 {
        let mut buf: Vec<u16, 96> = Vec::new();
        for _ in 0..write.capacity() {
            buf.push(sm.next().copied().unwrap()).unwrap();
        }

        // unwrap!(i2s.writer(&buf));

        // match select(sample_chan.send(buf), Timer::after_millis(1000)).await {
        match select(i2s.write(&buf), Timer::after_millis(1000)).await {
            Either::First(e) => {
                unwrap!(e);
            }
            Either::Second(()) => {
                println!("Timer");
            }
        };
    }

    info!("write I2S Done");

    let mut buf: Vec<u16, 96> = Vec::new();
    for _ in 0..write.capacity() {
        buf.push(0).unwrap();
    }
    unwrap!(i2s.write(&buf).await);
    unwrap!(i2s.write(&buf).await);
    unwrap!(i2s.write(&buf).await);
    unwrap!(i2s.write(&buf).await);

    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = false;
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        EP_OUT_BUFFER.init([0; 512]),
        config,
    );

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
    config.composite_with_iads = true;

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

    // unwrap!(spawner.spawn(usb_samples_task(class, sample_chan.sender())));
    unwrap!(spawner.spawn(usb_samples_task(class, i2s, status_pin)));

    loop {
        // class.wait_connection().await;
        // info!("EP Connected");
        led_status.toggle();

        Timer::after_millis(500).await;

        // //nfo!("write I2S {}", n);
        // for n in 0u32..1000 {
        //     let mut buf: TSamples = Vec::new();
        //     for _ in 0..32 {
        //         buf.push(sm.next().copied().unwrap()).unwrap();
        //     }

        //     //i2s.writer(&write);
        //     match select(sample_chan.send(buf), Timer::after_millis(1000)).await {
        //         Either::First(_) => {}
        //         Either::Second(()) => {
        //             println!("Timer");
        //         }
        //     };
        // }

        // println!("Send");
        // // Start Conv.
        // unwrap!(
        //     i2c_temp
        //         .write(addr, 0x2130_u16.to_be_bytes().as_slice())
        //         .await
        // );

        // println!("wait");

        // Timer::after(Duration::from_secs(1)).await;
        //led_status.toggle();

        // let cnt = tmr2.get_counter();
        // let cc = tmr2.get_cc1();

        // println!("CNT: {} CC: {} diff: {}", cnt, cc, cc.wrapping_sub(old));
        // old = cc;

        // println!("read");
        // unwrap!(
        //     i2c_temp.blocking_write_read(addr, 0xE000_u16.to_be_bytes().as_slice(), &mut buf[0..6]) //.await
        // );

        // let temp = u16::from_be_bytes(buf[0..2].try_into().unwrap());
        // let rh = u16::from_be_bytes(buf[3..5].try_into().unwrap());

        // let temp = -45.0 + 175.0 * f32::from(temp) / 65535.0;
        // let rh = 100.0 * f32::from(rh) / 65535.0;

        // println!("T {} C, RH {} %", temp, rh);
    }
}

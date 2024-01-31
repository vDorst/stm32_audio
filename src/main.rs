#![no_std]
#![no_main]
#![deny(clippy::pedantic)]
#![allow(unused_imports)]

use crate::i2s_process::{I2cPlayer, TSamples};
use core::borrow::BorrowMut;
use core::fmt::Write;
use core::ptr::slice_from_raw_parts;
use cortex_m::peripheral::NVIC;
use defmt::{error, info, println, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{Config as I2CConfig, I2c};
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::spi::{Config as SPIConfig, Spi};
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

mod i2s_process;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

use embassy_stm32::i2s::{Config as I2SConfig, Format, I2S};
use static_cell::StaticCell;
use usb_audio_class::{AudioClass, State};

mod audio;
mod usb_audio_class;

const SAMPLES_1K_HZ: [u16; 48] = [
    32767, 37044, 41248, 45307, 49151, 52715, 55937, 58763, 61144, 63040, 64418, 65254, 65535,
    65254, 64418, 63040, 61144, 58763, 55937, 52715, 49151, 45307, 41248, 37044, 32767, 28490,
    24286, 20227, 16383, 12819, 9597, 6771, 4390, 2494, 1116, 280, 0, 280, 1116, 2494, 4390, 6771,
    9597, 12819, 16383, 20227, 24286, 28490,
];

mod timer2;

#[embassy_executor::task]
async fn i2s_runner(
    mut i2s: I2cPlayer<
        NoopRawMutex,
        peripherals::SPI3,
        peripherals::DMA1_CH7,
        peripherals::DMA1_CH0,
    >,
) {
    loop {
        info!("init i2c runner");
        i2s.run().await;
    }
}

#[repr(align(8))]
struct SampleBuffer(pub [u8; 200]);

impl SampleBuffer {
    pub fn new() -> Self {
        Self([0; 200])
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
static SANPLE_BUF: StaticCell<Channel<NoopRawMutex, TSamples, 10>> = StaticCell::new();

#[embassy_executor::task]
async fn usb_samples_task(
    mut uac: AudioClass<'static, Driver<'static, USB_OTG_FS>>,
    samples: Sender<'static, NoopRawMutex, TSamples, 10>,
) {
    let mut packet_buf = SampleBuffer::new();
    loop {
        match uac.read_packet(&mut packet_buf.0).await {
            Ok(n) => {
                let buf_orig = &packet_buf.0[0..n & 0xFFFE];
                let buf = slice_from_raw_parts(buf_orig.as_ptr().cast::<u16>(), buf_orig.len() / 2);
                let buf = unsafe { &*buf };
                let mut sb: TSamples = Vec::new();
                unwrap!(sb.extend_from_slice(buf));
                // info!("Got: {} len {}", n, sb.len());
                samples.send(sb).await;
            }
            Err(e) => {
                // error!("Read: {:?}", e);
                Timer::after_millis(1).await;
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
            divp: Some(PllPDiv::DIV4), // 25mhz / 25 * 336 / 4 = 48Mhz.
            divq: Some(PllQDiv::DIV7), // 25mhz / 25 * 336 / 7 = 84Mhz.
            divr: None,
        });

        config.rcc.plli2s = Some(Pll {
            prediv: PllPreDiv::DIV25,
            mul: PllMul::MUL192,
            divp: None,
            divq: None,
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
        p.PC13,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );

    println!("Setup TIM2");

    info!("Setup I2S");

    let mut i2s_cfg = I2SConfig::default();
    i2s_cfg.format = Format::Data16Channel16;
    i2s_cfg.master_clock = false;

    let i2s = I2S::new_no_mck(
        p.SPI3,
        p.PB5,      // sd - DAta (Sample Data)
        p.PA15,     // ws - LRCK ( Data L/R )
        p.PB3,      // ck - SCK (Sample clock)
        p.DMA1_CH7, // Chab 0 SPI3_TX
        p.DMA1_CH0,
        Hertz(48_000),
        i2s_cfg,
    );

    let sample_chan = SANPLE_BUF.init(Channel::new());

    let i2c_play = I2cPlayer::new(i2s, sample_chan.receiver());

    unwrap!(spawner.spawn(i2s_runner(i2c_play)));

    let mut write: Vec<u16, 96> = Vec::new();

    // Enable TIM2 interrupt
    unsafe { NVIC::unmask(pac::interrupt::TIM2) };
    // unsafe { NVIC::unmask(pac::interrupt::DMA1_STREAM5) };
    // unsafe { NVIC::unmask(pac::interrupt::DMA1_STREAM7) };
    // unsafe { NVIC::unmask(pac::interrupt::SPI3) };

    for sample in SAMPLES_1K_HZ {
        unwrap!(write.push(sample));
        unwrap!(write.push(sample));
    }

    let mut sm = write.iter().cycle();

    info!("write I2S");
    for _ in 0u32..2000 {
        let mut buf: TSamples = Vec::new();
        for _ in 0..write.capacity() {
            buf.push(sm.next().copied().unwrap()).unwrap();
        }

        // unwrap!(i2s.writer(&buf));

        match select(sample_chan.send(buf), Timer::after_millis(1000)).await {
            Either::First(()) => {}
            Either::Second(()) => {
                println!("Timer");
            }
        };
    }

    info!("write I2S Done");

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

    unwrap!(spawner.spawn(usb_samples_task(class, sample_chan.sender())));

    loop {
        // class.wait_connection().await;
        // info!("EP Connected");
        led_status.toggle();

        Timer::after_millis(5000).await;

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

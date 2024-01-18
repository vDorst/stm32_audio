#![no_std]
#![no_main]
#![deny(clippy::pedantic)]
#![allow(unused_imports)]

use core::borrow::BorrowMut;
use core::fmt::Write;
use cortex_m::peripheral::NVIC;
use defmt::{info, println, unwrap};
use embassy_executor::Spawner;
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{Config as I2CConfig, I2c};
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::spi::{Config as SPIConfig, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::low_level::CaptureCompare16bitInstance;
use embassy_stm32::usb_otg::Driver;
use embassy_stm32::{bind_interrupts, i2c, pac, peripherals, usb_otg, Config, Peripheral};
use embassy_time::{Duration, Timer};
use heapless::{String, Vec};
use {defmt_rtt as _, panic_probe as _};

use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    OTG_FS => usb_otg::InterruptHandler<peripherals::USB_OTG_FS>;
});

use embassy_stm32::i2s::{Config as I2SConfig, Format, I2S};
use static_cell::StaticCell;
use usb_audio_class::State;

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
async fn usb_runner(mut usb: embassy_usb::UsbDevice<'static, Driver<'static, USB_OTG_FS>>) {
    loop {
        info!("init usb runner");
        usb.run().await;
    }
}

// Create embassy-usb DeviceBuilder using the driver and config.
// It needs some buffers for building the descriptors.
static EP_OUT_BUFFER: StaticCell<[u8; 256]> = StaticCell::new();
static DEVICE_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
// static MSOS_DESC: StaticCell<[u8; 128]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static AUDIO_STATE: StaticCell<State> = StaticCell::new();

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

    // Enable TIM2 interrupt
    unsafe { NVIC::unmask(pac::interrupt::TIM2) };

    let mut led_status = Output::new(
        p.PC13,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );

    println!("Setup TIM2");

    //p.TIM2;
    // info!("Setup I2C");
    // let mut i2c_cfg = I2CConfig::default();
    // i2c_cfg.scl_pullup = true;
    // i2c_cfg.sda_pullup = true;
    // i2c_cfg.timeout = Duration::from_millis(100);

    // let mut i2c_temp = I2c::new(
    //     p.I2C1,
    //     p.PB6,
    //     p.PB7,
    //     Irqs,
    //     p.DMA1_CH6,
    //     p.DMA1_CH0,
    //     Hertz(100_000),
    //     i2c_cfg,
    // );

    //let addr = 0x44;

    // // Soft Reset
    // unwrap!(
    //     i2c_temp
    //         .write(addr, 0x30a2_u16.to_be_bytes().as_slice())
    //         .await
    // );

    info!("Setup I2S");

    // let mut spi = Spi::new_txonly(
    //     p.SPI2,
    //     p.PB13,
    //     p.PB15,
    //     p.DMA1_CH4,
    //     NoDma,
    //     SPIConfig::default(),
    // );

    let mut i2s_cfg = I2SConfig::default();
    i2s_cfg.format = Format::Data16Channel16;
    i2s_cfg.master_clock = false;

    let mut i2s = I2S::new_no_mck(
        p.SPI3,
        p.PB5,      // sd - DAta (Sample Data)
        p.PA15,     // ws - LRCK ( Data L/R )
        p.PB3,      // ck - SCK (Sample clock)
        p.DMA1_CH7, // Chab 0 SPI3_TX
        p.DMA1_CH2,
        Hertz(48_000),
        i2s_cfg,
    );

    let mut write: Vec<u16, 96> = Vec::new();

    // for sample in SAMPLES_1K_HZ {
    //     write.push(sample);
    //     write.push(sample);
    // }

    // Create the driver, from the HAL.

    let mut config = embassy_stm32::usb_otg::Config::default();
    config.vbus_detection = false;
    let driver = Driver::new_fs(
        p.USB_OTG_FS,
        Irqs,
        p.PA12,
        p.PA11,
        EP_OUT_BUFFER.init([0; 256]),
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

    let mut class =
        usb_audio_class::AudioClass::new(&mut builder, AUDIO_STATE.init(State::default()), 64);

    // Build the builder.
    let usb = builder.build();

    // Run the USB device.
    unwrap!(spawner.spawn(usb_runner(usb)));

    // info!("write I2S");
    // for _ in 0u32..5000 {
    //     unwrap!(i2s.writer(&write));
    // }

    info!("write done");

    // let mut buf = [0, ];

    Timer::after(Duration::from_millis(10)).await;

    let mut old: u32 = tmr2.get_cc1();

    let mut packet_buf: [u8; 200] = [0; 200];

    loop {
        class.wait_connection().await;
        led_status.toggle();

        if let Ok(n) = class
            .read_packet(&mut packet_buf[0..class.max_packet_size() as usize])
            .await
        {
            println!("Got: {}", n);
        }

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

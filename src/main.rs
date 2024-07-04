#![no_std]
#![no_main]
#![deny(clippy::pedantic)]
#![allow(unused_imports)]

// use crate::i2s_process::{I2cPlayer, TSamples};
use core::borrow::BorrowMut;
use core::fmt::Write;
use core::num::NonZeroU8;
use core::ptr::slice_from_raw_parts;
use cortex_m::peripheral::NVIC;
use defmt::{error, info, println, unwrap};
use embassy_executor::Spawner;
use embassy_futures::select::{select, Either};
use embassy_stm32::dma::word::Word;
use embassy_stm32::dma::NoDma;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::i2c::{Config as I2CConfig, I2c};
use embassy_stm32::spi::{Config as SPIConfig, Polarity, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, i2c, pac, peripherals, Config, Peripheral};
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embassy_sync::channel::{self, Channel, Sender};
use embassy_sync::mutex::Mutex;
use embassy_sync::waitqueue::AtomicWaker;
use embassy_time::{Delay, Duration, Timer};
use embedded_hal::delay::DelayNs;
use heapless::{String, Vec};

use {defmt_rtt as _, panic_probe as _};

use embassy_usb::driver::EndpointError;
use embassy_usb::Builder;

use embassy_stm32::i2s::{ClockPolarity, Config as I2SConfig, Format, Mode, Standard, I2S};
use static_cell::StaticCell;

const SAMPLES: usize = 48 * 2 * 2 / 2 * 8;

static SAMPLE_DMABUF: StaticCell<[u16; SAMPLES]> = StaticCell::new();

const SAMPLES_1K_HZ: [u16; 48] = [
    0x0000, 0x10b5, 0x2120, 0x30fb, 0x4000, 0x4deb, 0x5a82, 0x658c, 0x6ed9, 0x7641, 0x7ba3, 0x7ee7,
    0x7fff, 0x7ee7, 0x7ba3, 0x7641, 0x6ed9, 0x658c, 0x5a82, 0x4deb, 0x3fff, 0x30fb, 0x2120, 0x10b5,
    0x0000, 0xef4b, 0xdee0, 0xcf05, 0xc000, 0xb215, 0xa57e, 0x9a74, 0x9127, 0x89bf, 0x845d, 0x8119,
    0x8000, 0x8119, 0x845d, 0x89bf, 0x9127, 0x9a74, 0xa57e, 0xb215, 0xc001, 0xcf05, 0xdee0, 0xef4b,
];

const SAMPLE_SILENTS: [u16; 48] = [
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
    0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000,
];

#[embassy_executor::main]
async fn main(_spawner: Spawner) {
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
            mul: PllMul::MUL192,
            divp: Some(PllPDiv::DIV4), // 25mhz / 25 * 336 / 4 = 84Mhz.
            divq: Some(PllQDiv::DIV4), // 25mhz / 25 * 336 / 7 = 48Mhz.
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

    let mut led_status = Output::new(
        p.PC13,
        embassy_stm32::gpio::Level::Low,
        embassy_stm32::gpio::Speed::Low,
    );

    let btn_key = Input::new(p.PA0, embassy_stm32::gpio::Pull::Up);

    info!("Setup I2S");

    let mut i2s_cfg = I2SConfig::default();
    i2s_cfg.format = Format::Data16Channel16;
    i2s_cfg.master_clock = false;
    i2s_cfg.clock_polarity = ClockPolarity::IdleLow;
    i2s_cfg.mode = Mode::Master;
    i2s_cfg.standard = Standard::Philips;

    let mut i2s = I2S::new_txonly_nomclk(
        p.SPI3,
        p.PB5,      // sd - DAta (Sample Data)
        p.PA15,     // ws - LRCK ( Data L/R )
        p.PB3,      // ck - SCK (Sample clock)
        p.DMA1_CH7, // Chan 0 SPI3_TX
        SAMPLE_DMABUF.init([0; SAMPLES]),
        Hertz(48_000),
        i2s_cfg,
    );

    let (n, left) = i2s.write_immediate(&SAMPLES_1K_HZ).unwrap();
    println!("i2s: n = {}, left = {}", n, left);

    i2s.start();

    loop {
        let samples = if btn_key.is_low() {
            led_status.set_high();
            &SAMPLES_1K_HZ
        } else {
            led_status.set_low();
            &SAMPLE_SILENTS
        };
        if let Err(err) = i2s.write(samples).await {
            println!("Error: {:?}", err);
        }
    }
}

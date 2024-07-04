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
use embassy_stm32::gpio::Output;
use embassy_stm32::i2c::{Config as I2CConfig, I2c};
use embassy_stm32::peripherals::USB_OTG_FS;
use embassy_stm32::spi::{Config as SPIConfig, Polarity, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::Driver;
use embassy_stm32::{bind_interrupts, i2c, pac, peripherals, usb, Config, Peripheral};
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

// mod i2s_process;

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

use embassy_stm32::i2s::{ClockPolarity, Config as I2SConfig, Format, Mode, Standard, I2S};
use static_cell::StaticCell;
use usb_audio_class::{AudioClass, State, UAC_Status};

mod audio;
mod usb_audio_class;

mod timer2;

#[repr(align(8))]
struct SampleBuffer(pub [u16; 200]);

impl SampleBuffer {
    pub fn new() -> Self {
        Self([0; 200])
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
static CONFIG_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static BOS_DESC: StaticCell<[u8; 256]> = StaticCell::new();
static CONTROL_BUF: StaticCell<[u8; 256]> = StaticCell::new();
static AUDIO_STATE: StaticCell<State> = StaticCell::new();

const SAMPLES: usize = 48 * 2 * 2 / 2 * 8;

static SAMPLE_DMABUF: StaticCell<[u16; SAMPLES]> = StaticCell::new();

#[derive(Debug, defmt::Format)]
enum I2SStatus {
    // Buffering N samples before start the real dma transfer
    Buffering(NonZeroU8),
    Running,
}

#[embassy_executor::task]
async fn usb_samples_task(
    uac: AudioClass<'static, Driver<'static, USB_OTG_FS>>,
    mut i2s: I2S<'static, u16>,
    mut status_pin: Output<'static>,
) {
    let mut status;
    let mut packet_buf = SampleBuffer::new();
    let mut cnt = 1;
    let mut fb_cnt = 0;
    let mut total = 0;

    let (mut tx, mut rx) = uac.split();
    loop {
        info!("Wait for USB Audio samples");
        status_pin.set_high();

        tx.wait_connection().await;
        rx.wait_connection().await;

        error!("Next step");

        status = I2SStatus::Buffering(NonZeroU8::new(1).expect("Should fit!"));

        loop {
            let ret = timer2::SOF.load(core::sync::atomic::Ordering::Relaxed);
            let ret = ret.to_le_bytes();
            cnt += 1;

            match select(
                rx.read_packet(packet_buf.as_mut_ptr()),
                tx.write_packet(&ret[0..3]),
            )
            .await
            {
                Either::First(samples) => match samples {
                    Ok(n) => {
                        status_pin.toggle();
                        total += n;
                        let buf_orig = &packet_buf.0[0..n / 2];
                        // let remain = match &mut status {
                        //     I2SStatus::Running => i2s.write(buf_orig).await,
                        //     I2SStatus::Buffering(n) => {
                        //         // Write samples to buffer
                        //         let ret = i2s.write(buf_orig).await;
                        //         match NonZeroU8::new(u8::from(*n) - 1) {
                        //             None => {
                        //                 status = I2SStatus::Running;
                        //                 // i2s.start();
                        //                 info!("\tStart I2s!\n\n");
                        //             }
                        //             Some(val) => {
                        //                 *n = val;
                        //             }
                        //         }
                        //         ret
                        //     }
                        // };
                        // if remain.is_err() {
                        //     error!("Sample buffer overrun!");
                        // }
                        let remain = 0;
                        if cnt == 1000 {
                            info!(
                                "GOT: {} BL {} S {} T {} R {}",
                                n,
                                buf_orig.len(),
                                status,
                                total,
                                remain
                            );
                            cnt = 0;
                            total = 0;
                        }
                    }
                    Err(e) => {
                        error!("Read: {:?}", e);
                        break;
                    }
                },
                Either::Second(ret) => {
                    fb_cnt += 1;
                    if let Err(e) = ret {
                        error!("Write: {:?}", e);
                        break;
                    }
                }
            }

            if cnt == 1000 {
                println!(
                    "T: {} F {} ret {:02x} {:02x}",
                    total,
                    fb_cnt,
                    ret[0..3],
                    ret
                );
                cnt = 0;
                total = 0;
                fb_cnt = 0;
            }
        }
        // error!("Stop I2s");
        // i2s.stop();
        // error!("Stop I2s DONE");
        // i2s.clear();
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

    let _tmr2 = timer2::CCTIM2::new(p.TIM2.into_ref(), timer2::ITR1_RMP::OTG_FS_SOF);

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

    // i2s_cfg.function = Function::Transmit;

    let i2s = I2S::new_txonly_nomclk(
        p.SPI3,
        p.PB5,      // sd - DAta (Sample Data)
        p.PA15,     // ws - LRCK ( Data L/R )
        p.PB3,      // ck - SCK (Sample clock)
        p.DMA1_CH7, // Chan 0 SPI3_TX
        SAMPLE_DMABUF.init([0; SAMPLES]),
        Hertz(48_000),
        i2s_cfg,
    );

    // let sample_chan = SANPLE_BUF.init(Channel::new());

    // let i2c_play = I2cPlayer::new(i2s, sample_chan.receiver());

    // unwrap!(spawner.spawn(i2s_runner(i2c_play)));

    // Enable TIM2 interrupt
    unsafe { NVIC::unmask(pac::interrupt::TIM2) };
    // unsafe { NVIC::unmask(pac::interrupt::DMA1_STREAM5) };
    // unsafe { NVIC::unmask(pac::interrupt::DMA1_STREAM7) };
    // unsafe { NVIC::unmask(pac::interrupt::SPI3) };

    let mut config = embassy_stm32::usb::Config::default();
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
    config.composite_with_iads = false;

    let mut builder = Builder::new(
        driver,
        config,
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

    // unwrap!(spawner.spawn(usb_samples_task(class, sample_chan.sender())));
    unwrap!(spawner.spawn(usb_samples_task(class, i2s, status_pin)));

    loop {
        //     // class.wait_connection().await;
        //     // info!("EP Connected");
        led_status.toggle();

        Timer::after_millis(500).await;

        //     //nfo!("write I2S {}", n);
        //     for n in 0u32..1000 {
        //         let mut buf: TSamples = Vec::new();
        //         for _ in 0..32 {
        //             buf.push(sm.next().copied().unwrap()).unwrap();
        //         }

        //         //i2s.writer(&write);
        //         match select(sample_chan.send(buf), Timer::after_millis(1000)).await {
        //             Either::First(_) => {}
        //             Either::Second(()) => {
        //                 println!("Timer");
        //             }
        //         };
        //     }

        //     println!("Send");
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

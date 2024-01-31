use core::default;
use defmt::{info, Format};
use embassy_stm32::dma::{Transfer, TransferOptions};
use embassy_stm32::i2s::I2S;
use embassy_stm32::pac::common::W;
use embassy_stm32::spi::TxDma;
use embassy_stm32::{spi::Instance, Peripheral, PeripheralRef};
use embassy_sync::{
    blocking_mutex::raw::RawMutex,
    channel::{Receiver, Sender},
};
use embassy_time::Timer;
use heapless::Vec;

#[derive(Debug, Format, Default)]
enum I2cState {
    #[default]
    Idle,
    Playing,
    Shutdown,
}

pub type TSamples = Vec<u16, 100>;

pub struct I2cPlayer<M: RawMutex + 'static, T: Instance, Tx: 'static, Rx: 'static> {
    samples: Receiver<'static, M, TSamples, 10>,
    state: I2cState,
    pub pheriferal: I2S<'static, T, Tx, Rx>,
}

impl<M: RawMutex, T: Instance, Tx, Rx> I2cPlayer<M, T, Tx, Rx> {
    pub fn new(dev: I2S<'static, T, Tx, Rx>, samples: Receiver<'static, M, TSamples, 10>) -> Self {
        Self {
            samples,
            state: I2cState::default(),
            pheriferal: dev,
        }
    }

    pub async fn run(&mut self)
    where
        Tx: TxDma<T>,
    {
        T::REGS.cr1().modify(|w| {
            w.set_spe(false);
            w.set_dff(embassy_stm32::pac::spi::vals::Dff::SIXTEENBIT);
        });

        let mut buf: Option<TSamples> = None;

        let mut samp_num = 0;
        let mut samp_bytes = 0;

        loop {
            match self.state {
                I2cState::Idle => {
                    info!("Wait for audio samples!");
                    let data = self.samples.receive().await;
                    buf = Some(data);
                    self.state = I2cState::Playing;

                    // setup periheral
                    T::REGS.cr1().modify(|w| {
                        w.set_spe(true);
                    });

                    T::REGS.cr2().modify(|reg| {
                        reg.set_txdmaen(true);
                    });

                    T::REGS.i2scfgr().modify(|r| r.set_i2se(true));

                    Timer::after_millis(5).await;

                    samp_num = 0;
                    samp_bytes = 0;
                }
                I2cState::Playing => {
                    samp_num += 1;
                    let data = buf.take().unwrap();
                    samp_bytes += data.len();
                    let tx_request = self.pheriferal._peri.txdma.request();
                    let tx_dst = T::REGS.dr().as_ptr().cast::<u16>();

                    let tx_f = unsafe {
                        Transfer::new_write(
                            &mut self.pheriferal._peri.txdma,
                            tx_request,
                            &data,
                            tx_dst,
                            TransferOptions::default(),
                        )
                    };

                    tx_f.await;

                    match self.samples.try_receive() {
                        Ok(data) => {
                            buf = Some(data);
                        }
                        Err(_) => self.state = I2cState::Shutdown,
                    }
                }
                I2cState::Shutdown => {
                    info!("Shutdown audio: played = {}, {} B", samp_num, samp_bytes);
                    T::REGS.cr2().modify(|reg| {
                        reg.set_txdmaen(false);
                    });
                    T::REGS.cr1().modify(|w| {
                        w.set_spe(false);
                    });
                    self.state = I2cState::Idle;
                }
            }
        }
    }
}

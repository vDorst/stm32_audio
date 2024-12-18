use core::cell::Cell;
use core::ops::DerefMut;
use core::sync::atomic::AtomicU32;

use defmt::println;
use embassy_stm32::pac::timer::vals::{CcmrInputCcs, Dir, FilterValue};
use embassy_stm32::pac::timer::TimGp32;
use embassy_stm32::pac::RCC;
use embassy_stm32::peripherals::TIM2;
use embassy_stm32::timer::low_level::{InputTISelection, Timer};
use embassy_stm32::timer::{AdvancedInstance1Channel, Channel, Channel1Pin, CoreInstance};
use embassy_stm32::{interrupt, PeripheralRef, Peripherals};
use static_cell::StaticCell;

pub struct CCTIM2(pub Timer<'static, TIM2>);

pub static SOF: AtomicU32 = AtomicU32::new(0);

#[allow(non_camel_case_types, dead_code)]
#[repr(u32)]
/// TIM2: Option register Internal trigger 1 remap
pub enum ITR1_RMP {
    Reserved = 0b00 << 10,
    /// PTP trigger output is connected to `TIM2_ITR1`
    PTP_TRG_OUT = 0b01 << 10,
    /// OTG FS SOF is connected to the `TIM2_ITR1` input
    OTG_FS_SOF = 0b10 << 10,
    /// OTG HS SOF is connected to the `TIM2_ITR1` input
    OTG_HS_SOF = 0b11 << 10,
}

impl CCTIM2 {
    pub fn new(timer2: PeripheralRef<'static, TIM2>, inp: ITR1_RMP) -> Self {
        let timer2 = Timer::new(timer2);

        // Reset timer
        RCC.apb1rstr().modify(|w| w.set_tim2rst(true));
        RCC.apb1rstr().modify(|w| w.set_tim2rst(false));

        // Enable timer
        RCC.apb1enr().modify(|w| w.set_tim2en(true));

        let tmr2 = timer2.regs_gp32();

        println!("tmr2: {:08x}", tmr2.as_ptr());

        tmr2.ccer().modify(|w| w.set_cce(0, false));

        timer2.set_input_ti_selection(Channel::Ch1, InputTISelection::TRC);
        timer2.set_input_capture_prescaler(Channel::Ch1, 0);
        timer2.set_input_capture_filter(Channel::Ch1, FilterValue::FCK_INT_N2);

        // Set TIM2 option register (TIM2_OR): See `RM0368`
        {
            let tmr2_or = tmr2.or().as_ptr().cast::<ITR1_RMP>();
            // SAFETY: This is save because `ITR1_RMP` is also u32.
            unsafe { *tmr2_or = inp };
        }

        // fCK_PSC / (PSC[15:0] + 1)
        tmr2.psc().write_value(0);

        tmr2.smcr().modify(|r| {
            r.set_ece(false);
            r.set_ts(embassy_stm32::pac::timer::vals::Ts::ITR1);
        });

        // Set Input Compate first channel
        tmr2.ccmr_input(0).modify(|w| {
            w.set_icf(0, FilterValue::NOFILTER);
            w.set_ccs(0, CcmrInputCcs::TRC);
            w.set_icpsc(0, 0);
        });

        // Enable CC channel
        tmr2.ccer().modify(|w| w.set_cce(0, true));

        // Enable timer
        tmr2.cr1().modify(|w| w.set_cen(true));

        // Clear status register
        tmr2.sr().write(|r| r.0 = 0x0000_0000);

        // enable interrupt
        timer2.enable_input_interrupt(Channel::Ch1, true);

        CCTIM2(timer2)
    }

    #[allow(dead_code, clippy::unused_self)]
    pub fn get_counter(&self) -> u32 {
        let tmr2 = self.0.regs_gp32();
        tmr2.cnt().read()
    }

    #[allow(dead_code, clippy::unused_self)]
    pub fn get_cc1(&self) -> u32 {
        let tmr2 = self.0.regs_gp32();
        tmr2.ccr(0).read()
    }
}

#[interrupt]
unsafe fn TIM2() {
    static mut VALUES: [u32; 4] = [48000_u32; 4];
    static mut IDX: usize = 0;
    static mut LAST: u32 = 0;

    static mut CNT: usize = 0;

    let tmr2 = unsafe { crate::pac::timer::TimGp32::from_ptr(TIM2::regs()) };

    if tmr2.sr().read().ccif(0) {
        let old = *LAST;
        *LAST = tmr2.ccr(0).read();

        let diff = (*LAST).wrapping_sub(old);
        VALUES[*IDX] = diff;
        *IDX = (*IDX + 1) & 0x3;

        let avg: u32 = VALUES.iter().sum();
        let mul = avg.checked_mul(256).unwrap_or(48000_u32 * 4);

        let fb: u32 = (mul / 1000) << 6;

        if *CNT >= 1000 {
            *CNT = 0;
            let val = unsafe { tmr2.as_ptr().byte_add(0x10) }.cast::<u32>();
            println!(
                "SOF: C {}: A {} {=14..24} + {=0..14}/16384 SR: {:08x}",
                diff,
                avg,
                fb,
                fb,
                unsafe { *val },
            );
        }
        *CNT += 1;

        SOF.store(fb, core::sync::atomic::Ordering::Relaxed);
    } else {
        // let int = tmr2.sr().read();
        let val = unsafe { tmr2.as_ptr().byte_add(0x10) }.cast::<u32>();
        defmt::error!("IRQ TMR2: SR: 0x{:08x}", val);
    }
    // Note: We don't need to clear the timer capture interrupt.
    // It's cleared by hardware by reading the capture register.
}

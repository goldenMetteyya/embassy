//! ADC v1
//!
//!
//! The 12-bit ADC is a successive approximation analog-to-digital converter.
//! It has up to 19 multiplexed channels allowing it to measure signals from 16 external
//! and 3 internal sources. A/D conversion of the various channels can be performed in
//! single, continuous, scan or discontinuous mode. The result of the ADC is stored
//! in a left-aligned or right-aligned 16-bit data register.
//!
//! ADC input range: VSSA ≤ VIN ≤ VDDA
//!
//
// https://github.com/stm32-rs/stm32f0xx-hal/blob/master/src/adc.rs

use embassy_hal_common::into_ref;
use crate::adc::{Adc, Instance};

const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
const VDD_CALIB: u16 = 3300;

/// 1 channel for internal temperature sensor (VSENSE)
/// Internal temperature sensor output voltage
pub struct Temperature;
impl<T: Instance> AdcPin<T> for Temperature {}
impl<T: Instance> super::sealed::AdcPin<T> for Temperature {
    fn channel(&self) -> u8 {
        // The temperature sensor is connected to channel ADC VIN[16].
        16
    }
}

/// 1 channel for internal reference voltage (VREFINT)
/// Internal voltage reference output voltage§
pub struct Vref;
impl<T: Instance> AdcPin<T> for Vref {}
impl<T: Instance> super::sealed::AdcPin<T> for Vref {
    fn channel(&self) -> u8 {
        // The internal voltage reference VREFINT is connected to channel ADC VIN[17].
        17
    }
}

/// 1 channel for monitoring external VBAT power supply pin
pub struct Vbat;
impl<T: Instance> AdcPin<T> for Vbat {}
impl<T: Instance> super::sealed::AdcPin<T> for Vbat {
    fn channel(&self) -> u8 {
        // VBAT channel is connected to ADC VIN[18] channel.
        18
    }
  
  impl<'d, T> Adc<'d, T>
where
    T: Instance,
{
   pub fn new(adc: impl Peripheral<P = T> + 'd, delay: &mut impl DelayUs<u32>) -> Self {
        into_ref!(adc);

        // A.7.4 ADC clock selection
        critical_section::with(|_| unsafe {
            // (1) Enable the peripheral clock of the ADC
            crate::pac::RCC.apb2enr().modify(|reg| reg.set_adcen(true));
            // (2) Start HSI14 RC oscillator
            crate::pac::RCC.cr2().modify(|reg| reg.set_hsi14on(true));
            // (3) Wait HSI14 is ready
            while !crate::pac::RCC.cr2().read().hsi14rdy() {}
            // (4) Select HSI14 by writing 00 in CKMODE (reset value)
        });

        // Delay 1μs when using HSI14 as the ADC clock.
        //
        // Table 57. ADC characteristics
        // tstab = 14 * 1/fadc
        //delay.delay_us(1);

        let freq = unsafe { crate::rcc::get_freqs().sys.0 };

        let mut s = Self {
            adc: adc,
            sample_time: SampleTime::Cycles239_5,
            vref_mv: VDD_CALIB.into(),
            resolution: Resolution::default(),
            align: Align::default(),
            //phantom: PhantomData,
        };

        Self::calibrate();
        // ADEN bit cannot be set when ADCAL = 1 and during four ADC clock cycles after the
        // ADCAL bit is cleared by hardware (end of calibration).
        //delay.delay_us(4);
        //crate::rcc::get_freqs().sys.0
        delay.delay_us((1_000_000 * 4) / freq + 1);

        //info!("EXIT calibrate");

        s.enable();
        //         // 11.4: Before starting a calibration, the ADC must have been in power-on state (ADON bit = ‘1’)
        //         // for at least two ADC clock cycles
        //         //delay.delay_us((1_000_000 * 2) / Self::freq().0 + 1);
        delay.delay_us((1_000_000 * 2) / freq + 1);

        //info!("ADC enabled: {:?}", s.is_enabled());

        s
    }
  
  }
  
  
}

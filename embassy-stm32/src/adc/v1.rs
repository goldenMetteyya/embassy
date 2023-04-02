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
use crate::adc::{Adc, Instance, Config};

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
   pub fn new(adc: impl Peripheral<P = T> + 'd, delay: &mut impl DelayUs<u32>, config: Config) -> Self {
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
            vref_mv: VDD_CALIB.into(),
            config: config
        };

        Self::calibrate();
        // ADEN bit cannot be set when ADCAL = 1 and during four ADC clock cycles after the
        // ADCAL bit is cleared by hardware (end of calibration).
        delay.delay_us((1_000_000 * 4) / freq + 1);

        s.enable();
        // 11.4: Before starting a calibration, the ADC must have been in power-on state (ADON bit = ‘1’)
        // for at least two ADC clock cycles
        delay.delay_us((1_000_000 * 2) / freq + 1);

        //info!("ADC enabled: {:?}", s.is_enabled());
        s
    }
    
      /// Procedure to enable the ADC
    fn enable(&mut self) {
        // A.7.2 ADC enable sequence code example
        //critical_section::with(|_| unsafe {
        unsafe {
            // (1) Ensure that ADRDY = 0
            if T::regs().isr().read().adrdy() {
                // (2) Clear ADRDY
                T::regs().isr().modify(|reg| reg.set_adrdy(false));
            }

            // (3) Enable the ADC, Set ADEN = 1 in the ADC_CR register.
            T::regs().cr().modify(|reg| reg.set_aden(true));

            // (4) Wait until ADC ready
            while !T::regs().isr().read().adrdy() {
                // ES0233, 2.4.3 ADEN bit cannot be set immediately after the ADC calibration
                // Workaround: When the ADC calibration is complete (ADCAL = 0), keep setting the
                // ADEN bit until the ADRDY flag goes high.
                // T::regs().cr().modify(|reg| reg.set_aden(true));
            }
        }
        //});
    }

    /// Procedure to disable the ADC
    fn disable(&mut self) {
        // A.7.3 ADC disable sequence code example
        critical_section::with(|_| unsafe {
            // (1) Stop any ongoing conversion
            T::regs().cr().modify(|reg| reg.set_adstp(true));

            // (2) Wait until ADSTP is reset by hardware i.e conversion is stopped
            while T::regs().cr().read().adstp() {}

            // (3) Disable the ADC
            // Set ADDIS = 1 in the ADC_CR register.
            T::regs().cr().modify(|reg| reg.set_addis(true));

            // (4) Wait until the ADC is fully disabled
            // If required by the application, wait until ADEN = 0 in the ADC_CR register,
            // indicating that the ADC is fully disabled (ADDIS is automatically reset once ADEN = 0).
            while T::regs().cr().read().aden() {}

            // (5) Clear the ADRDY bit in ADC_ISR register by programming this bit to 1 (optional).
            //T::regs().isr().modify(|_, w| w.set_adrdy(true));
        });
    }
    
     /// Check if the ADC is enabled.
    pub fn is_enabled(&self) -> bool {
        unsafe { T::regs().cr().read().aden() }
    }
    
     /// ADC Calibration procedure
    /// During the calibration phase, the ADC calculates
    /// a calibration factor which is internally applied
    /// to the ADC until the next ADC power-off.
    fn calibrate() {
        // A.7.1 ADC calibration code example
        unsafe {
            // (1) Ensure that ADEN = 0
            if T::regs().cr().read().aden() {
                T::regs().cr().modify(|reg| reg.set_addis(true));
            }

            // (2) Clear ADEN by setting ADDIS
            while T::regs().cr().read().aden() {
                // spin
                // For robust implementation, add here time-out management
            }

            // (3) Clear DMAEN
            T::regs().cfgr1().modify(|reg| reg.set_dmaen(false));

            // (4) Launch the calibration by setting ADCAL
            T::regs().cr().modify(|reg| reg.set_adcal(true));

            // (5) Wait until ADCAL=0
            while T::regs().cr().read().adcal() {
                // spin
                // For robust implementation, add here time-out management
            }
        }
    }
  
  }
    
  impl<'d, T: Instance> Drop for Adc<'d, T> {
    fn drop(&mut self) {
        self.disable();
    }
}
  
}

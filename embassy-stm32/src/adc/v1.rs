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

use core::marker::PhantomData;
use core::ptr;
use core::sync::atomic::{compiler_fence, Ordering};

use embassy_hal_common::into_ref;
use embedded_hal_02::blocking::delay::DelayUs;

use crate::adc::{Adc, AdcPin, Align, Instance, Resolution, SampleTime};
use crate::{pac, Peripheral};

const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
const VDD_CALIB: u16 = 3300;

const VREF_CHANNEL: u8 = 17;

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
}

// pub struct Adc<'d, T: Instance> {
//     sample_time: SampleTime,
//     vref_mv: u32,
//     resolution: Resolution,
//     //phantom: PhantomData<&'d mut T>,
// }

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

    // fn freq() -> Hertz {
    //     unsafe { crate::rcc::get_freqs() }.adc
    // }

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

    /// Enable the internal VBat sense, remember to disable when not in use
    /// as otherwise it will sap current from the VBat source.
    pub fn enable_vbat(&self, delay: &mut impl DelayUs<u32>) {
        // SMP must be ≥ 56 ADC clock cycles when using HSI14.
        //
        // 6.3.20 Vbat monitoring characteristics
        // ts_vbat ≥ 4μs
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_vbaten(true));
        }
        delay.delay_us(4);
    }

    /// Disable the internal VBat sense.
    pub fn disable_vbat(&mut self) {
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_vbaten(false));
        }
    }

    /// Returns if the internal VBat sense is enabled
    pub fn is_vbat_enabled(&self) -> bool {
        unsafe { T::regs().ccr().read().vbaten() }
    }

    /// Reads the value of VBat
    pub fn read_vbat(&mut self, delay: &mut impl DelayUs<u32>) -> u16 {
        // Table 28. Embedded internal reference voltage
        // tstart = 10μs
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_vbaten(true));
        }
        delay.delay_us(10);

        let mut vref;
        unsafe {
            //pin.set_as_analog();
            vref = self.read_channel(18);
        }

        //disable
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_vbaten(false));
        }

        vref
    }

    /// Reads the value of VBat in milli-volts
    pub fn read_vbat_mv(&mut self, delay: &mut impl DelayUs<u32>) -> u16 {
        let mut vbat = Vbat;

        let vbat_val: u16 = if self.is_vbat_enabled() {
            self.read_abs_mv(&mut vbat, delay)
        } else {
            self.enable_vbat(delay);

            let ret = self.read_abs_mv(&mut vbat, delay);

            self.disable_vbat();
            ret
        };

        vbat_val * 2
    }

    pub fn enable_vref(&mut self, delay: &mut impl DelayUs<u32>) {
        // Table 28. Embedded internal reference voltage
        // tstart = 10μs
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_vrefen(true));
        }
        delay.delay_us(10);
    }

    pub fn disable_vref(&mut self) {
        //disable
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_vrefen(false));
        }
    }

    pub fn is_vref_enabled(&self) -> bool {
        unsafe { T::regs().ccr().read().vrefen() }
    }

    pub fn read_vref(&mut self) -> u16 {
        let mut vref;
        unsafe {
            vref = self.read_channel(VREF_CHANNEL);
        }
        vref
    }

    /// Reads the value of VDDA in milli-volts
    pub fn read_vdda(&mut self, delay: &mut impl DelayUs<u32>) -> u16 {
        let vrefint_cal = u32::from(unsafe { ptr::read(VREFCAL) });

        //info!("vrefint_cal {}", vrefint_cal);

        let vref_val: u32 = if self.is_vref_enabled() {
            self.read_vref().into()
        } else {
            self.enable_vref(delay);
            let vref = self.read_vref();
            self.disable_vref();

            vref.into()
        };

        //info!("vref_val {}", vref_val);

        // The following formula gives the actual VDDA voltage supplying the device:
        //  VDDA = VDDA_Charac x VREFINT_CAL / VREFINT_DATA
        //
        // VDDA_Charac is the value of VDDA voltage characterized at VREFINT during the manufacturing process. It is specified in the device datasheet.
        // VREFINT_CAL is the VREFINT calibration value
        // VREFINT_DATA is the actual VREFINT output value converted by ADC
        (u32::from(VDD_CALIB) * vrefint_cal / vref_val) as u16
    }

    pub fn enable_vtemp(&self, delay: &mut impl DelayUs<u32>) {
        // SMP must be ≥ 56 ADC clock cycles when using HSI14.
        //
        // 6.3.19 Temperature sensor characteristics
        // tstart ≤ 10μs
        // ts_temp ≥ 4μs
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_tsen(true));
        }
        delay.delay_us(10);
    }

    pub fn disable_vtemp(&self) {
        unsafe {
            T::regs().ccr().modify(|reg| reg.set_tsen(false));
        }
    }

    pub fn is_vtemp_enabled(&self) -> bool {
        unsafe { T::regs().ccr().read().tsen() }
    }

    fn convert_temp(vtemp: u16, vdda: u16) -> i16 {
        let vtemp30_cal = unsafe { ptr::read(VTEMPCAL30) } as i32;
        let vtemp110_cal = unsafe { ptr::read(VTEMPCAL110) } as i32;
        let raw_temp_comp = vtemp as u32 * vdda as u32 / VDD_CALIB as u32;
        ((raw_temp_comp as i32 - vtemp30_cal) * 10 * (110 - 30) / (vtemp110_cal - vtemp30_cal) + 300) as i16
    }

    /// Read the value of the internal temperature sensor and return the
    /// result in 10ths of a degree centigrade.
    ///
    /// Given a delay reference it will attempt to restrict to the
    /// minimum delay needed to ensure a 10 us t<sub>START</sub> value.
    /// Otherwise it will approximate the required delay using ADC reads.
    pub fn read_vtemp(&mut self, delay: &mut impl DelayUs<u32>) -> i16 {
        let mut vtemp = Temperature;
        let vtemp_preenable = self.is_vtemp_enabled();

        if !vtemp_preenable {
            self.enable_vtemp(delay);
            delay.delay_us(2);
        }

        let vdda = self.read_vdda(delay);
        let vtemp_val = self.read(&mut vtemp);

        if !vtemp_preenable {
            self.disable_vtemp();
        }

        Self::convert_temp(vtemp_val, vdda)
    }

    pub fn set_sample_time(&mut self, sample_time: SampleTime) {
        self.sample_time = sample_time;
    }

    pub fn set_vref_mv(&mut self, vref_mv: u32) {
        self.vref_mv = vref_mv;
    }

    pub fn set_resolution(&mut self, resolution: Resolution) {
        self.resolution = resolution;
    }

    pub fn to_millivolts(&self, sample: u16) -> u16 {
        ((u32::from(sample) * self.vref_mv) / self.resolution.to_max_count()) as u16
    }

    /// Returns the largest possible sample value for the current settings
    pub fn max_sample(&self) -> u16 {
        4095
    }

    /// Read the ADC value on given Pin
    pub fn read(&mut self, pin: &mut impl AdcPin<T>) -> u16 {
        let channel = pin.channel();
        unsafe {
            //pin.set_as_analog();
            self.read_channel(channel)
        }
    }

    /// Read the value of a channel and converts the result to milli-volts
    pub fn read_abs_mv(&mut self, pin: &mut impl AdcPin<T>, delay: &mut impl DelayUs<u32>) -> u16 {
        let vdda = u32::from(self.read_vdda(delay));
        let v: u32 = self.read(pin).into();
        let max_samp = u32::from(self.max_sample());

        (v * vdda / max_samp) as u16
    }

    /// Read a value on a given ADC channel, Single conversion
    unsafe fn read_channel(&mut self, channel: u8) -> u16 {
        // A.7.5 ADC Single conversion sequence code example - Software trigger
        T::regs().cfgr1().modify(|reg| reg.set_res(self.resolution.into()));
        T::regs().isr().modify(|reg| {
            reg.set_eoc(true); // End of conversion flag
            reg.set_eosmp(true); //End of sampling flag
                                 //reg.set_eocie(false)// End of Conversion Interrupt
        });

        // Select Channel
        T::regs().chselr().write(|reg| reg.set_chselx(channel as usize, true));

        // Set Sample Time
        let sample_time = self.sample_time.into();
        T::regs().smpr().modify(|reg| reg.set_smp(sample_time));

        // Performs the AD conversion

        // Start the ADC conversion
        T::regs().cr().modify(|reg| reg.set_adstart(true));

        // Wait end of conversion
        while !T::regs().isr().read().eoc() {
            // spin
        }

        // Read data from register
        let value = T::regs().dr().read().0 as u16;

        value
    }

    // /// Read all adc channels in one go, single mode
    // unsafe fn sweep_read(&mut self) {}

    // unsafe fn read_triggered(&mut self) {
    //     T::regs().cfgr1().modify(|reg| reg.set_res(self.resolution.into()));
    //     while {
    //     }
    // }

    //async fn read_channel() {}
}

impl<'d, T: Instance> Drop for Adc<'d, T> {
    fn drop(&mut self) {
        self.disable();
    }
}

//
// ADC DMA with Ringbuffer
//

use crate::dma::ringbuffer::{DmaCtrl, DmaRingBuffer, OverrunError};
use crate::dma::TransferOptions;

pub struct RingBufferedAdc<'d, T: BasicInstance, RxDma: super::RxDma<T>> {
    adc: UartRx<'d, T, RxDma>,
    ring_buf: DmaRingBuffer<'d>,
}

impl<'d, T: BasicInstance, RxDma: super::RxDma<T>> DmaCtrl for UartRx<'d, T, RxDma> {
    fn ndtr(&self) -> usize {
        self.rx_dma.remaining_transfers() as usize
    }

    fn tcif(&self) -> bool {
        self.rx_dma.get_tcif()
    }

    fn clear_tcif(&mut self) {
        self.rx_dma.clear_tcif()
    }
}

impl<'d, T: BasicInstance, RxDma: super::RxDma<T>> UartRx<'d, T, RxDma> {
    /// Turn the `ADC` into a buffered adc which can continously receive in the background
    /// without the possibility of loosing bytes. The `dma_buf` is a buffer registered to the
    /// DMA controller, and must be sufficiently large, such that it will not overflow.
    pub fn into_ring_buffered(self, dma_buf: &'d mut [u8]) -> RingBufferedAdc<'d, T, RxDma> {
        assert!(dma_buf.len() > 0 && dma_buf.len() <= 0xFFFF);

        RingBufferedAdc {
            adc: self,
            ring_buf: DmaRingBuffer::new(dma_buf),
        }
    }
}

impl<'d, T: BasicInstance, RxDma: super::RxDma<T>> RingBufferedAdc<'d, T, RxDma> {
    /// Start receiving in the background into the previously provided `dma_buf` buffer.
    pub fn start(&mut self) -> Result<(), Error> {
        // Clear the ring buffer so that it is ready to receive data
        self.ring_buf.clear();

        self.setup_adc();

        Ok(())
    }

    /// Start adc background receive
    fn setup_adc(&mut self) {
        let ch = &mut self.adc.rx_dma;
        let request = ch.request();

        unsafe {
            // Start dma read
            // The memory address cannot be changed once the transfer is started
            let mut options = TransferOptions::default();
            options.circ = true; // Enable circular buffer mode
            options.tcie = false; // Do not enable transfer completed interrupt
            ch.start_read(request, rdr(T::regs()), self.ring_buf.dma_buf, options);
        }

        compiler_fence(Ordering::SeqCst);

        let r = T::regs();

        // clear all interrupts and DMA Rx Request
        // SAFETY: only clears Rx related flags
        // TODO: CHANGE FOR ADC DETAILS
        unsafe {
            r.cr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // enable parity interrupt if not ParityNone
                w.set_peie(w.pce());
                // disable idle line interrupt
                w.set_idleie(false);
            });
            r.cr3().modify(|w| {
                // enable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(true);
                // enable DMA Rx Request
                w.set_dmar(true);
            });
        }
    }

    /// Stop uart background receive
    fn teardown_adc(&mut self) {
        let r = T::regs();
        // clear all interrupts and DMA Rx Request
        // SAFETY: only clears Rx related flags

        unsafe {
            r.cr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // disable parity interrupt
                w.set_peie(false);
                // disable idle line interrupt
                w.set_idleie(false);
            });
            r.cr3().modify(|w| {
                // disable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(false);
                // disable DMA Rx Request
                w.set_dmar(false);
            });
        }

        compiler_fence(Ordering::SeqCst);

        let ch = &mut self.adc.rx_dma;
        ch.request_stop();
        while ch.is_running() {}
    }
}

impl<T: BasicInstance, RxDma: super::RxDma<T>> Drop for RingBufferedAdc<'_, T, RxDma> {
    fn drop(&mut self) {
        self.teardown_adc();
    }
}

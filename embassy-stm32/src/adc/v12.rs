//!
//!

use embassy_cortex_m::interrupt::InterruptExt;
use embassy_hal_common::{into_ref, PeripheralRef};
use embedded_hal_02::blocking::delay::DelayUs;
use crate::adc::{SampleTime, Resolution, Align};
use crate::dma::NoDma;


const VREFCAL: *const u16 = 0x1FFF_F7BA as *const u16;
const VTEMPCAL30: *const u16 = 0x1FFF_F7B8 as *const u16;
const VTEMPCAL110: *const u16 = 0x1FFF_F7C2 as *const u16;
const VDD_CALIB: u16 = 3300;

 /// Possible external triggers the ADC can listen to
 #[cfg_attr(feature = "defmt", derive(defmt::Format))]
 #[derive(Copy, Clone, PartialEq, Eq, Debug)]
 pub enum ExternalTrigger {
     /// TIM1 compare channel 1
     Tim_1_cc_1,
     /// TIM1 compare channel 2
     Tim_1_cc_2,
     /// TIM1 compare channel 3
     Tim_1_cc_3,
     /// TIM2 compare channel 2
     Tim_2_cc_2,
     /// TIM2 compare channel 3
     Tim_2_cc_3,
     /// TIM2 compare channel 4
     Tim_2_cc_4,
     /// TIM2 trigger out
     Tim_2_trgo,
     /// TIM3 compare channel 1
     Tim_3_cc_1,
     /// TIM3 trigger out
     Tim_3_trgo,
     /// TIM4 compare channel 4
     Tim_4_cc_4,
     /// TIM5 compare channel 1
     Tim_5_cc_1,
     /// TIM5 compare channel 2
     Tim_5_cc_2,
     /// TIM5 compare channel 3
     Tim_5_cc_3,
     /// External interupt line 11
     Exti_11,
 }
 impl From<ExternalTrigger> for u8 {
     fn from(et: ExternalTrigger) -> u8 {
         match et {
             ExternalTrigger::Tim_1_cc_1 => 0b0000,
             ExternalTrigger::Tim_1_cc_2 => 0b0001,
             ExternalTrigger::Tim_1_cc_3 => 0b0010,
             ExternalTrigger::Tim_2_cc_2 => 0b0011,
             ExternalTrigger::Tim_2_cc_3 => 0b0100,
             ExternalTrigger::Tim_2_cc_4 => 0b0101,
             ExternalTrigger::Tim_2_trgo => 0b0110,
             ExternalTrigger::Tim_3_cc_1 => 0b0111,
             ExternalTrigger::Tim_3_trgo => 0b1000,
             ExternalTrigger::Tim_4_cc_4 => 0b1001,
             ExternalTrigger::Tim_5_cc_1 => 0b1010,
             ExternalTrigger::Tim_5_cc_2 => 0b1011,
             ExternalTrigger::Tim_5_cc_3 => 0b1100,
             ExternalTrigger::Exti_11 => 0b1111,
         }
     }
 }

 /// Possible trigger modes
 #[cfg_attr(feature = "defmt", derive(defmt::Format))]
 #[derive(Copy, Clone, PartialEq, Eq, Debug)]
 pub enum TriggerMode {
     /// Don't listen to external trigger
     Disabled,
     /// Listen for rising edges of external trigger
     RisingEdge,
     /// Listen for falling edges of external trigger
     FallingEdge,
     /// Listen for both rising and falling edges of external trigger
     BothEdges,
 }
 impl From<TriggerMode> for u8 {
     fn from(tm: TriggerMode) -> u8 {
         match tm {
             TriggerMode::Disabled => 0,
             TriggerMode::RisingEdge => 1,
             TriggerMode::FallingEdge => 2,
             TriggerMode::BothEdges => 3,
         }
     }
 }

#[non_exhaustive]
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub struct Config {
    sample_time: SampleTime,
    resolution: Resolution,
    align: Align,
}

impl Default for Config {
    fn default() -> Self {
        Self {
          sample_time: SampleTime::Cycles239_5,
          resolution: Resolution::default(), // 12bit
          align: Align::default() // Right
        }
    }
}

/// Serial error
#[derive(Debug, Eq, PartialEq, Copy, Clone)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
#[non_exhaustive]
pub enum Error {
    /// Framing error
    Framing,
    /// Noise error
    Noise,
    /// RX buffer overrun
    Overrun,
    /// Parity check error
    Parity,
    /// Buffer too large for DMA
    BufferTooLong,
}

pub struct Adc<'d, T: BasicInstance, RxDma = NoDma> {
    _peri: PeripheralRef<'d, T>,
    rx_dma: PeripheralRef<'d, RxDma>,
    // ring_buf: DmaRingBuffer<'d>,
}

impl<'d, T: BasicInstance, RxDma> UartRx<'d, T, RxDma> {
    pub fn new(
        peri: impl Peripheral<P = T> + 'd,
        irq: impl Peripheral<P = T::Interrupt> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        rx_dma: impl Peripheral<P = RxDma> + 'd,
        config: Config,
        delay: &mut impl DelayUs<u32>
    ) -> Self {
        T::enable();
        T::reset();

        Self::new_inner(peri, irq, rx, rx_dma, config, delay)
    }

    fn new_inner(
        peri: impl Peripheral<P = T> + 'd,
        irq: impl Peripheral<P = T::Interrupt> + 'd,
        rx: impl Peripheral<P = impl RxPin<T>> + 'd,
        rx_dma: impl Peripheral<P = RxDma> + 'd,
        config: Config,
        delay: &mut impl DelayUs<u32>
    ) -> Self {
        into_ref!(peri, irq, rx, rx_dma);

        let r = T::regs();

        // unsafe {
        //     rx.set_as_af(rx.af_num(), AFType::Input);
        // }

        // configure(r, &config, T::frequency(), T::MULTIPLIER, true, false);

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

        let freq = unsafe { crate::rcc::get_freqs().sys.0 };

        Self::calibrate;
        // ADEN bit cannot be set when ADCAL = 1 and during four ADC clock cycles after the
        // ADCAL bit is cleared by hardware (end of calibration).
        //delay.delay_us(4);
        //crate::rcc::get_freqs().sys.0
        delay.delay_us((1_000_000 * 4) / freq + 1);

        Self::enable();
        // 11.4: Before starting a calibration, the ADC must have been in power-on state (ADON bit = ‘1’)
        // for at least two ADC clock cycles
        //delay.delay_us((1_000_000 * 2) / Self::freq().0 + 1);
        delay.delay_us((1_000_000 * 2) / freq + 1);

        irq.set_handler(Self::on_interrupt);
        irq.unpend();
        irq.enable();

        // create state once!
        let _s = T::state();

        Self {
            _peri: peri,
            rx_dma
        }
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

     /// Procedure to enable the ADC
     fn enable() {
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
    fn disable() {
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


    fn on_interrupt(_: *mut ()) {
        let r = T::regs();
        let s = T::state();

        let (sr, cr1, cr3) = unsafe { (sr(r).read(), r.cr1().read(), r.cr3().read()) };

        let has_errors = (sr.pe() && cr1.peie()) || ((sr.fe() || sr.ne() || sr.ore()) && cr3.eie());

        if has_errors {
            // clear all interrupts and DMA Rx Request
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

            s.rx_waker.wake();
        } else if cr1.idleie() && sr.idle() {
            // IDLE detected: no more data will come
            unsafe {
                r.cr1().modify(|w| {
                    // disable idle line detection
                    w.set_idleie(false);
                });

                r.cr3().modify(|w| {
                    // disable DMA Rx Request
                    w.set_dmar(false);
                });
            }
            compiler_fence(Ordering::SeqCst);

            s.rx_waker.wake();
        }
    }

    pub async fn read(&mut self, buffer: &mut [u8]) -> Result<(), Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        self.inner_read(buffer, false).await?;

        Ok(())
    }

    pub fn blocking_read(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        unsafe {
            let r = T::regs();
            for b in buffer {
                loop {
                    let sr = sr(r).read();
                    if sr.pe() {
                        rdr(r).read_volatile();
                        return Err(Error::Parity);
                    } else if sr.fe() {
                        rdr(r).read_volatile();
                        return Err(Error::Framing);
                    } else if sr.ne() {
                        rdr(r).read_volatile();
                        return Err(Error::Noise);
                    } else if sr.ore() {
                        rdr(r).read_volatile();
                        return Err(Error::Overrun);
                    } else if sr.rxne() {
                        break;
                    }
                }
                *b = rdr(r).read_volatile();
            }
        }
        Ok(())
    }

    async fn inner_read_run(
        &mut self,
        buffer: &mut [u8],
        enable_idle_line_detection: bool,
    ) -> Result<ReadCompletionEvent, Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        let r = T::regs();

        // make sure USART state is restored to neutral state when this future is dropped
        let on_drop = OnDrop::new(move || {
            // defmt::trace!("Clear all USART interrupts and DMA Read Request");
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
        });

        let ch = &mut self.rx_dma;
        let request = ch.request();

        // Start USART DMA
        // will not do anything yet because DMAR is not yet set
        // future which will complete when DMA Read request completes
        let transfer = crate::dma::read(ch, request, rdr(T::regs()), buffer);

        // SAFETY: The only way we might have a problem is using split rx and tx
        // here we only modify or read Rx related flags, interrupts and DMA channel
        unsafe {
            // clear ORE flag just before enabling DMA Rx Request: can be mandatory for the second transfer
            if !self.detect_previous_overrun {
                let sr = sr(r).read();
                // This read also clears the error and idle interrupt flags on v1.
                rdr(r).read_volatile();
                clear_interrupt_flags(r, sr);
            }

            r.cr1().modify(|w| {
                // disable RXNE interrupt
                w.set_rxneie(false);
                // enable parity interrupt if not ParityNone
                w.set_peie(w.pce());
            });

            r.cr3().modify(|w| {
                // enable Error Interrupt: (Frame error, Noise error, Overrun error)
                w.set_eie(true);
                // enable DMA Rx Request
                w.set_dmar(true);
            });

            compiler_fence(Ordering::SeqCst);

            // In case of errors already pending when reception started, interrupts may have already been raised
            // and lead to reception abortion (Overrun error for instance). In such a case, all interrupts
            // have been disabled in interrupt handler and DMA Rx Request has been disabled.

            let cr3 = r.cr3().read();

            if !cr3.dmar() {
                // something went wrong
                // because the only way to get this flag cleared is to have an interrupt

                // DMA will be stopped when transfer is dropped

                let sr = sr(r).read();
                // This read also clears the error and idle interrupt flags on v1.
                rdr(r).read_volatile();
                clear_interrupt_flags(r, sr);

                if sr.pe() {
                    return Err(Error::Parity);
                }
                if sr.fe() {
                    return Err(Error::Framing);
                }
                if sr.ne() {
                    return Err(Error::Noise);
                }
                if sr.ore() {
                    return Err(Error::Overrun);
                }

                unreachable!();
            }

            if !enable_idle_line_detection {
                transfer.await;

                return Ok(ReadCompletionEvent::DmaCompleted);
            }

            // clear idle flag
            let sr = sr(r).read();
            // This read also clears the error and idle interrupt flags on v1.
            rdr(r).read_volatile();
            clear_interrupt_flags(r, sr);

            // enable idle interrupt
            r.cr1().modify(|w| {
                w.set_idleie(true);
            });
        }

        compiler_fence(Ordering::SeqCst);

        // future which completes when idle line is detected
        let idle = poll_fn(move |cx| {
            let s = T::state();

            s.rx_waker.register(cx.waker());

            // SAFETY: read only and we only use Rx related flags
            let sr = unsafe { sr(r).read() };

            // SAFETY: only clears Rx related flags
            unsafe {
                // This read also clears the error and idle interrupt flags on v1.
                rdr(r).read_volatile();
                clear_interrupt_flags(r, sr);
            }

            compiler_fence(Ordering::SeqCst);

            let has_errors = sr.pe() || sr.fe() || sr.ne() || sr.ore();

            if has_errors {
                // all Rx interrupts and Rx DMA Request have already been cleared in interrupt handler

                if sr.pe() {
                    return Poll::Ready(Err(Error::Parity));
                }
                if sr.fe() {
                    return Poll::Ready(Err(Error::Framing));
                }
                if sr.ne() {
                    return Poll::Ready(Err(Error::Noise));
                }
                if sr.ore() {
                    return Poll::Ready(Err(Error::Overrun));
                }
            }

            if sr.idle() {
                // Idle line detected
                return Poll::Ready(Ok(()));
            }

            Poll::Pending
        });

        // wait for the first of DMA request or idle line detected to completes
        // select consumes its arguments
        // when transfer is dropped, it will stop the DMA request
        let r = match select(transfer, idle).await {
            // DMA transfer completed first
            Either::First(()) => Ok(ReadCompletionEvent::DmaCompleted),

            // Idle line detected first
            Either::Second(Ok(())) => Ok(ReadCompletionEvent::Idle),

            // error occurred
            Either::Second(Err(e)) => Err(e),
        };

        drop(on_drop);

        r
    }

    async fn inner_read(&mut self, buffer: &mut [u8], enable_idle_line_detection: bool) -> Result<usize, Error>
    where
        RxDma: crate::usart::RxDma<T>,
    {
        if buffer.is_empty() {
            return Ok(0);
        } else if buffer.len() > 0xFFFF {
            return Err(Error::BufferTooLong);
        }

        let buffer_len = buffer.len();

        // wait for DMA to complete or IDLE line detection if requested
        let res = self.inner_read_run(buffer, enable_idle_line_detection).await;

        let ch = &mut self.rx_dma;

        match res {
            Ok(ReadCompletionEvent::DmaCompleted) => Ok(buffer_len),
            Ok(ReadCompletionEvent::Idle) => {
                let n = buffer_len - (ch.remaining_transfers() as usize);
                Ok(n)
            }
            Err(e) => Err(e),
        }
    }
}


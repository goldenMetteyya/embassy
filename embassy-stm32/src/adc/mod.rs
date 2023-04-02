#![macro_use]

#[cfg_attr(adc_f1, path = "f1.rs")]
#[cfg_attr(adc_v1, path = "v1.rs")]
#[cfg_attr(adc_v2, path = "v2.rs")]
#[cfg_attr(any(adc_v3, adc_g0), path = "v3.rs")]
#[cfg_attr(adc_v4, path = "v4.rs")]
mod _version;

// #[cfg(not(any(adc_f1, adc_v1)))]
#[cfg(not(any(adc_f1)))]
mod resolution;
// #[cfg(not(adc_v1))]
mod sample_time;

#[allow(unused)]
pub use _version::*;
// #[cfg(not(any(adc_f1, adc_v1)))]
#[cfg(not(any(adc_f1)))]
pub use resolution::Resolution;
// #[cfg(not(adc_v1))]
pub use sample_time::SampleTime;

use crate::peripherals;

#[cfg(not(adc_v1))]
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: crate::PeripheralRef<'d, T>,
    sample_time: SampleTime,
}

#[cfg(adc_v1)]
pub struct Adc<'d, T: Instance> {
    #[allow(unused)]
    adc: crate::PeripheralRef<'d, T>,
    sample_time: SampleTime,
    resolution: Resolution,
    align: Align,
    //
    vref_mv: u32,
    //phantom: PhantomData<&'d mut T>,
}

pub(crate) mod sealed {
    use embassy_sync::waitqueue::AtomicWaker;

    use super::*;

    pub struct State {
        pub rx_waker: AtomicWaker,
    }

    impl State {
        pub const fn new() -> Self {
            Self {
                rx_waker: AtomicWaker::new(),
            }
        }
    }

    pub trait Instance {
        type Interrupt: crate::interrupt::Interrupt;

        fn regs() -> crate::pac::adc::Adc;
        #[cfg(all(not(adc_f1), not(adc_v1)))]
        fn common_regs() -> crate::pac::adccommon::AdcCommon;

        fn state() -> &'static State;
    }

    pub trait AdcPin<T: Instance> {
        fn channel(&self) -> u8;
    }

    pub trait InternalChannel<T> {
        fn channel(&self) -> u8;
    }
}

#[cfg(not(any(adc_f1, adc_v2, adc_v4)))]
pub trait Instance: sealed::Instance + crate::Peripheral<P = Self> {}
#[cfg(any(adc_f1, adc_v2, adc_v4))]
pub trait Instance: sealed::Instance + crate::Peripheral<P = Self> + crate::rcc::RccPeripheral {}

pub trait AdcPin<T: Instance>: sealed::AdcPin<T> {}
pub trait InternalChannel<T>: sealed::InternalChannel<T> {}

#[cfg(not(stm32h7))]
foreach_peripheral!(
    (adc, $inst:ident) => {
        impl crate::adc::sealed::Instance for peripherals::$inst {
            fn regs() -> crate::pac::adc::Adc {
                crate::pac::$inst
            }
            #[cfg(all(not(adc_f1), not(adc_v1)))]
            fn common_regs() -> crate::pac::adccommon::AdcCommon {
                foreach_peripheral!{
                    (adccommon, $common_inst:ident) => {
                        return crate::pac::$common_inst
                    };
                }
            }
        }

        impl crate::adc::Instance for peripherals::$inst {}
    };
);

#[cfg(stm32h7)]
foreach_peripheral!(
    (adc, ADC3) => {
        impl crate::adc::sealed::Instance for peripherals::ADC3 {
            fn regs() -> crate::pac::adc::Adc {
                crate::pac::ADC3
            }
            #[cfg(all(not(adc_f1), not(adc_v1)))]
            fn common_regs() -> crate::pac::adccommon::AdcCommon {
                foreach_peripheral!{
                    (adccommon, ADC3_COMMON) => {
                        return crate::pac::ADC3_COMMON
                    };
                }
            }
        }

        impl crate::adc::Instance for peripherals::ADC3 {}
    };
    (adc, $inst:ident) => {
        impl crate::adc::sealed::Instance for peripherals::$inst {
            fn regs() -> crate::pac::adc::Adc {
                crate::pac::$inst
            }
            #[cfg(all(not(adc_f1), not(adc_v1)))]
            fn common_regs() -> crate::pac::adccommon::AdcCommon {
                foreach_peripheral!{
                    (adccommon, ADC_COMMON) => {
                        return crate::pac::ADC_COMMON
                    };
                }
            }
        }

        impl crate::adc::Instance for peripherals::$inst {}
    };
);

macro_rules! impl_adc_pin {
    ($inst:ident, $pin:ident, $ch:expr) => {
        impl crate::adc::AdcPin<peripherals::$inst> for crate::peripherals::$pin {}

        impl crate::adc::sealed::AdcPin<peripherals::$inst> for crate::peripherals::$pin {
            fn channel(&self) -> u8 {
                $ch
            }
        }
    };
}

/// ADC Result Alignment
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub enum Align {
    /// Right aligned results (least significant bits)
    ///
    /// Results in all precisions returning values from 0-(2^bits-1) in
    /// steps of 1.
    Right,
    /// Left aligned results (most significant bits)
    ///
    /// Results in all precisions returning a value in the range 0-65535.
    /// Depending on the precision the result will step by larger or smaller
    /// amounts.
    Left,
}
impl From<Align> for bool {
    fn from(a: Align) -> bool {
        match a {
            Align::Right => false,
            Align::Left => true,
        }
    }
}

impl Default for Align {
    fn default() -> Self {
        Align::Right
    }
}

dma_trait!(RxDma, Instance);

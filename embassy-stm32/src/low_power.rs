//! Low-power support.
//!
//! The STM32 line of microcontrollers support various deep-sleep modes which exploit clock-gating
//! to reduce power consumption. The `embassy-stm32` HAL provides a `sleep()` function which
//! can use knowledge of which peripherals are currently blocked upon to transparently and safely
//! enter such low-power modes including `STOP1` and `STOP2` when possible.
//!
//! `sleep()` determines which peripherals are active by their RCC state; consequently,
//! low-power states can only be entered if peripherals which block stop have been `drop`'d and if
//! peripherals that do not block stop are busy. Peripherals which never block stop include:
//!
//!  * `GPIO`
//!  * `RTC`
//!
//! Other peripherals which block stop when busy include (this list may be stale):
//!
//!  * `I2C`
//!  * `USART`
//!
//! Since entering and leaving low-power modes typically incurs a significant latency, `sleep()`
//! will only attempt to enter when the next timer event is at least [`config.min_stop_pause`] in the future.
//!
//! `embassy-stm32` also provides an `embassy-executor` platform implementation that integrates `sleep()` into the main loop. It is available
//! in the `embassy_stm32::executor` module, and is enabled by the `executor-thread` or `executor-interrupt` features. This stm32-specific
//! executor is the preferred way to lower power consumption if you're using `async`, instead of calling `sleep()` directly.

use core::mem;
use core::sync::atomic::{AtomicBool, Ordering, compiler_fence};
use cortex_m::interrupt::Mutex;
use cortex_m::peripheral::SCB;
use critical_section::{CriticalSection, with};

use crate::dma::info;
#[cfg(all(feature = "rt", not(feature = "_lp-time-driver")))]
use crate::interrupt;
pub use crate::rcc::StopMode;
use crate::rcc::get_stop_mode;
use crate::time_driver::{LPTimeDriver, get_driver};

#[cfg(all(feature = "rt", not(any(stm32u0, feature = "_lp-time-driver"))))]
foreach_interrupt! {
    (RTC, rtc, $block:ident, WKUP, $irq:ident) => {
        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $irq() {
        }
    };
}

#[cfg(all(feature = "rt", stm32u0))]
foreach_interrupt! {
    (RTC, rtc, $block:ident, TAMP, $irq:ident) => {
        #[interrupt]
        #[allow(non_snake_case)]
        unsafe fn $irq() {
        }
    };
}

#[cfg(any(stm32l4, stm32l5, stm32u5, stm32u3, stm32wba, stm32wb, stm32wl, stm32u0))]
use crate::pac::pwr::vals::Lpms;

#[cfg(any(stm32l4, stm32l5, stm32u5, stm32u3, stm32wba, stm32wb, stm32wl, stm32u0))]
impl Into<Lpms> for StopMode {
    fn into(self) -> Lpms {
        match self {
            StopMode::Stop1 => Lpms::STOP1,
            #[cfg(not(stm32wba))]
            StopMode::Standby | StopMode::Stop2 => Lpms::STOP2,
            #[cfg(stm32wba)]
            // WBA STOP2 is auto-entered by hardware when LPMS=STOP0 and
            // the 2.4 GHz radio is in deep sleep. It's not a separate LPMS value.
            StopMode::Standby | StopMode::Stop2 => Lpms::STOP0,
        }
    }
}

mod platform {
    use critical_section::CriticalSection;

    use crate::rcc::StopMode;

    /// Enter stop mode
    pub fn enter_stop(_cs: CriticalSection, stop_mode: StopMode) -> Result<(), ()> {
        #[cfg(stm32wb)]
        fn enter_stop_stm32wb(
            _cs: CriticalSection<'_>,
        ) -> Result<crate::hsem::HardwareSemaphoreMutex<'_, crate::peripherals::HSEM>, ()> {
            use core::task::Poll;

            use embassy_futures::poll_once;

            use crate::hsem::get_hsem;
            use crate::pac::rcc::vals::{Smps, Sw};
            use crate::pac::{PWR, RCC};

            trace!("low power: trying to get sem3");

            let sem3_mutex = match poll_once(get_hsem(3).lock(0)) {
                Poll::Pending => None,
                Poll::Ready(mutex) => Some(mutex),
            }
            .ok_or(())?;

            trace!("low power: got sem3");

            let sem4_mutex = get_hsem(4).try_lock(0);
            if let Some(sem4_mutex) = sem4_mutex {
                trace!("low power: got sem4");

                if PWR.extscr().read().c2ds() {
                    drop(sem4_mutex);
                } else {
                    return Ok(sem3_mutex);
                }
            }

            // Sem4 not granted
            // Set HSION
            RCC.cr().modify(|w| {
                w.set_hsion(true);
            });

            // Wait for HSIRDY
            while !RCC.cr().read().hsirdy() {}

            // Set SW to HSI
            RCC.cfgr().modify(|w| {
                w.set_sw(Sw::HSI);
            });

            // Wait for SWS to report HSI
            while !RCC.cfgr().read().sws().eq(&Sw::HSI) {}

            // Set SMPSSEL to HSI
            RCC.smpscr().modify(|w| {
                w.set_smpssel(Smps::HSI);
            });

            Ok(sem3_mutex)
        }

        #[cfg(stm32wb)]
        let mutex = {
            use crate::pac::{PWR, RCC};

            let mutex = enter_stop_stm32wb(_cs)?;

            // on PWR
            RCC.apb1enr1().modify(|r| r.0 |= 1 << 28);
            cortex_m::asm::dsb();

            // off SMPS, on Bypass
            PWR.cr5().modify(|r| {
                let mut val = r.0;
                val &= !(1 << 15); // sdeb = 0 (off SMPS)
                val |= 1 << 14; // sdben = 1 (on Bypass)
                r.0 = val
            });

            cortex_m::asm::delay(1000);

            mutex
        };

        #[cfg(any(stm32l4, stm32l5, stm32u5, stm32u3, stm32u0, stm32wb, stm32wba, stm32wl))]
        {
            #[cfg(not(feature = "_core-cm0p"))]
            crate::pac::PWR.cr1().modify(|m| m.set_lpms(stop_mode.into()));
            #[cfg(feature = "_core-cm0p")]
            crate::pac::PWR.c2cr1().modify(|m| m.set_lpms(stop_mode.into()));
        }
        #[cfg(stm32h5)]
        crate::pac::PWR.pmcr().modify(|v| {
            use crate::pac::pwr::vals;
            v.set_lpms(vals::Lpms::STOP);
            v.set_svos(vals::Svos::SCALE3);
        });

        #[cfg(stm32l0)]
        {

            use crate::pac::pwr::vals::Pdds;
            crate::pac::PWR.cr().modify(|w| {
                w.set_pdds(Pdds::STOP_MODE);// STANDBY_MODE
                w.set_cwuf(true);
                w.set_ulp(true); // ref (page 151, RM0376)
                w.set_lpsdsr(stm32_metapac::pwr::vals::Mode::LOW_POWER_MODE); // ref (page 168, RM0376)
                w.set_fwu(true); // fast wakeup https://forum.digikey.com/t/low-power-modes-on-the-stm32l0-series/13306
            });

            // set the MSI16 as wake-up from stop clock
            crate::pac::RCC.cfgr().modify(|w| 
            {
                w.set_stopwuck(stm32_metapac::rcc::vals::Stopwuck::MSI);
            });

            // crate::pac::ADC1.cr().modify(|w| {
            //     w.set_addis(true);
            // });
            // crate::pac::DAC1.cr().modify(|w| {
            //     w.set_en(1, false);
            // });
        }

        #[cfg(stm32wb)]
        drop(mutex);

        let _ = stop_mode;

        Ok(())
    }

    /// Clear any previous stop flags
    pub fn clear_flags() {
        #[cfg(stm32wl)]
        crate::pac::PWR.extscr().modify(|w| {
            #[cfg(not(feature = "_core-cm0p"))]
            w.set_c1cssf(true);
            #[cfg(feature = "_core-cm0p")]
            w.set_c2cssf(true);
        });
        #[cfg(stm32wba)]
        crate::pac::PWR.sr().modify(|w| w.set_cssf(true));

        #[cfg(stm32l0)]
        crate::pac::PWR.cr().modify(|w| w.set_cwuf(true));
    }

    /// Exit stop mode, reinitializing timer and rcc if required
    pub fn exit_stop(_cs: CriticalSection) {
        #[cfg(any(stm32l0, stm32wl, stm32wb, stm32wba))]
        {
            // stm32wl5x is dual core and we don't want BOTH cores to re-initialize RCC so we hold a lock
            #[cfg(stm32wl5x)]
            let lock = crate::hsem::get_hsem(3).blocking_lock(0);

            #[cfg(any(stm32wl, stm32wb))]
            let es = crate::pac::PWR.extscr().read();

            #[cfg(stm32wba)]
            let es = crate::pac::PWR.sr().read();

            #[cfg(stm32l0)]
            let es = crate::pac::PWR.csr().read();

            // we need to re-initialize RCC if *BOTH* cores have been in some STOP mode!
            #[cfg(any(stm32l0, stm32wl, stm32wba))]
            let re_initialize_rcc = {
                #[cfg(stm32wl5x)]
                {
                    // core 1 in any STOP mode AND core 2 in any STOP mode
                    (es.c1stopf() || es.c1stop2f()) && (es.c2stopf() || es.c2stop2f())
                }
                #[cfg(stm32wlex)]
                {
                    es.c1stop2f() || es.c1stopf()
                }
                #[cfg(stm32wba)]
                {
                    es.stopf()
                }
                #[cfg(stm32l0)]
                {
                    es.wuf()
                }
            };

            #[cfg(any(stm32wl, stm32wba))]
            let re_initialize_timer = {
                #[cfg(all(stm32wl, not(feature = "_core-cm0p")))]
                {
                    es.c1stop2f()
                }
                #[cfg(all(stm32wl, feature = "_core-cm0p"))]
                {
                    es.c2stop2f()
                }
                #[cfg(stm32wba)]
                {
                    es.stopf()
                }
            };

            #[cfg(any(stm32l0, stm32wl, stm32wba))]
            if re_initialize_rcc {
                // when we wake from any stop mode we need to re-initialize the rcc
                crate::rcc::reinit_saved(_cs);
            }

            #[cfg(stm32wba)]
            match (es.stopf(), es.stop2f()) {
                (true, true) => debug!("low power: WBA woke from STOP2"),
                (true, false) => debug!("low power: WBA woke from STOP0/1"),
                _ => {}
            };

            #[cfg(stm32wl)]
            match (es.c1stopf(), es.c1stop2f()) {
                (true, false) => debug!("low power: cpu1 has been in STOP1"),
                (false, true) => debug!("low power: cpu1 has been in STOP2"),
                (true, true) => debug!("low power: cpu1 has been in STOP1 and STOP2 ???"),
                (false, false) => trace!("low power: cpu1 stop mode not entered"),
            };

            #[cfg(stm32wl5x)]
            // TODO: only for the current cpu
            match (es.c2stopf(), es.c2stop2f()) {
                (true, false) => debug!("low power: cpu2 has been in STOP1"),
                (false, true) => debug!("low power: cpu2 has been in STOP2"),
                (true, true) => debug!("low power: cpu2 has been in STOP1 and STOP2 ???"),
                (false, false) => trace!("low power: cpu2 stop mode not entered"),
            };

            #[cfg(stm32wb)]
            match (es.c1stopf(), es.c2stopf()) {
                (true, false) => debug!("low power: cpu1 has been in STOP"),
                (false, true) => debug!("low power: cpu2 has been in STOP"),
                (true, true) => debug!("low power: cpu1 and cpu2 have been in STOP"),
                (false, false) => trace!("low power: stop mode not entered"),
            };

            #[cfg(stm32l0)]
            match es.wuf() {
                true => debug!("low power: L0 has been in stop"),
                _ => {}
            };

            clear_flags();

            #[cfg(stm32wl5x)]
            drop(lock);

            #[cfg(any(stm32wl, stm32wba))]
            if re_initialize_timer {
                trace!("low power: re-initializing timer");
                // when we wake from STOP2, we need to re-initialize the time driver
                #[cfg(not(feature = "_lp-time-driver"))]
                super::get_driver().init_timer(_cs);
            }
        }
    }
}

static STOP_ENTERED: AtomicBool = AtomicBool::new(false);

unsafe fn on_wakeup(cs: CriticalSection) {
    if STOP_ENTERED.load(Ordering::Acquire) {
        platform::exit_stop(cs);

        get_driver().resume_time(cs);
        trace!("low power: resumed");
    }

    STOP_ENTERED.store(false, Ordering::Release);
}

fn configure_pwr(cs: CriticalSection) {
    const fn get_scb() -> SCB {
        unsafe { mem::transmute(()) }
    }

    // get_scb().clear_sleepdeep();
    unsafe 
    {
        cortex_m::Peripherals::steal().SCB.clear_sleepdeep();
    }
    platform::clear_flags();

    compiler_fence(Ordering::Acquire);

    let Some(stop_mode) = get_stop_mode(cs) else {
        return;
    };

    if get_driver().pause_time(cs).is_err() {
        warn!("low_power: failed to pause time, not entering stop");
    } else if platform::enter_stop(cs, stop_mode).is_err() {
        warn!("low_power: failed to enter stop");
    } else {
        #[cfg(stm32l0)]
        trace!("low power: enter stop");
        #[cfg(not(stm32l0))]
        trace!("low power: enter stop: {}", stop_mode);

        STOP_ENTERED.store(true, Ordering::Release);

        #[cfg(not(feature = "low-power-debug-with-sleep"))]
        // get_scb().set_sleepdeep();
        unsafe 
        {
            cortex_m::Peripherals::steal().SCB.set_sleepdeep();
        }
    }
}

/// Sleep with WFI, attempting to enter the deepest STOP mode possible.
///
/// If it's not possible to enter any STOP mode due to running peripherals it will
/// still do a `WFI` sleep. Therefore this function is equivalent to `WFI` except
/// with lower power consumption and higher latency.
///
/// ## SAFETY
///
/// Care must be taken that we have ensured that the system is ready to go to deep
/// sleep, otherwise HAL peripherals may misbehave. HAL drivers automatically prevent
/// sleep as needed, but you might have to do it manually if you're using some peripherals
/// with the PAC directly.
pub unsafe fn sleep(cs: CriticalSection) {
    configure_pwr(cs);

    let scb_scr;
    unsafe 
    {
        scb_scr = cortex_m::Peripherals::steal().SCB.scr.read();
    }

    let pwr_cr = crate::pac::PWR.cr().read();
    let exti_pr = crate::pac::EXTI.pr(0).read();
    
    info!("---- START REGISTER DUMP ---- \n");

    info!("SCB_SCR: {:x}", scb_scr);
    info!("PWR_CR: {:x}", pwr_cr);
    info!("EXTI_PR: {:x}", exti_pr);

    info!("---- END REGISTER DUMP ---- \n");

    #[cfg(feature = "low-power-defmt-flush")]
    defmt::flush();

    store_gpio_context();
    put_all_gpios_into_analog();

    cortex_m::asm::dsb();
    cortex_m::asm::wfi();

    recover_gpio_context();

    cortex_m::asm::isb();
    cortex_m::asm::dsb();

    on_wakeup(cs);
}


use stm32_metapac::gpio::Gpio;
use stm32_metapac::gpio::regs::{Afr, Lckr, Moder, Odr, Ospeedr, Otyper, Pupdr};
use core::cell::RefCell;

struct GpioContext
{
    moder: Moder,
    otyper: Otyper,
    ospeedr: Ospeedr,
    pupdr: Pupdr,
    odr: Odr,
    lckr: Lckr,
    afrl: Afr,
    afrh: Afr,
}

impl GpioContext
{
    fn from_peripheral(gpio: Gpio) -> GpioContext
    {
        GpioContext { 
            moder: gpio.moder().read(),
            otyper: gpio.otyper().read(), 
            ospeedr: gpio.ospeedr().read(), 
            pupdr: gpio.pupdr().read(), 
            odr: gpio.odr().read(), 
            lckr: gpio.lckr().read(),
            afrl: gpio.afr(0).read(), 
            afrh: gpio.afr(1).read(), 
            }
    }

    fn into_peripheral(&self, gpio: Gpio)
    {
        gpio.moder().write_value(self.moder);
        gpio.otyper().write_value(self.otyper);
        gpio.ospeedr().write_value(self.ospeedr);
        gpio.pupdr().write_value(self.pupdr);
        gpio.odr().write_value(self.odr);
        gpio.lckr().write_value(self.lckr);
        gpio.afr(0).write_value(self.afrl);
        gpio.afr(1).write_value(self.afrh);
    }

    pub const fn new() -> GpioContext {
        Self { moder: Moder(0), otyper: Otyper(0), ospeedr: Ospeedr(0), pupdr: Pupdr(0), odr: Odr(0), lckr: Lckr(0), afrl: Afr(0), afrh: Afr(0) }
    }
}

static CONTEXT_GPIOA: Mutex<RefCell<GpioContext>> = Mutex::new(RefCell::new(GpioContext::new()));
static CONTEXT_GPIOB: Mutex<RefCell<GpioContext>> = Mutex::new(RefCell::new(GpioContext::new()));
static CONTEXT_GPIOC: Mutex<RefCell<GpioContext>> = Mutex::new(RefCell::new(GpioContext::new()));
static CONTEXT_GPIOD: Mutex<RefCell<GpioContext>> = Mutex::new(RefCell::new(GpioContext::new()));
static CONTEXT_GPIOE: Mutex<RefCell<GpioContext>> = Mutex::new(RefCell::new(GpioContext::new()));
static CONTEXT_GPIOH: Mutex<RefCell<GpioContext>> = Mutex::new(RefCell::new(GpioContext::new()));

fn store_gpio_context() {
    cortex_m::interrupt::free(|cs| {
        *CONTEXT_GPIOA.borrow(cs).borrow_mut() =
            GpioContext::from_peripheral(crate::pac::GPIOA);
        *CONTEXT_GPIOB.borrow(cs).borrow_mut() =
            GpioContext::from_peripheral(crate::pac::GPIOB);
        *CONTEXT_GPIOC.borrow(cs).borrow_mut() =
            GpioContext::from_peripheral(crate::pac::GPIOC);
        *CONTEXT_GPIOD.borrow(cs).borrow_mut() =
            GpioContext::from_peripheral(crate::pac::GPIOD);
        *CONTEXT_GPIOE.borrow(cs).borrow_mut() =
            GpioContext::from_peripheral(crate::pac::GPIOE);
        *CONTEXT_GPIOH.borrow(cs).borrow_mut() =
            GpioContext::from_peripheral(crate::pac::GPIOH);
    });
}

fn recover_gpio_context() {
    cortex_m::interrupt::free(|cs| {
        CONTEXT_GPIOA
            .borrow(cs)
            .borrow()
            .into_peripheral(crate::pac::GPIOA);

        CONTEXT_GPIOB
            .borrow(cs)
            .borrow()
            .into_peripheral(crate::pac::GPIOB);

        CONTEXT_GPIOC
            .borrow(cs)
            .borrow()
            .into_peripheral(crate::pac::GPIOC);

        CONTEXT_GPIOD
            .borrow(cs)
            .borrow()
            .into_peripheral(crate::pac::GPIOD);

        CONTEXT_GPIOE
            .borrow(cs)
            .borrow()
            .into_peripheral(crate::pac::GPIOE);

        CONTEXT_GPIOH
            .borrow(cs)
            .borrow()
            .into_peripheral(crate::pac::GPIOH);
    });
}

fn put_all_gpios_into_analog()
{
    crate::pac::GPIOA.moder().write_value(Moder(0xEBFF_FCFF)); // 0xEBFF_FCFF - reset value!
    crate::pac::GPIOB.moder().write_value(Moder(0xFFFF_FFFF));
    crate::pac::GPIOC.moder().write_value(Moder(0xFFFF_FFFF));
    crate::pac::GPIOD.moder().write_value(Moder(0xFFFF_FFFF));
    crate::pac::GPIOE.moder().write_value(Moder(0xFFFF_FFFF));
    crate::pac::GPIOH.moder().write_value(Moder(0xFFFF_FFFF));
}
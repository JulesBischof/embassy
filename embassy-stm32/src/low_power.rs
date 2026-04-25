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
use stm32_metapac::rcc::regs::{Ahbenr, Apb1enr, Apb2enr, Gpioenr};

use crate::dma::info;
#[cfg(all(feature = "rt", not(feature = "_lp-time-driver")))]
use crate::interrupt;
use crate::peripherals::RCC;
pub use crate::rcc::StopMode;
use crate::rcc::{LsConfig, get_stop_mode};
use crate::time_driver::{LPTimeDriver, get_driver};

use stm32_metapac::gpio::Gpio;
use stm32_metapac::gpio::regs::{Afr, Lckr, Moder, Odr, Ospeedr, Otyper, Pupdr};
use core::cell::RefCell;

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
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

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
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
        Self { moder: Moder(0), 
            otyper: Otyper(0), 
            ospeedr: Ospeedr(0), 
            pupdr: Pupdr(0), odr: 
            Odr(0), lckr: Lckr(0), 
            afrl: Afr(0), 
            afrh: Afr(0) 
        }
    }
}

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
struct GpioContexts {
    pub a: GpioContext,
    pub b: GpioContext,
    pub c: GpioContext,
    pub d: GpioContext,
    pub e: GpioContext,
    pub h: GpioContext,
}

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
static GPIO_CONTEXTS: critical_section::Mutex<RefCell<Option<GpioContexts>>> =  critical_section::Mutex::new(RefCell::new(None));

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
static GPIO_RCC_STATE: critical_section::Mutex<RefCell<Option<Gpioenr>>> = critical_section::Mutex::new(RefCell::new(None));

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
fn _store_gpio_context(cs: CriticalSection) {

    // store gpio configureations
    let contexts = GpioContexts {
        a: GpioContext::from_peripheral(crate::pac::GPIOA),
        b: GpioContext::from_peripheral(crate::pac::GPIOB),
        c: GpioContext::from_peripheral(crate::pac::GPIOC),
        d: GpioContext::from_peripheral(crate::pac::GPIOD),
        e: GpioContext::from_peripheral(crate::pac::GPIOE),
        h: GpioContext::from_peripheral(crate::pac::GPIOH),
    };
    *GPIO_CONTEXTS.borrow(cs).borrow_mut() = Some(contexts);

    // store gpio rcc state
    let rcc_gpioenr = crate::pac::RCC.gpioenr().read();
    *GPIO_RCC_STATE.borrow(cs).borrow_mut() = Some(rcc_gpioenr);
}

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
fn _recover_gpio_context(cs: CriticalSection) 
{
    let mut ctxt_ref = GPIO_CONTEXTS
                                            .borrow(cs)
                                            .borrow_mut();

    let context = ctxt_ref.as_mut().expect("GPIO CONTEXT WAS NOT INTIZIALIDED");

    context.a.into_peripheral(crate::pac::GPIOA);
    context.b.into_peripheral(crate::pac::GPIOB);
    context.c.into_peripheral(crate::pac::GPIOC);
    context.d.into_peripheral(crate::pac::GPIOD);
    context.e.into_peripheral(crate::pac::GPIOE);
    context.h.into_peripheral(crate::pac::GPIOH);

    let mut rcc_gpioenr_ref = GPIO_RCC_STATE
                                                            .borrow(cs)
                                                            .borrow_mut();

    let rcc_gpioenr = rcc_gpioenr_ref
                                        .as_mut()
                                        .expect("GPIO RCC STATE WAS NOT INITIZED");

    crate::pac::RCC.gpioenr().write(|_| rcc_gpioenr);
}

fn _gpio_to_analog(gpio: Gpio)
{
    gpio.moder().write_value(Moder(0xFFFF_FFFF)); // 0xFF = analog
    gpio.pupdr().write_value(Pupdr(0x0000_0000)); // 0x00 = floating
}

#[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
fn _disable_all_gpios()
{
    // enable all gpios - in order to let the changes take effect
    crate::pac::RCC.gpioenr().write(|w|
    {
        w.set_gpioaen(true);
        w.set_gpioben(true);
        w.set_gpiocen(true);
        w.set_gpioden(true);
        w.set_gpioeen(true);
        w.set_gpiohen(true);
    });

    // config all gpios as analog
    _gpio_to_analog(crate::pac::GPIOA);
    _gpio_to_analog(crate::pac::GPIOB);
    _gpio_to_analog(crate::pac::GPIOC);
    _gpio_to_analog(crate::pac::GPIOD);
    _gpio_to_analog(crate::pac::GPIOE);
    _gpio_to_analog(crate::pac::GPIOH);

    // disable all gpios
    crate::pac::RCC.gpioenr().write(|w|
    {
        w.set_gpioaen(false);
        w.set_gpioben(false);
        w.set_gpiocen(false);
        w.set_gpioden(false);
        w.set_gpioeen(false);
        w.set_gpiohen(false);
    });
}

static RCC_AHBENR_STATE: critical_section::Mutex<RefCell<Option<Ahbenr>>> = critical_section::Mutex::new(RefCell::new(None));
static RCC_APB1ENR_STATE: critical_section::Mutex<RefCell<Option<Apb1enr>>> = critical_section::Mutex::new(RefCell::new(None));
static RCC_APB2ENR_STATE: critical_section::Mutex<RefCell<Option<Apb2enr>>> = critical_section::Mutex::new(RefCell::new(None));

fn _store_rcc_state(cs: CriticalSection<'_>)
{
    let ahbenr = crate::pac::RCC.ahbenr().read();
    let apb1enr = crate::pac::RCC.apb1enr().read();
    let apb2enr = crate::pac::RCC.apb2enr().read();

    *RCC_AHBENR_STATE.borrow(cs).borrow_mut() = Some(ahbenr);
    *RCC_APB1ENR_STATE.borrow(cs).borrow_mut() = Some(apb1enr);
    *RCC_APB2ENR_STATE.borrow(cs).borrow_mut() = Some(apb2enr);
}

fn _gate_all_peripherals()
{
    crate::pac::RCC.ahbenr().write_value(Ahbenr(0 as u32));
    crate::pac::RCC.apb1enr().write_value(Apb1enr(0 as u32));
    crate::pac::RCC.apb2enr().write_value(Apb2enr(0 as u32));
}

fn _restore_rcc_state(cs: CriticalSection<'_>)
{
    let ahbenr  = RCC_AHBENR_STATE.borrow(cs).borrow_mut().unwrap();
    let apb1enr = RCC_APB1ENR_STATE.borrow(cs).borrow_mut().unwrap();
    let apb2enr = RCC_APB2ENR_STATE.borrow(cs).borrow_mut().unwrap();
    
    crate::pac::RCC.ahbenr().write_value(ahbenr);
    crate::pac::RCC.apb1enr().write_value(apb1enr);
    crate::pac::RCC.apb2enr().write_value(apb2enr);
}

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

            use stm32_metapac::pwr::vals::Mode;
            use crate::pac::pwr::vals::Pdds;
            
            crate::pac::PWR.cr().modify(|w| {
                w.set_cwuf(true);
                w.set_ulp(true); // ref (page 151, RM0376)
                w.set_fwu(true); // fast wakeup https://forum.digikey.com/t/low-power-modes-on-the-stm32l0-series/13306
                w.set_pdds(Pdds::STOP_MODE);
                w.set_lpsdsr(Mode::LOW_POWER_MODE); // ref (page 168, RM0376)
            });

            // disable internal voltage reference
            crate::pac::SYSCFG.cfgr3().modify(|w| {w.set_en_vrefint(false)});

            // disable adc / dac
            crate::pac::ADC1.cr().modify(|w| w.set_addis(true));
            crate::pac::DAC1.cr().modify(|w| w.set_en(0, false));
            crate::pac::DAC1.cr().modify(|w| w.set_en(1, false));

            /* -- some experimental settings -- */
            // let mut syst = unsafe{cortex_m::Peripherals::steal().SYST};
            // syst.disable_counter();
            // syst.disable_interrupt();

            // crate::pac::RCC.csr().modify(|w| {
            //     w.set_csslseon(false);
            //     w.set_lseon(false);
            //     }
            // );

            // crate::pac::SYSCFG.cfgr3().modify(|w|
            // {
            //     w.set_enref_hsi48(false);
            //     w.set_enbuf_vrefint_comp2(false);
            //     w.set_enbuf_sensor_adc(false);
            //     w.set_enbuf_vrefint_adc(false);
            //     w.set_sel_vref_out(0b00);
            // });

            // super::pre_stop_register_dump();

            // super::_store_rcc_state(_cs);
            // super::_gate_all_peripherals();
            /* -- end -- */
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

            /* JULIAN EDIT */
            // super::_restore_rcc_state(_cs);

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

    get_scb().clear_sleepdeep();

    platform::clear_flags();

    compiler_fence(Ordering::Acquire);

    let Some(stop_mode) = get_stop_mode(cs) else {
        #[cfg(feature = "low-power-use-low-power-sleep")]
        enter_low_power_sleep(cs);
        return;
    };

    if get_driver().pause_time(cs).is_err() 
    { warn!("low_power: failed to pause time, not entering stop");
    } else if platform::enter_stop(cs, stop_mode).is_err() {
        warn!("low_power: failed to enter stop");
    } else {
        #[cfg(stm32l0)]
        trace!("low power: enter stop");
        #[cfg(not(stm32l0))]
        trace!("low power: enter stop: {}", stop_mode);

        STOP_ENTERED.store(true, Ordering::Release);

        #[cfg(not(feature = "low-power-debug-with-sleep"))]
        get_scb().set_sleepdeep();
    }
}


fn enter_low_power_sleep(_: CriticalSection)
{
    trace!("enter low-power-sleep!");
    crate::pac::FLASH.acr().modify(|w| w.set_sleep_pd(true));
    crate::pac::PWR.cr().modify(|w| w.lpsdsr());
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
    #[cfg(feature = "low-power-use-low-power-sleep")]
    crate::rcc::gearshift_slow(cs);

    configure_pwr(cs);

    #[cfg(feature = "low-power-defmt-flush")]
    defmt::flush();

    #[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
    {
        _store_gpio_context(cs);
        _disable_all_gpios();
    }

    cortex_m::asm::dsb();
    cortex_m::asm::wfi();
    cortex_m::asm::isb();

    #[cfg(all(feature = "low-power-no-gpios-while-stop", stm32l0))]
    {
        _recover_gpio_context(cs);
    }

    on_wakeup(cs);

    #[cfg(feature = "low-power-use-low-power-sleep")]
    crate::rcc::gearshift_fast(cs);
}


fn pre_stop_register_dump() {

   /* relevant bit | description
    * 
    * SYSCFG_CFGR2 
    *      - 0 - Firewall disabled when 1 
    *        
    * SYSCFG_CFGR3
    *      - 13 - VREFINT for HSI48 enabled
    *      - 12 - VREFINT for COMP2 enabled
    *      - 9  - TEMPSENSOR ref for ADC enabled
    *      - 8  - VREFINT for ADC enabled
    *      - 5-4- VREFINT_ADC connection bit (shall be 00 )
    *      - 0  - VREFINT enabled
    * 
    * DMA_CCRx (channels 1 .. 6) 
    *      - 2  - half interrupt enable
    *      - 1  - tx complete interrupt enable
    *      - 0  - channel enable
    *
    * ADC_CR 
    *      - 0 - ADCEN
    * 
    * DAC_CR
    *      - 0 - DACEN
    * 
    * COMP1_CSR
    *      - 0 - COMP1_EN
    * 
    * COMP2_CSR
    *      - 0 - COMP2_EN
    * 
    * TSC_CR
    *      - 0 - TSCE - Touch Sensor Enable
    * 
    * AES_CR                                !!! NOT PART OF STM32-PAC
    *      - 0 - AES Enable
    * 
    * RNG_CR
    *      - 2 - RNG (random generator) enable
    * 
    * TIMx_CR1 (TIM1 / TIM2 / TIM3 / TIM6 / TIM7 / TIM21 / TIM22 / LPTIM)
    *      - 0 - Counter enable
    * 
    * WWDG_CR
    *      - 7 - WDGA Waatchdog enabled
    * 
    * I2Cx_CR1 (x = 1..3)
    *      - 0 - Peripheral enable
    * 
    * USARTx_CR1 (x = 1/2/4/5)
    *      - 2 Rx Enable
    *      - 1 USART enable in Stop Mode
    *      - 0 USART Enable
    *      
    * LPUART_CR1
    *      - 0 Uart Enable
    * 
    * SPI_CR1
    *      - 6 - SPE - SPI enable
    * 
    * USB_CNTR
    *      - 1 - PWDN Power Down
    *      - 0 - Force USB Reset
    * 
    * DBG_CR - SIDENOTE thiese bits get set on a power on reset, NOT on a system Reset !! therefore to disable the debug unit after restart, a powercycle must be done
    *      2 - Debug Standby
    *      1 - Debug Stop
    *      0 - Debug Sleep
    * 
    * GPIOx_AFRL (x = A...E + H)
    *      -> should be 00 (alternatice function selection)
    * 
    * GPIOx_MODER
    *      -> should ne 0xFFFF_FFFF (analog)
    * 
    * CRS_CR (quote: CRS stops operating until stop is exited, registers are frozen during Stop)
    *      5 - enable (frequency error counter)
    * 
    * RCC_CR
    *      24 - PLL enable
    *      19 - CSSHSEON - Clock security system on HSE enable
    *      16 - HSEON
    *      9  - MSIRDY
    *      8  - MSION
    *      5  - HSI16OUT
    *      1  - HSI16KERON (HSI16 is Forced on, even in Stop Mode)
    *      0  - HSI16ON
    * 
    * RCC_CRRCR
    *      0 - HSI48ON (the HSI48 is disabled during STOP automatically)
    * 
    * RCC_IOPEN
    *      -> shall be 0, enabled GPIOs
    * 
    * RCC_AHBENR (clock enables)
    *      24 - Crypto
    *      20 - Random Generator
    *      16 - Tocuh sensing 
    *      12 - CRC
    *      8  - Flash Clock 
    *      0  - DMA clock
    * 
    * RCC_APB2ENR
    *      22 - Debugger
    *      14 - USART1
    *      12 - SPI1
    *      9  - ADC1
    *      7  - FWEN
    *      5  - TIM22
    *      2  - TIM21
    *      0  - SYSCFG
    * 
    * RCC_APB1ENR
    *      31 - LPTIM1
    *      30 - I2C3EN
    *      29 - DACEN
    *      28 - PWREN
    *      27 - CRSEN
    *      23 - USBEN
    *      22 - I2CEN
    *      21 - I2C1EN
    *      20 - USART5EN
    *      19 - USART4EN
    *      18 - LPUART1EN
    *      17 - USART2EN
    *      14 - SPI2EN
    *      11 - WWDGEN
    *      5  - TIM7EN
    *      4  - TIM6EN
    *      1  - TIM3EN
    *      0  - TIM2EN
    * 
    * PWR_CR
    *      14 - LPRUN
    *      13 - DS_EE_KOFF - Flash will not be woken up when exiting STOP
    *      12,11- Voltage Range selection (0b11 = Range 3 = 1.2V - Low Power) )
    *      9  - ULP - Ultra Low Power (Now VREFINT during low power mode) -> shall be 1
    *      1  - PDDS - 0 = enter Stop; 1 = enter Standby
    *      0  - LPSDSR Voltage Regulator enters low power mode, when the cpu enters these modes
    * 
    * FLASH_ACR
    *      4  - RUN_PD - NVM is in power down mode when mcu in run mode
    *      3  - SLEEP_PD - NVM is in power down mode during sleep
    * 
    */

    // SYSCFG
    let syscfg_cfgr2 = crate::pac::SYSCFG.cfgr2().read();
    let syscfg_cfgr3 = crate::pac::SYSCFG.cfgr3().read();

    // DMA Channels (1-6)
    let dma_ccr1 = crate::pac::DMA1.ch(0).cr().read();
    let dma_ccr2 = crate::pac::DMA1.ch(1).cr().read();
    let dma_ccr3 = crate::pac::DMA1.ch(2).cr().read();
    let dma_ccr4 = crate::pac::DMA1.ch(3).cr().read();
    let dma_ccr5 = crate::pac::DMA1.ch(4).cr().read();
    let dma_ccr6 = crate::pac::DMA1.ch(5).cr().read();

    // ADC & DAC
    let adc_cr = crate::pac::ADC1.cr().read();
    let dac_cr = crate::pac::DAC1.cr().read();

    // Comparators
    let comp1_csr = crate::pac::COMP1;
    let comp2_csr = crate::pac::COMP2;

    // Touch Sensing Controller
    let tsc_cr = crate::pac::TSC.cr().read();

    // AES & RNG
    // let aes_cr = crate::pac::AES.cr().read();
    let rng_cr = crate::pac::RNG.cr().read();

    // Timers
    let tim2_cr1 = crate::pac::TIM2.cr1().read();
    let tim3_cr1 = crate::pac::TIM3.cr1().read();
    let tim6_cr1 = crate::pac::TIM6.cr1().read();
    let tim7_cr1 = crate::pac::TIM7.cr1().read();
    let tim21_cr1 = crate::pac::TIM21.cr1().read();
    let tim22_cr1 = crate::pac::TIM22.cr1().read();
    let lptim_cr = crate::pac::LPTIM1.cr().read();

    // Watchdog
    let wwdg_cr = crate::pac::WWDG.cr().read();

    // I2C
    let i2c1_cr1 = crate::pac::I2C1.cr1().read();
    let i2c2_cr1 = crate::pac::I2C2.cr1().read();
    let i2c3_cr1 = crate::pac::I2C3.cr1().read();

    // USART
    let usart1_cr1 = crate::pac::USART1.cr1().read();
    let usart2_cr1 = crate::pac::USART2.cr1().read();
    let usart4_cr1 = crate::pac::USART4.cr1().read();
    let usart5_cr1 = crate::pac::USART5.cr1().read();

    // LPUART
    let lpuart_cr1 = crate::pac::LPUART1.cr1().read();

    // SPI
    let spi1_cr1 = crate::pac::SPI1.cr1().read();

    // USB
    let usb_cntr = crate::pac::USB.cntr().read();

    // Debug
    let dbg_cr = crate::pac::DBGMCU.cr().read();

    // GPIO (Ports A, B, C, D, E, H)
    let gpioa_moder = crate::pac::GPIOA.moder().read();
    let gpioa_afrl = crate::pac::GPIOA.afr(0).read();
    let gpiob_moder = crate::pac::GPIOB.moder().read();
    let gpiob_afrl = crate::pac::GPIOB.afr(0).read();
    let gpioc_moder = crate::pac::GPIOC.moder().read();
    let gpioc_afrl = crate::pac::GPIOC.afr(0).read();
    let gpiod_moder = crate::pac::GPIOD.moder().read();
    let gpiod_afrl = crate::pac::GPIOD.afr(0).read();
    let gpioe_moder = crate::pac::GPIOE.moder().read();
    let gpioe_afrl = crate::pac::GPIOE.afr(0).read();
    let gpioh_moder = crate::pac::GPIOH.moder().read();
    let gpioh_afrl = crate::pac::GPIOH.afr(0).read();

    // CRS
    let crs_cr = crate::pac::CRS.cr().read();

    // RCC
    let rcc_cr = crate::pac::RCC.cr().read();
    let rcc_crrcr = crate::pac::RCC.crrcr().read();
    let rcc_iopen = crate::pac::RCC.gpioenr().read();
    let rcc_ahbenr = crate::pac::RCC.ahbenr().read();
    let rcc_apb2enr = crate::pac::RCC.apb2enr().read();
    let rcc_apb1enr = crate::pac::RCC.apb1enr().read();

    // Power & Flash
    let pwr_cr = crate::pac::PWR.cr().read();
    let flash_acr = crate::pac::FLASH.acr().read();

    loop {
        // stop here in order to inspect the previous values
    }
}
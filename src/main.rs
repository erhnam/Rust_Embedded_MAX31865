#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

// Halt on panic
use panic_halt as _; // panic handler

use stm32f4xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    spi::Spi,
};

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use max31865::{FilterMode, Max31865, SensorType};

#[entry]
fn main() -> ! {
    // The Stm32 peripherals
    let dp = pac::Peripherals::take().expect("cannot take peripherals");
    // The Cortex-m peripherals
    let cp = cortex_m::peripheral::Peripherals::take().unwrap();

    // Constrain clock registers
    let rcc = dp.RCC.constrain();
    let clocks = rcc.cfgr.sysclk(84.MHz()).freeze();

    let _syscfg = dp.SYSCFG.constrain();

    // Create a delay abstraction based on SysTick
    let mut delay = cp.SYST.delay(&clocks);

    let gpioa = dp.GPIOA.split();

    // Configure SPI1 pins
    let ncs = gpioa.pa4.into_push_pull_output();
    let sck = gpioa.pa5.into_alternate();
    let miso = gpioa.pa6; //spi::NoMiso {};
    let mosi = gpioa.pa7.into_alternate();
    let rdy = gpioa.pa8;

    // Initializate SPI
    let spi = Spi::new(dp.SPI1, (sck, miso, mosi), max31865::MODE, 5.MHz(), &clocks);

    let mut max31865 = Max31865::new(spi, ncs, rdy).unwrap();

    // Optionally set the calibration reference resistance by specifying the
    // reference resistance in ohms multiplied by 100. See documentation for
    // `set_calibration` function.

    max31865.set_calibration(43234);
    max31865.configure(
        true,
        true,
        false,
        SensorType::ThreeWire,
        FilterMode::Filter50Hz,
    ).unwrap();

    let mut last = 0;

    hprintln!("Testing MAX31865");

    loop {
        if max31865.is_ready().unwrap() {
            let temp = max31865.read_default_conversion().unwrap();

            hprintln!("Temperature: {}.{:0>2} ºC", temp / 100, (temp % 100).abs());

            if temp != last {
                last = temp;
            }
        } else {
            hprintln!("Max31865 Not ready");
        }

        delay.delay_ms(5000_u32);
    }
}
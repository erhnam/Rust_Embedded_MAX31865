#![deny(unsafe_code)]
#![allow(clippy::empty_loop)]
#![no_main]
#![no_std]

use embedded_hal::digital::v2::IoPin;
use embedded_hal::digital::v2::OutputPin;
// Halt on panic
use panic_halt as _; // panic handler

use stm32f4xx_hal as hal;

use crate::hal::{
    pac,
    prelude::*,
    spi::{Mode, Phase, Polarity, Spi},
};

use cortex_m_rt::entry;
use cortex_m_semihosting::hprintln;
use max31865::{FilterMode, Max31865, SensorType};

use w5500_dhcp::{Client as DhcpClient, Hostname, State as DhcpState};
use w5500_hl::Tcp;
use w5500_ll::{
    eh0::{reset, vdm::W5500},
    net::{Eui48Addr, Ipv4Addr, SocketAddrV4},
    Registers, Sn, VERSION,
};

const MAC_ADDRESS: Eui48Addr = Eui48Addr::new(0x82, 0x33, 0x56, 0x78, 0x9A, 0xBC);
const HOSTNAME: Hostname = Hostname::new_unwrapped("w5500-testsuite");
const DHCP_SN: Sn = Sn::Sn0;

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
    let gpiob = dp.GPIOB.split();

    // Configure SPI1 pins MAX31865
    let ncs1 = gpioa.pa4.into_push_pull_output();
    let sck1 = gpioa.pa5.into_alternate();
    let miso1 = gpioa.pa6; //spi::NoMiso {};
    let mosi1 = gpioa.pa7.into_alternate();
    let rdy: hal::gpio::Pin<'A', 8> = gpioa.pa8;

    // Configure SPI2 pins Ethernet
    let ncs2 = gpiob.pb12.into_push_pull_output();
    let sck2 = gpiob.pb13.into_alternate();
    let miso2 = gpiob.pb14;
    let mosi2 = gpiob.pb15.into_alternate();
    let mut rst = gpiob.pb1.into_push_pull_output();

    // Initializate SPI
    let spi1: Spi<pac::SPI1, (hal::gpio::Pin<'A', 5, hal::gpio::Alternate<5>>, hal::gpio::Pin<'A', 6>, hal::gpio::Pin<'A', 7, hal::gpio::Alternate<5>>)> = Spi::new(dp.SPI1, (sck1, miso1, mosi1), max31865::MODE, 5.MHz(), &clocks);
    let spi2 = Spi::new(dp.SPI2, (sck2, miso2, mosi2),  Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    }, 33.MHz(), &clocks);

    let mut max31865 = Max31865::new(spi1, ncs1, rdy).unwrap();

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

    hprintln!("Testing W5500");

    reset(&mut rst, &mut delay).unwrap();

    let mut w5500 = W5500::new(spi2, ncs2);

    hprintln!("Version: {}", w5500.version().unwrap());

    w5500.set_shar(&MAC_ADDRESS).unwrap();

    let dhcp_seed: u64 = 1234;
    
    let dhcp_client = DhcpClient::new(DHCP_SN, dhcp_seed, MAC_ADDRESS, HOSTNAME);

    hprintln!("Testing MAX31865");

    let mut last = 0;


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
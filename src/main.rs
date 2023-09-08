#![deny(unsafe_code)]
#![no_main]
#![no_std]
#![allow(clippy::type_complexity)]
#[warn(unused_mut, unused)]

// Halt on panic
use embedded_hal::digital::v2::OutputPin;
use panic_halt as _; // panic handler
use stm32f4xx_hal::{
    gpio::{
        self,
        gpioa::{PA4, PA5, PA6, PA7, PA8},
        gpiob::{PB1, PB12, PB13, PB14, PB15},
        AF5,
        Edge,
    },
    pac::{EXTI, SPI1, SPI2},
    prelude::*,
    spi::{Spi},
};
use core::fmt::Write;
use core::sync::atomic::{compiler_fence, Ordering::SeqCst};
use cortex_m_semihosting::{heprintln, hprintln};
use max31865::{FilterMode, Max31865, SensorType};

use w5500_dhcp::{
    hl::Hostname,
    ll::{
        eh0::vdm_infallible_gpio::W5500,
        eh0::MODE as W5500_MODE,
        net::{Eui48Addr, Ipv4Addr, SocketAddrV4},
        LinkStatus, OperationMode, PhyCfg, Registers, Sn,
    },
    Client as DhcpClient,
};

use w5500_mqtt::{Client as MqttClient, ClientId, Event as MqttEvent, SRC_PORT as MQTT_SRC_PORT};

use systick_monotonic::{Systick};

const MAC_ADDRESS: Eui48Addr = Eui48Addr::new(0x82, 0x33, 0x56, 0x78, 0x9A, 0xBC);
const DHCP_SN: Sn = Sn::Sn0;
const MQTT_SN: Sn = Sn::Sn1;

const MQTT_SERVER: SocketAddrV4 = SocketAddrV4::new(Ipv4Addr::new(192, 168, 0, 34), 1883);
const NAME: &str = "temperature";
const HOSTNAME: Hostname<'static> = Hostname::new_unwrapped(NAME);
const CLIENT_ID: ClientId<'static> = ClientId::new_unwrapped(NAME);
const TEMPERATURE_TOPIC: &str = "/home/temperature";

const SYSCLK_HZ: u32 = 8_000_000;

pub struct CycleDelay;

impl Default for CycleDelay {
    fn default() -> CycleDelay {
        CycleDelay
    }
}

impl embedded_hal::blocking::delay::DelayMs<u8> for CycleDelay {
    fn delay_ms(&mut self, ms: u8) {
        delay_ms(ms.into())
    }
}

/// Worlds worst delay function.
#[inline(always)]
pub fn delay_ms(ms: u32) {
    const CYCLES_PER_MILLIS: u32 = SYSCLK_HZ / 1000;
    cortex_m::asm::delay(CYCLES_PER_MILLIS.saturating_mul(ms));
}

fn monotonic_secs() -> u32 {
    app::monotonics::now()
        .duration_since_epoch()
        .to_secs()
        .try_into()
        .unwrap()
}

#[rtic::app(
    device = stm32f4xx_hal::pac, peripherals = true,
    dispatchers = [USART1, USART2],
)]
mod app {
    use super::*;

    // RTIC manual says not to use this in production.
    #[monotonic(binds = SysTick, default = true)]
    type MyMono = Systick<1>; // 1 Hz / 1 s granularity

    #[shared]
    struct Shared {
        w5500: W5500<Spi<SPI2>, gpio::Pin<'B', 12, gpio::Output>>,
        dhcp: DhcpClient<'static>,
        mqtt: MqttClient<'static>,
        dhcp_spawn_at: Option<u32>,
        mqtt_spawn_at: Option<u32>,
    }

    #[local]
    struct Local {
        exti: EXTI,
        max31865: Max31865<Spi<SPI1>, gpio::Pin<'A', 4, gpio::Output>, gpio::Pin<'A', 8>>,
    }

    #[init]
    fn init(cx: init::Context) -> (Shared, Local, init::Monotonics) {
        hprintln!("Initializate");

        // symptom of a version mismatch when using the RTIC alpha
        // see: https://github.com/rust-embedded/cortex-m/pull/350
        // replace with `cx.cs` when cortex-m gets updated
        let cs = cx.cs;

        // The Stm32 peripherals
        let mut dp = cx.device;

        // Constrain clock registers
        let rcc = dp.RCC.constrain();

        //let clocks = rcc.cfgr.use_hse(8.MHz()).sysclk(84.MHz()).freeze();
        let clocks = rcc.cfgr.use_hse(8.MHz()).freeze();
        //let clocks = rcc.cfgr.use_hse(25.MHz()).sysclk(48.MHz()).freeze();
        let mut syscfg = dp.SYSCFG.constrain();
        let mut exti: EXTI = dp.EXTI;

        let systick = cx.core.SYST;

        let mono = Systick::new(systick, SYSCLK_HZ);

        let gpioa = dp.GPIOA.split();
        let gpiob = dp.GPIOB.split();

        hprintln!("Configurado GPIO");

        // Configure SPI1 pins MAX31865
        let ncs1 = gpioa.pa4.into_push_pull_output();
        let sck1 = gpioa.pa5.into_alternate();
        let miso1 = gpioa.pa6; //spi::NoMiso {};
        let mosi1 = gpioa.pa7.into_alternate();
        let rdy: gpio::Pin<'A', 8> = gpioa.pa8;

        // Configure SPI2 pins Ethernet
        let w5500_cs = gpiob.pb12.into_push_pull_output();
        let w5500_sck = gpiob.pb13.into_alternate();
        let w5500_miso = gpiob.pb14.into_alternate();
        let w5500_mosi = gpiob.pb15.into_alternate();
        let mut w5500_rst = gpiob.pb1.into_push_pull_output();

        let mut w5500_int = gpiob.pb2.into_pull_down_input(); // INT
        
        // enable external interrupt for (W5500 interrupt)
        w5500_int.make_interrupt_source(&mut syscfg);
        w5500_int.trigger_on_edge(&mut exti, Edge::Falling);
        w5500_int.enable_interrupt(&mut exti);

        hprintln!("Inicializando SPI");

        // Initializate SPI
        let spi1 = Spi::new(
            dp.SPI1,
            (sck1, miso1, mosi1),
            max31865::MODE,
            5.MHz(),
            &clocks,
        );
        let spi_w5500 = Spi::new(
            dp.SPI2,
            (w5500_sck, w5500_miso, w5500_mosi),
            W5500_MODE,
            1.MHz(),
            &clocks,
        );

        hprintln!("SPI Inicializado");

        hprintln!("Configurando MAX31865");

        let mut max31865: Max31865<Spi<SPI1>, gpio::Pin<'A', 4, gpio::Output>, gpio::Pin<'A', 8>> =
            Max31865::new(spi1, ncs1, rdy).unwrap();

        // Optionally set the calibration reference resistance by specifying the
        // reference resistance in ohms multiplied by 100. See documentation for
        // `set_calibration` function.

        max31865.set_calibration(43234);
        max31865
            .configure(
                true,
                true,
                false,
                SensorType::ThreeWire,
                FilterMode::Filter50Hz,
            )
            .unwrap();

        hprintln!("MAX31865 configurado");

        hprintln!("Testing W5500");

        w5500_dhcp::ll::eh0::reset(&mut w5500_rst, &mut CycleDelay).unwrap();

        let mut w5500: W5500<Spi<SPI2>, gpio::Pin<'B', 12, gpio::Output>> =
            W5500::new(spi_w5500, w5500_cs);

        // wait for the PHY to indicate the Ethernet link is up
        let mut attempts: u32 = 0;

        hprintln!("Polling for link up");

        const PHY_CFG: PhyCfg = PhyCfg::DEFAULT.set_opmdc(OperationMode::FullDuplex10bt);
        w5500.set_phycfgr(PHY_CFG).unwrap();

        const LINK_UP_POLL_PERIOD_MILLIS: u32 = 100;
        const LINK_UP_POLL_ATTEMPTS: u32 = 50;

        // continually initialize the W5500 until we link up
        // since we are using power over Ethernet we know that if the device
        // has power it also has an Ethernet cable connected.
        let _phy_cfg: PhyCfg = 'outer: loop {
            // sanity check W5500 communications
            assert_eq!(w5500.version().unwrap(), w5500_dhcp::ll::VERSION);

            // load the MAC address we got from EEPROM
            w5500.set_shar(&MAC_ADDRESS).unwrap();
            debug_assert_eq!(w5500.shar().unwrap(), MAC_ADDRESS);

            // wait for the PHY to indicate the Ethernet link is up
            let mut attempts: u32 = 0;
            hprintln!("Polling for link up");

            const PHY_CFG: PhyCfg = PhyCfg::DEFAULT.set_opmdc(OperationMode::FullDuplex10bt);
            w5500.set_phycfgr(PHY_CFG).unwrap();

            const LINK_UP_POLL_PERIOD_MILLIS: u32 = 100;
            const LINK_UP_POLL_ATTEMPTS: u32 = 50;
            loop {
                let phy_cfg: PhyCfg = w5500.phycfgr().unwrap();
                if phy_cfg.lnk() == LinkStatus::Up {
                    break 'outer phy_cfg;
                }
                if attempts >= LINK_UP_POLL_ATTEMPTS {
                    hprintln!(
                        "Failed to link up in {} ms",
                        attempts * LINK_UP_POLL_PERIOD_MILLIS
                    );
                    break;
                }
                delay_ms(LINK_UP_POLL_PERIOD_MILLIS);
                attempts += 1;
            }

            w5500_rst.set_low();
            delay_ms(1);
            w5500_rst.set_high();
            delay_ms(3);
        };
        hprintln!("Done link up\n{}", _phy_cfg);

        let mut mqtt: MqttClient = MqttClient::new(MQTT_SN, MQTT_SRC_PORT, MQTT_SERVER);
        mqtt.set_client_id(CLIENT_ID);

        let seed: u64 = u64::from(cortex_m::peripheral::SYST::get_current()) << 32
            | u64::from(cortex_m::peripheral::SYST::get_current());

        let dhcp = DhcpClient::new(DHCP_SN, seed, MAC_ADDRESS, HOSTNAME);
        dhcp.setup_socket(&mut w5500).unwrap();

        // start the DHCP client
        dhcp_sn::spawn().unwrap();

        // start the timeout tracker
        timeout_tracker::spawn().unwrap();

        (
            Shared {
                w5500,
                dhcp,
                mqtt,
                dhcp_spawn_at: None,
                mqtt_spawn_at: None,
            },
            Local { max31865, exti },
            init::Monotonics(mono),
        )
    }

    #[idle]
    fn idle(_: idle::Context) -> ! {
        hprintln!("[TASK] idle");
        loop {
            compiler_fence(SeqCst);
        }
    }

    #[task(shared = [w5500, dhcp, dhcp_spawn_at])]
    fn dhcp_sn(cx: dhcp_sn::Context) {
        hprintln!("[TASK] dhcp_sn");

        (cx.shared.w5500, cx.shared.dhcp, cx.shared.dhcp_spawn_at).lock(
            |w5500, dhcp, dhcp_spawn_at| {
                let leased_before: bool = dhcp.has_lease();
                let now: u32 = monotonic_secs();
                let spawn_after_secs: u32 = dhcp.process(w5500, now).unwrap();

                let spawn_at: u32 = now + spawn_after_secs;
                *dhcp_spawn_at = Some(spawn_at);
                hprintln!("[DHCP] spawning after {} seconds, at {}", spawn_after_secs, spawn_at);

                // spawn MQTT task if bound
                if dhcp.has_lease() && !leased_before && mqtt_sn::spawn().is_err() {
                    heprintln!("MQTT task is already spawned")
                }
            },
        )
    }

    /// This is the W5500 interrupt.
    ///
    /// The only interrupts we should get are for the DHCP & MQTT sockets.
    #[task(binds = EXTI0, local = [exti], shared = [w5500])]
    #[allow(clippy::collapsible_if)]
    fn exti0(mut cx: exti0::Context) {
        hprintln!("[TASK] exti0");

        cx.shared.w5500.lock(|w5500| {
            let sir: u8 = w5500.sir().unwrap();

            cx.local.exti.pr.write(|w| w.pr0().set_bit());

            // may occur when there are power supply issues
            if sir == 0 {
                hprintln!("[W5500] spurious interrupt");
                return;
            }

            if sir & DHCP_SN.bitmask() != 0 {
                if dhcp_sn::spawn().is_err() {
                    heprintln!("DHCP task already spawned")
                }
            }

            if sir & MQTT_SN.bitmask() != 0 {
                if mqtt_sn::spawn().is_err() {
                    heprintln!("MQTT task already spawned")
                }
            }
        });
    }

    #[task(shared = [dhcp_spawn_at, mqtt_spawn_at])]
    fn timeout_tracker(mut cx: timeout_tracker::Context) {
        timeout_tracker::spawn_after(systick_monotonic::ExtU64::secs(1)).unwrap();

        let now: u32 = monotonic_secs();

        cx.shared.dhcp_spawn_at.lock(|dhcp_spawn_at| {
            if let Some(then) = dhcp_spawn_at {
                if now >= *then {
                    if dhcp_sn::spawn().is_err() {
                       heprintln!("DHCP task is already spawned")
                    }
                    *dhcp_spawn_at = None;
                }
            }
        });

        cx.shared.mqtt_spawn_at.lock(|mqtt_spawn_at| {
            if let Some(then) = mqtt_spawn_at {
                if now >= *then {
                    if mqtt_sn::spawn().is_err() {
                        heprintln!("MQTT task is already spawned")
                    }
                    *mqtt_spawn_at = None;
                }
            }
        });
    }

    #[task(shared = [w5500, mqtt, mqtt_spawn_at], local = [max31865])]
    fn mqtt_sn(cx: mqtt_sn::Context) {
        hprintln!("[TASK] mqtt_sn");

        let max31865: &mut Max31865<Spi<SPI1>, gpio::Pin<'A', 4, gpio::Output>, gpio::Pin<'A', 8>> = cx.local.max31865;

        (cx.shared.w5500, cx.shared.mqtt, cx.shared.mqtt_spawn_at).lock(
            |w5500, mqtt, mqtt_spawn_at| {
                let mut last = 0;
                let mut temp = 0;

                loop {
                    let now: u32 = monotonic_secs();
                    match mqtt.process(w5500, now) {
                        Ok(MqttEvent::CallAfter(secs)) => {
                            *mqtt_spawn_at = Some(now + secs);
                            break;
                        }
                        Ok(MqttEvent::ConnAck) => {
                            hprintln!("[MQTT] ConnAck");
                            // can subscribe to topics here
                            // not needed for this
                        }
                        Ok(MqttEvent::Publish(reader)) => {
                            hprintln!("should not get Publish never subscribed");
                            reader.done().unwrap();
                        }
                        Ok(MqttEvent::SubAck(_) | MqttEvent::UnSubAck(_)) => {
                            hprintln!("should not get (Un)SubAck, never (un)subscribed");
                        }
                        Ok(MqttEvent::None) => {
                            hprintln!("Testing MAX31865");

                            if max31865.is_ready().unwrap() {
                                temp = max31865.read_default_conversion().unwrap();

                                if temp != last {
                                    last = temp;
                                }
                            } else {
                                hprintln!("Max31865 Not ready");
                            }

                            hprintln!("Temperature: {}.{:0>2} ºC", temp / 100, (temp % 100).abs());
                            {
                                let mut data: heapless::String<16> = heapless::String::new();
                                write!(&mut data, "{}.{:0>2} ºC", temp / 100, (temp % 100).abs())
                                    .unwrap();
                                mqtt.publish(w5500, TEMPERATURE_TOPIC, data.as_bytes())
                                    .unwrap();
                            }
                            *mqtt_spawn_at = Some(now + 5);
                            break;
                        }
                        Err(e) => {
                            heprintln!("[MQTT] {e:?}");
                            *mqtt_spawn_at = Some(now + 10);
                            break;
                        }
                    }
                }
            },
        );
    }
}

#![no_std]
#![no_main]
#![feature(type_alias_impl_trait)]
#![feature(async_closure)]

use core::cell::RefCell;

use bleps::{
    ad_structure::{
        create_advertising_data, AdStructure, BR_EDR_NOT_SUPPORTED, LE_GENERAL_DISCOVERABLE,
    },
    async_attribute_server::AttributeServer,
    asynch::Ble,
    attribute_server::NotificationData,
    gatt,
};
use embassy_executor::Spawner;
use embassy_time::{Duration, Timer};
use esp_backtrace as _;
use esp_println::println;
use esp_wifi::{ble::controller::asynch::BleConnector, initialize, EspWifiInitFor};
#[path = "../../examples-util/util.rs"]
mod examples_util;
use esp_hal::adc::{AdcConfig, Attenuation, ADC};
use esp_hal::Rmt;
use esp_hal_smartled::smartLedBuffer;
use esp_hal_smartled::SmartLedsAdapter;
use examples_util::hal;
use hal::{
    clock::ClockControl, embassy, i2c::I2C, peripherals::*, prelude::*, timer::TimerGroup, Rng, IO,
};
use smart_leds::{
    brightness,
    colors::{BLACK, DARK_RED, GREEN, VIOLET},
    gamma, SmartLedsWrite,
};
use vl53l0x::VL53L0x;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum SensedRange {
    VeryNear,
    Near,
    Far,
    None,
}

impl SensedRange {
    /// Converts a distance in mm to a `SensedRange`
    pub fn from_distance(distance: u16) -> Self {
        match distance {
            0..=200 => SensedRange::VeryNear,
            201..=600 => SensedRange::Near,
            601..=1000 => SensedRange::Far,
            _ => SensedRange::None,
        }
    }

    /// Returns the next time in ms to display the sensed range
    pub fn next_blink_loop(&self) -> u8 {
        match self {
            SensedRange::VeryNear => 2,
            SensedRange::Near => 4,
            SensedRange::Far => 6,
            SensedRange::None => 8,
        }
    }

    pub fn into_color(&self) -> [smart_leds::RGB<u8>; 1] {
        match self {
            SensedRange::VeryNear => [DARK_RED],
            SensedRange::Near => [VIOLET],
            SensedRange::Far => [GREEN],
            SensedRange::None => [BLACK],
        }
    }
}

#[main]
async fn main(_spawner: Spawner) -> ! {
    #[cfg(feature = "log")]
    esp_println::logger::init_logger(log::LevelFilter::Info);

    let peripherals = Peripherals::take();

    let system = peripherals.SYSTEM.split();
    let clocks = ClockControl::max(system.clock_control).freeze();

    #[cfg(target_arch = "xtensa")]
    let timer = hal::timer::TimerGroup::new(peripherals.TIMG1, &clocks).timer0;
    #[cfg(target_arch = "riscv32")]
    let timer = hal::systimer::SystemTimer::new(peripherals.SYSTIMER).alarm0;
    let init = initialize(
        EspWifiInitFor::Ble,
        timer,
        Rng::new(peripherals.RNG),
        system.radio_clock_control,
        &clocks,
    )
    .unwrap();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);
    cfg_if::cfg_if! {
        if #[cfg(feature = "esp32")] {
            let led_pin = io.pins.gpio33;
        } else if #[cfg(feature = "esp32c3")] {
            let led_pin = io.pins.gpio2;
        } else if #[cfg(any(feature = "esp32c6", feature = "esp32h2"))] {
            let led_pin = io.pins.gpio8;
        } else if #[cfg(feature = "esp32s2")] {
            let led_pin = io.pins.gpio18;
        } else if #[cfg(feature = "esp32s3")] {
            let led_pin = io.pins.gpio48;
        }
    }
    let rmt = Rmt::new(
        peripherals.RMT,
        esp_hal::prelude::_fugit_RateExtU32::MHz(80),
        &clocks,
    )
    .unwrap();
    let mut adc1_config = AdcConfig::new();
    // ESP32-C3 has A2 on GPIO1 according to https://learn.adafruit.com/adafruit-qt-py-esp32-c3-wifi-dev-board/pinouts
    let a2_pin = adc1_config.enable_pin(io.pins.gpio1.into_analog(), Attenuation::Attenuation11dB);
    let adc1 = &RefCell::new(ADC::<ADC1>::new(peripherals.ADC1, adc1_config));

    // We use one of the RMT channels to instantiate a `SmartLedsAdapter` which can
    // be used directly with all `smart_led` implementations
    let rmt_buffer = smartLedBuffer!(1);
    let led = SmartLedsAdapter::new(rmt.channel0, led_pin, rmt_buffer, &clocks);
    let i2c = I2C::new(
        peripherals.I2C0,
        io.pins.gpio5,
        io.pins.gpio6,
        100u32.kHz(),
        &clocks,
    );
    let mut tof = VL53L0x::new(i2c).unwrap();
    tof.set_measurement_timing_budget(20_000).unwrap();
    tof.start_continuous(0).unwrap();

    #[cfg(any(feature = "esp32", feature = "esp32s2", feature = "esp32s3"))]
    let _button = io.pins.gpio0.into_pull_down_input();
    #[cfg(any(
        feature = "esp32c2",
        feature = "esp32c3",
        feature = "esp32c6",
        feature = "esp32h2"
    ))]
    let _button = io.pins.gpio9.into_pull_down_input();

    // Async requires the GPIO interrupt to wake futures
    hal::interrupt::enable(
        hal::peripherals::Interrupt::GPIO,
        hal::interrupt::Priority::Priority1,
    )
    .unwrap();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    embassy::init(&clocks, timer_group0);

    let mut bluetooth = peripherals.BT;

    let next_blink_loop = &RefCell::new(SensedRange::None.next_blink_loop());

    let connector = BleConnector::new(&init, &mut bluetooth);
    let mut ble = Ble::new(connector, esp_wifi::current_millis);
    println!("Connector created");

    let tof_ref = &RefCell::new(tof);
    let current_sensed_range = &RefCell::new(SensedRange::None);
    let led_ref = &RefCell::new(led);

    let a2_pin = &RefCell::new(a2_pin);
    let a2_reading = &RefCell::new(0u16);

    loop {
        println!("{:?}", ble.init().await);
        println!("{:?}", ble.cmd_set_le_advertising_parameters().await);
        println!(
            "{:?}",
            ble.cmd_set_le_advertising_data(
                create_advertising_data(&[
                    AdStructure::Flags(LE_GENERAL_DISCOVERABLE | BR_EDR_NOT_SUPPORTED),
                    AdStructure::ServiceUuids16(&[Uuid::Uuid16(0x1809)]),
                    AdStructure::CompleteLocalName(examples_util::SOC_NAME),
                ])
                .unwrap()
            )
            .await
        );
        println!("{:?}", ble.cmd_set_le_advertise_enable(true).await);

        println!("started advertising");

        let mut wf = |offset: usize, data: &[u8]| {
            // TODO: Blink LED
            println!("RECEIVED: {} {:?}", offset, data);
        };

        let mut rf_distance = |_offset: usize, data: &mut [u8]| {
            if let Ok(distance) = tof_ref.borrow_mut().read_range_mm() {
                let dst_be_bytes = distance.to_be_bytes();
                println!(
                    "read_distance, distance: {}, bytes: {:?}",
                    distance, dst_be_bytes
                );
                data[..4].copy_from_slice(&dst_be_bytes);
            } else {
                data[..4].copy_from_slice(&0u32.to_be_bytes());
            }
            2
        };

        gatt!([service {
            uuid: "997312e0-2354-11eb-9f10-fbc30a62cf38",
            characteristics: [characteristic {
                name: "seb_patch_distance",
                uuid: "997312e0-2354-11eb-9f10-fbc30a62cf38",
                read: rf_distance,
                write: wf,
                notify: true,
            },],
        },]);

        let mut rng = bleps::no_rng::NoRng;
        let mut srv = AttributeServer::new(&mut ble, &mut gatt_attributes, &mut rng);

        let mut notifier = || {
            // TODO how to check if notifications are enabled for the characteristic?
            // maybe pass something into the closure which just can query the characteristic value
            // probably passing in the attribute server won't work?

            async {
                Timer::after(Duration::from_millis(100)).await;
                let distance = tof_ref.borrow_mut().read_range_mm().unwrap();
                let dst_be_bytes = distance.to_be_bytes();
                let mut a2_reading = a2_reading.borrow_mut();
                let mut last_sensed_range = current_sensed_range.borrow_mut();
                let mut next_blink_loop = next_blink_loop.borrow_mut();
                let current_sense_range = SensedRange::from_distance(distance);
                if current_sense_range != *last_sensed_range {
                    *last_sensed_range = current_sense_range;
                    *next_blink_loop = current_sense_range.next_blink_loop();
                } else {
                    if *next_blink_loop == 0 {
                        *next_blink_loop = current_sense_range.next_blink_loop();
                    } else {
                        *next_blink_loop = *next_blink_loop - 1;
                    }
                }
                if *next_blink_loop == 0 {
                    let mut a2_pin = a2_pin.borrow_mut();
                    let mut adc1 = adc1.borrow_mut();
                    if let Ok(voltage) = adc1.read(&mut a2_pin) {
                        *a2_reading = voltage;
                    }
                    println!(
                        "send_notification, distance: {}, voltage: {} bytes: [{:?}]",
                        distance, a2_reading, &dst_be_bytes,
                    );
                    let color = current_sense_range.into_color();
                    led_ref
                        .borrow_mut()
                        .write(brightness(gamma(color.iter().cloned()), 10))
                        .unwrap();
                }
                Timer::after(Duration::from_millis(400)).await;
                if *next_blink_loop == 0 {
                    // Reset the LED to black
                    led_ref
                        .borrow_mut()
                        .write(brightness(gamma([BLACK].iter().cloned()), 10))
                        .unwrap();
                }
                let a2_bytes = a2_reading.to_be_bytes();
                let data: [u8; 4] = [dst_be_bytes[0], dst_be_bytes[1], a2_bytes[0], a2_bytes[1]];
                NotificationData::new(seb_patch_distance_handle, &data)
            }
        };

        srv.run(&mut notifier).await.unwrap();
    }
}

#![no_main]
#![no_std]

use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use embedded_hal::digital::v2::OutputPin;
use rtt_target::{rprintln, rtt_init_print};

// access to board peripherals:
use nrf52840_hal::{
    self as hal,
    gpio::Level,
    spim::{self, Spim},
    twim::{self, Error, Instance, Twim},
    Timer,
};

use sgp40::*;

use arrayvec::ArrayString;
use core::fmt::Write;

use embedded_graphics::{
    egtext,
    fonts::{Font12x16, Font24x32, Text},
    geometry::Point,
    pixelcolor::BinaryColor,
    prelude::*,
    style::TextStyle,
    text_style,
};

use epd_waveshare::{epd4in2::*, graphics::Display, prelude::*};

use crc_all::Crc;

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

#[derive(Default)]
pub struct SensorData {
    pub co2: f32,
    pub temperature: f32,
    pub humidity: f32,
}

pub const DEFAULT_ADDRESS: u8 = 0x61;

const CO2_POSITION: (i32, i32) = (220, 90);
const CO2_UNIT: &str = "ppm";

const TEMP_POSITION: (i32, i32) = (220, 130);
const TEMP_UNIT: &str = "°C";

const HUMIDITY_POSITION: (i32, i32) = (220, 170);
const HUMIDITY_UNIT: &str = "%";

const TVOC_POSITION: (i32, i32) = (220, 210);
const TVOC_UNIT: &str = "";

pub struct SCD30<T: Instance>(Twim<T>);

impl<T> SCD30<T>
where
    T: Instance,
{
    pub fn init(i2c2: Twim<T>) -> Self {
        SCD30(i2c2)
    }

    pub fn get_firmware_version(&mut self) -> Result<[u8; 2], Error> {
        let command: [u8; 2] = [0xd1, 0x00];
        let mut rd_buffer = [0u8; 2];

        self.0.write(DEFAULT_ADDRESS, &command)?;
        self.0.read(DEFAULT_ADDRESS, &mut rd_buffer)?;

        let major = u8::from_be(rd_buffer[0]);
        let minor = u8::from_be(rd_buffer[1]);

        Ok([major, minor])
    }

    pub fn start_continuous_measurement(&mut self, pressure: u16) -> Result<(), Error> {
        let mut command: [u8; 5] = [0x00, 0x10, 0x00, 0x00, 0x00];

        let argument_bytes = &pressure.to_be_bytes();

        command[2] = argument_bytes[0];
        command[3] = argument_bytes[1];
        let mut crc = Crc::<u8>::new(0x31, 8, 0xff, 0x00, false);

        crc.update(&pressure.to_be_bytes());
        command[4] = crc.finish();

        self.0.write(DEFAULT_ADDRESS, &command)?;

        Ok(())
    }

    pub fn read_measurement(&mut self) -> Result<SensorData, Error> {
        let command: [u8; 2] = [0x03, 0x00];
        let mut rd_buffer = [0u8; 18];

        self.0.write(DEFAULT_ADDRESS, &command)?;
        self.0.read(DEFAULT_ADDRESS, &mut rd_buffer)?;

        let data = SensorData {
            co2: f32::from_bits(u32::from_be_bytes([
                rd_buffer[0],
                rd_buffer[1],
                rd_buffer[3],
                rd_buffer[4],
            ])),
            temperature: f32::from_bits(u32::from_be_bytes([
                rd_buffer[6],
                rd_buffer[7],
                rd_buffer[9],
                rd_buffer[10],
            ])),
            humidity: f32::from_bits(u32::from_be_bytes([
                rd_buffer[12],
                rd_buffer[13],
                rd_buffer[15],
                rd_buffer[16],
            ])),
        };
        Ok(data)
    }
}

#[cortex_m_rt::entry]
fn main() -> ! {
    rtt_init_print!();
    let p = hal::pac::Peripherals::take().unwrap();
    let mut delay = Timer::new(p.TIMER1);
    let delay1 = Timer::new(p.TIMER2);

    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let port1 = hal::gpio::p1::Parts::new(p.P1);

    let _button = port0.p0_11.into_pullup_input();
    let mut led = port0.p0_13.into_push_pull_output(Level::Low);

    let din = port0.p0_12.into_push_pull_output(Level::Low).degrade();
    let clk = port0.p0_19.into_push_pull_output(Level::Low).degrade();
    let cs = port0.p0_21.into_push_pull_output(Level::Low);
    let dc = port0.p0_23.into_push_pull_output(Level::Low);
    let rst = port0.p0_07.into_push_pull_output(Level::Low);
    let busy = port1.p1_09.into_floating_input();

    let sda_scd30 = port1.p1_13.into_floating_input().degrade();
    let scl_scd30 = port1.p1_15.into_floating_input().degrade();

    let sda_sgp40 = port1.p1_11.into_floating_input().degrade();
    let scl_sgp40 = port1.p1_10.into_floating_input().degrade();

    let spi_pins = spim::Pins {
        sck: clk,
        miso: None,
        mosi: Some(din),
    };

    let mut spi = Spim::new(p.SPIM3, spi_pins, spim::Frequency::K500, spim::MODE_0, 0);

    // instantiate ePaper
    let mut epd4in2 = EPD4in2::new(&mut spi, cs, busy, dc, rst, &mut delay).unwrap();

    let display = Display4in2::default();

    let pins_scd30 = twim::Pins {
        scl: scl_scd30,
        sda: sda_scd30,
    };

    let pins_sgp40 = twim::Pins {
        scl: scl_sgp40,
        sda: sda_sgp40,
    };

    let i2c_scd30 = Twim::new(p.TWIM0, pins_scd30, twim::Frequency::K100);
    let i2c_sgp40 = Twim::new(p.TWIM1, pins_sgp40, twim::Frequency::K100);

    let mut sgp40 = Sgp40::new(i2c_sgp40, 0x59, delay1);

    for _ in 0..50 {
        let _ = sgp40.measure_voc_index();
    }

    let mut scd30 = SCD30::init(i2c_scd30);

    let firmware_version = scd30.get_firmware_version().unwrap_or_default();
    rprintln!(
        "SCD30 Firmware Version: {}.{}",
        firmware_version[0],
        firmware_version[1]
    );

    let pressure = 1020_u16;

    scd30
        .start_continuous_measurement(pressure)
        .unwrap_or_default();

    epd4in2.update_frame(&mut spi, &display.buffer()).unwrap();
    epd4in2
        .display_frame(&mut spi)
        .expect("display frame new graphics");

    let mut state: bool = false;

    loop {
        let scd30_result = scd30.read_measurement().unwrap_or_default();

        let co2 = scd30_result.co2;
        let temperature = scd30_result.temperature;
        let humidity = scd30_result.humidity;

        let tvoc = sgp40
            .measure_voc_index_with_rht(humidity as u16, temperature as i16)
            .unwrap_or_default();

        let display = Display4in2::default();

        let display = draw_numbers(co2, CO2_UNIT, CO2_POSITION, display);
        let display = draw_numbers(temperature, TEMP_UNIT, TEMP_POSITION, display);
        let display = draw_numbers(humidity, HUMIDITY_UNIT, HUMIDITY_POSITION, display);
        let display = draw_numbers(tvoc as f32, TVOC_UNIT, TVOC_POSITION, display);

        let display = draw_text(display);

        epd4in2.update_frame(&mut spi, &display.buffer()).unwrap();
        epd4in2
            .display_frame(&mut spi)
            .expect("display frame new graphics");

        if state {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
        state = !state;

        delay.delay_ms(10000u32);
    }
}

pub fn draw_text(mut display: Display4in2) -> Display4in2 {
    Text::new("Air Quality", Point::new(20, 30))
        .into_styled(TextStyle::new(Font24x32, BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    Text::new("Carbon Dioxide:", Point::new(20, 90))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    Text::new("Temperature:", Point::new(20, 130))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    Text::new("Humidity:", Point::new(20, 170))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    Text::new("TVOC Score:", Point::new(20, 210))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display)
        .unwrap();

    display
}

pub fn draw_numbers(
    value: f32,
    unit: &str,
    position: (i32, i32),
    mut display: Display4in2,
) -> Display4in2 {
    let mut buf = ArrayString::<12>::new();

    write!(&mut buf, "{:.2} {}", value, unit).expect("Failed to write to buffer");

    egtext!(
        text = &buf,
        top_left = position,
        style = text_style!(font = Font12x16, text_color = BinaryColor::On,)
    )
    .draw(&mut display)
    .unwrap();

    display
}

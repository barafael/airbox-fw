#![no_main]
#![no_std]

use embedded_hal::digital::v2::InputPin;
use embedded_hal::digital::v2::OutputPin;
use rtt_target::{rprintln, rtt_init_print};

// access to board peripherals:
use nrf52840_hal::{
    self as hal,
    gpio::{p0::Parts as P0Parts, p1::Parts as P1Parts, Level},
    prelude::*,
    spim::{self, Spim},
    Timer,
    Temp,
    twim::{self, Twim, Error, Instance},
};

//use sgp40::*;

use arrayvec::ArrayString;
use core::fmt::Write;

use embedded_graphics::{
    fonts::{Font12x16, Font24x32, Text},
    egtext,
    text_style,
    geometry::Point,
    pixelcolor::BinaryColor,
    prelude::*,
    primitives::{ Circle, Triangle },
    style::PrimitiveStyle,
    style::TextStyle,
};

use epd_waveshare::{
    epd4in2::*,
    graphics::Display,
    prelude::*,
};

use crc_all::Crc;

#[panic_handler] // panicking behavior
fn panic(_: &core::panic::PanicInfo) -> ! {
    loop {
        cortex_m::asm::bkpt();
    }
}

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

pub struct SCD30<T: Instance>(Twim<T>);

impl<T> SCD30<T> where T: Instance {

    pub fn init(i2c2: Twim<T>) -> Self {
        SCD30(i2c2)
    }

        pub fn get_firmware_version(&mut self) -> Result<[u8; 2], Error> {
        let command:[u8; 2] = [0xd1, 0x00];
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
    let port0 = hal::gpio::p0::Parts::new(p.P0);
    let button = port0.p0_11.into_pullup_input();
    let mut led = port0.p0_13.into_push_pull_output(Level::Low);
    let mut delay = Timer::new(p.TIMER1);

    let pins_1 = hal::gpio::p1::Parts::new(p.P1);
    let din = pins_1.p1_01.into_push_pull_output(Level::Low).degrade();
    let clk = pins_1.p1_02.into_push_pull_output(Level::Low).degrade();
    let cs = pins_1.p1_03.into_push_pull_output(Level::Low);
    let dc = pins_1.p1_04.into_push_pull_output(Level::Low);
    let rst = port0.p0_29.into_push_pull_output(Level::Low);
    let busy = pins_1.p1_06.into_floating_input();

    let scl = port0.p0_30.into_floating_input().degrade();
    let sda = port0.p0_31.into_floating_input().degrade();

    let scl1 = pins_1.p1_13.into_floating_input().degrade();
    let sda1 = pins_1.p1_15.into_floating_input().degrade();

    let pins = twim::Pins { scl, sda };
    let pins1 = twim::Pins { scl: scl1, sda: sda1 };

    let i2c = Twim::new(p.TWIM0, pins, twim::Frequency::K100);
    let i2c1 = Twim::new(p.TWIM1, pins1, twim::Frequency::K100);

    //let sgp40 = Sgp40::new(i2c1, 0x31, delay.clone());

    let mut sensor = SCD30::init(i2c);

    let firmware_version = sensor.get_firmware_version().unwrap();
    rprintln!("Firmware Version: {}.{}", firmware_version[0], firmware_version[1]);

    let pressure = 1020_u16;

    sensor.start_continuous_measurement(pressure).unwrap();

    let spi_pins = spim::Pins {
        sck: clk,
        miso: None,
        mosi: Some(din),
    };

    let mut spi = Spim::new(p.SPIM3, spi_pins, spim::Frequency::K500, spim::MODE_0, 0);

    // instantiate ePaper
    let mut epd4in2 = EPD4in2::new(&mut spi, cs, busy, dc, rst, &mut delay).unwrap();

    let mut display = Display4in2::default();

    epd4in2.update_frame(&mut spi, &display.buffer()).unwrap();
    epd4in2.display_frame(&mut spi)
        .expect("display frame new graphics");

    loop {

    let result = sensor.read_measurement().unwrap();

    let co2 = result.co2;
    let temp = result.temperature;
    let humidity = result.humidity;

        let display = Display4in2::default();

        let display = draw_numbers(co2, CO2_UNIT, CO2_POSITION, display);
        let display = draw_numbers(temp, TEMP_UNIT, TEMP_POSITION, display);
        let display = draw_numbers(humidity, HUMIDITY_UNIT, HUMIDITY_POSITION, display);

        let display = draw_text(display);

        epd4in2.update_frame(&mut spi, &display.buffer()).unwrap();
        epd4in2.display_frame(&mut spi).expect("display frame new graphics");

        if button.is_high().unwrap() {
            led.set_high().unwrap();
        } else {
            led.set_low().unwrap();
        }
        delay.delay(1000000);
    }
}

pub fn draw_text (mut display: Display4in2 ) -> Display4in2 {
    Text::new("Air Quality", Point::new(20, 30))
        .into_styled(TextStyle::new(Font24x32, BinaryColor::On))
        .draw(&mut display).unwrap();

    Text::new("Carbon Dioxide:", Point::new(20, 90))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display).unwrap();

    Text::new("Temperature:", Point::new(20, 130))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display).unwrap();

    Text::new("Humidity:", Point::new(20, 170))
        .into_styled(TextStyle::new(Font12x16, BinaryColor::On))
        .draw(&mut display).unwrap();

    display
}

pub fn draw_numbers (value: f32, unit: &str, position: (i32, i32), mut display: Display4in2 ) -> Display4in2 {

let mut buf = ArrayString::<[_; 12]>::new();

write!(&mut buf, "{:.2} {}", value, unit).expect("Failed to write to buffer");

egtext!(
    text = &buf,
    top_left = position,
    style = text_style!(
        font = Font12x16,
        text_color = BinaryColor::On,
    )
)
.draw(&mut display).unwrap();

    display

}
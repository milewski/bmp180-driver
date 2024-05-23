use std::error::Error;
use esp_idf_svc::hal::delay::FreeRtos;

use esp_idf_svc::hal::i2c::config::Config;
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::hal::prelude::{FromValueType, Peripherals};

use bmp180_driver::{BMP180, Common, Sampling};

fn main() -> Result<(), Box<dyn Error>> {
    esp_idf_svc::sys::link_patches();

    let peripherals = Peripherals::take()?;

    let sda = peripherals.pins.gpio6;
    let scl = peripherals.pins.gpio5;

    let config = Config::default().baudrate(100.kHz().into());
    let mut i2c = I2cDriver::new(
        peripherals.i2c0, sda, scl, &config,
    )?;

    let mut sensor = BMP180::new(i2c, FreeRtos);

    // sensor.test_connection()?;
    // sensor.reset()?;
    //
    let mut sensor = sensor.initialize()?;

    loop {
        println!("{:?}", sensor.get_pressure(Sampling::HighResolution)?);
    }

    // let mut buffer = [0;8];
    //
    // i2c.write(0b111_0_111, &[0x2E], BLOCK)?;
    // i2c.write(0b111_0_111, &[0x2E], BLOCK)?;
    // FreeRtos::delay_ms(2);
    // i2c.write_read(0b111_0_111, &[0xAA], &mut buffer, BLOCK)?;
    //
    // print!("{:?}", buffer);

    // Bind the log crate to the ESP Logging facilities
    // esp_idf_svc::log::EspLogger::initialize_default();

    // log::info!("Hello, world!");

    Ok(())
}

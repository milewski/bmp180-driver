# BMP180 Driver

This library provides an interface for interacting with the BMP180 pressure sensor.

## Installation

You can install the package via Cargo:

```shell
cargo add bmp180-driver
```

Then, you can use the library in your code as follows:

```rust
use std::error::Error;
use bmp180_driver::{Common, Resolution, BMP180};

use esp_idf_svc::hal::delay::FreeRtos;
use esp_idf_svc::hal::i2c::config::Config;
use esp_idf_svc::hal::i2c::I2cDriver;
use esp_idf_svc::hal::prelude::Peripherals;

fn main() -> Result<(), Box<dyn Error>> {
    // Initialize peripherals
    let peripherals = Peripherals::take()?;

    // Define SDA and SCL pins
    let sda = peripherals.pins.gpio6;
    let scl = peripherals.pins.gpio5;

    // Initialize I2C driver
    let i2c = I2cDriver::new(peripherals.i2c0, sda, scl, &Config::default())?;

    // Create BMP180 sensor instance
    let mut sensor = BMP180::new(i2c, FreeRtos);

    // Check connection to the sensor
    sensor.check_connection()?;

    // Initialize the sensor
    let mut sensor = sensor.initialize()?;

    // Continuously read and print sensor data
    loop {
        let (temperature, pressure, altitude) = sensor.read_all(Resolution::UltraHighResolution)?;

        println!("Temperature: {} °C", temperature);
        println!("Pressure: {} Pa", pressure);
        println!("Altitude: {} meters", altitude);
    }
}
```

## References

- [BMP180 Datasheet](https://cdn-shop.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf)

## License

The MIT License (MIT). Please see [License File](./LICENSE) for more information.
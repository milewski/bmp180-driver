#![doc = include_str!("../README.md")]
use coefficients::Coefficients;
use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use error::CustomError;

mod coefficients;
pub mod error;

/// Represents the resolution settings of the BMP180 sensor.
#[derive(Copy, Clone)]
pub enum Resolution {
    UltraLowPower = 0,
    Standard = 1,
    HighResolution = 2,
    UltraHighResolution = 3,
}

enum Register {
    ChipId = 0xD0,
    SoftReset = 0xE0,
    MeasurementControl = 0xF4,
    DataOutputStart = 0xF6,
    CalibrationOutStart = 0xAA,
}

enum Command {
    Temperature = 0x2E,
    Pressure = 0x34,
    SoftReset = 0xB6,
}

/// A trait representing common functionalities shared between [BMP180] and [InitializedBMP180].
pub trait Common<T: I2c> {
    /// Reads the chip identifier.
    ///
    /// The returned value is hardcoded by the manufacturer and should be equivalent to `0x55`.
    fn read_chip_id(&mut self) -> Result<u8, CustomError<T::Error>>;

    /// Tests the connection to the device.
    ///
    /// It checks whether the device responds with the expected identifier `0x55`.
    fn check_connection(&mut self) -> Result<(), CustomError<T::Error>> {
        match self.read_chip_id() {
            Ok(0x55) => Ok(()),
            Err(error) => Err(error),
            _ => Err(CustomError::InvalidDeviceIdentifier),
        }
    }

    /// Resets the device.
    fn reset(&mut self) -> Result<(), CustomError<T::Error>>;
}

/// Represents an uninitialized version of the sensor.
pub struct BMP180<T, D> {
    address: u8,
    i2c: T,
    delay: D,
}

/// Represents an initialized version of the sensor.
pub struct InitializedBMP180<T, D> {
    inner: BMP180<T, D>,
    data: Coefficients,
}

impl<T: I2c, D: DelayNs> Common<T> for InitializedBMP180<T, D> {
    /// Reads the chip identifier.
    ///
    /// The returned value is hardcoded by the manufacturer and should be equivalent to `0x55`.
    fn read_chip_id(&mut self) -> Result<u8, CustomError<T::Error>> {
        self.inner.write_and_read_single_byte(&[Register::ChipId as u8])
    }

    /// Resets the device.
    fn reset(&mut self) -> Result<(), CustomError<T::Error>> {
        self.inner.write(&[Register::SoftReset as u8, Command::SoftReset as u8])?;
        self.inner.delay.delay_ms(5);

        Ok(())
    }
}

impl<T: I2c, D: DelayNs> Common<T> for BMP180<T, D> {
    /// Reads the chip identifier.
    ///
    /// The returned value is hardcoded by the manufacturer and should be equivalent to `0x55`.
    fn read_chip_id(&mut self) -> Result<u8, CustomError<T::Error>> {
        self.write_and_read_single_byte(&[Register::ChipId as u8])
    }

    /// Resets the device.
    fn reset(&mut self) -> Result<(), CustomError<T::Error>> {
        self.write(&[Register::SoftReset as u8, Command::SoftReset as u8])?;
        self.delay.delay_ms(5);

        Ok(())
    }
}

impl<T: I2c, D: DelayNs> InitializedBMP180<T, D> {
    /// Reads uncompensated temperature.
    fn read_uncompensated_temperature(&mut self) -> Result<u16, CustomError<T::Error>> {
        self.inner.write(&[Register::MeasurementControl as u8, Command::Temperature as u8])?;
        self.inner.delay.delay_ms(5);

        Ok(u16::from_be_bytes(self.inner.write_and_read_exact_bytes(&[Register::DataOutputStart as u8])?))
    }

    /// Reads uncompensated pressure.
    fn read_uncompensated_pressure(&mut self, resolution: Resolution) -> Result<i32, CustomError<T::Error>> {
        let oss = resolution as u8;
        let mut buffer = [0u8; 4];

        self.inner.write(&[Register::MeasurementControl as u8, Command::Pressure as u8 + (oss << 6)])?;

        self.inner.delay.delay_ms(match resolution {
            Resolution::UltraLowPower => 5,
            Resolution::Standard => 8,
            Resolution::HighResolution => 14,
            Resolution::UltraHighResolution => 26,
        });

        self.inner.write_and_read_range(&[Register::DataOutputStart as u8], &mut buffer[1..4])?;

        Ok(i32::from_be_bytes(buffer) >> (8 - oss))
    }

    /// Measures temperature.
    pub fn temperature(&mut self) -> Result<f32, CustomError<T::Error>> {
        let uncompensated_temperature = self.read_uncompensated_temperature()?;
        let (temperature, _) = self.data.calculate_temperature(uncompensated_temperature as i32);

        Ok(temperature)
    }

    /// Measures pressure.
    pub fn pressure(&mut self, resolution: Resolution) -> Result<i32, CustomError<T::Error>> {
        let uncompensated_temperature = self.read_uncompensated_temperature()?;
        let (_, b5) = self.data.calculate_temperature(uncompensated_temperature as i32);

        let uncompensated_pressure = self.read_uncompensated_pressure(resolution)?;
        let pressure = self.data.calculate_pressure(b5, uncompensated_pressure, resolution);

        Ok(pressure)
    }

    /// Calculates altitude.
    pub fn altitude(&mut self, resolution: Resolution) -> Result<f32, CustomError<T::Error>> {
        let pressure = self.pressure(resolution)?;

        Ok(self.data.calculate_altitude(pressure))
    }

    /// Reads all data including temperature, pressure, and altitude.
    pub fn read_all(&mut self, resolution: Resolution) -> Result<(f32, i32, f32), CustomError<T::Error>> {
        let uncompensated_temperature = self.read_uncompensated_temperature()?;
        let uncompensated_pressure = self.read_uncompensated_pressure(resolution)?;

        let (temperature, b5) = self.data.calculate_temperature(uncompensated_temperature as i32);

        let pressure = self.data.calculate_pressure(b5, uncompensated_pressure, resolution);
        let altitude = self.data.calculate_altitude(pressure);

        Ok((temperature, pressure, altitude))
    }
}

impl<T: I2c, D: DelayNs> BMP180<T, D> {
    /// Creates a new instance of BMP180 sensor.
    pub fn new(i2c: T, delay: D) -> Self {
        Self {
            address: 0b111_0111,
            delay,
            i2c,
        }
    }

    /// Initializes the BMP180 sensor.
    pub fn initialize(mut self) -> Result<InitializedBMP180<T, D>, CustomError<T::Error>> {
        let mut buffer = [0u8; 22];

        self.i2c.write_read(self.address, &[Register::CalibrationOutStart as u8], &mut buffer)?;

        let calibration = Coefficients::new(buffer)?;

        Ok(InitializedBMP180 {
            inner: self,
            data: calibration,
        })
    }

    /// Sets the I2C address of the sensor.
    pub fn set_address(&mut self, address: u8) {
        self.address = address;
    }

    /// Writes data to the sensor and reads back exactly the specified number of bytes.
    pub fn write_and_read_exact_bytes<const BYTES: usize>(&mut self, command: &[u8]) -> Result<[u8; BYTES], CustomError<T::Error>> {
        let mut buffer = [0u8; BYTES];

        self.i2c.write_read(self.address, command, &mut buffer)?;

        Ok(buffer)
    }

    /// Writes data to the sensor and reads back a range of bytes.
    pub fn write_and_read_range(&mut self, command: &[u8], buffer: &mut [u8]) -> Result<(), CustomError<T::Error>> {
        self.i2c.write_read(self.address, command, buffer)?;

        Ok(())
    }

    /// Write and read a single byte back.
    pub fn write_and_read_single_byte(&mut self, command: &[u8]) -> Result<u8, CustomError<T::Error>> {
        let mut buffer = [0u8; 1];

        self.i2c.write_read(self.address, command, &mut buffer)?;

        Ok(buffer[0])
    }

    /// Writes data to the sensor.
    pub fn write(&mut self, command: &[u8]) -> Result<(), CustomError<T::Error>> {
        self.i2c.write(self.address, command)?;

        Ok(())
    }
}

use embedded_hal::delay::DelayNs;
use embedded_hal::i2c::I2c;
use coefficients::Coefficients;

mod coefficients;

#[derive(Copy, Clone)]
pub enum Sampling {
    UltraLowPower = 0,
    Standard = 1,
    HighResolution = 2,
    UltraHigResolution = 3,
}

enum CustomError<T> {
    InvalidCalibrationData,
    InvalidDeviceIdentifier,
    I2c(T),
}

impl<T> From<T> for CustomError<T> {
    fn from(error: T) -> Self {
        CustomError::I2c(error)
    }
}

enum RegisterMap {
    ChipId = 0xD0,
    SoftReset = 0xE0,
    MeasurementControl = 0xF4,

    DataOutputStart = 0xF6,
    CalibrationOut = 0xAA,
}

enum Command {
    TemperatureCommand = 0x2E,
    PressureCommand = 0x34,
    SoftReset = 0xB6,
}

pub trait Common<T: I2c> {
    fn id(&mut self) -> Result<u8, CustomError<T::Error>>;

    fn test_connection(&mut self) -> Result<(), CustomError<T::Error>> {
        match self.id() {
            Ok(0x55) => Ok(()),
            Err(error) => Err(error),
            _ => Err(CustomError::InvalidDeviceIdentifier),
        }
    }

    fn reset(&mut self) -> Result<(), CustomError<T::Error>>;
}

pub struct BMP180<T, D> {
    address: u8,
    i2c: T,
    delay: D,
}

pub struct InitializedBMP180<T, D> {
    inner: BMP180<T, D>,
    data: Coefficients,
}

impl<T: I2c, D: DelayNs> Common<T> for InitializedBMP180<T, D> {
    fn id(&mut self) -> Result<u8, CustomError<T::Error>> {
        self.inner.write_single_byte(&[RegisterMap::ChipId as u8])
    }

    fn reset(&mut self) -> Result<(), CustomError<T::Error>> {
        self.inner.write(&[RegisterMap::SoftReset as u8, Command::SoftReset as u8])?;
        self.inner.delay.delay_ms(5);

        Ok(())
    }
}

impl<T: I2c, D: DelayNs> Common<T> for BMP180<T, D> {
    fn id(&mut self) -> Result<u8, CustomError<T::Error>> {
        self.write_single_byte(&[RegisterMap::ChipId as u8])
    }

    fn reset(&mut self) -> Result<(), CustomError<T::Error>> {
        self.write(&[RegisterMap::SoftReset as u8, Command::SoftReset as u8])?;
        self.delay.delay_ms(5);

        Ok(())
    }
}

impl<T: I2c, D: DelayNs> InitializedBMP180<T, D> {
    fn read_ut(&mut self) -> Result<u16, CustomError<T::Error>> {
        self.inner.write(&[RegisterMap::MeasurementControl as u8, Command::TemperatureCommand as u8])?;
        self.inner.delay.delay_ms(5);

        Ok(u16::from_be_bytes(self.inner.write_and_read_exactly(&[RegisterMap::DataOutputStart as u8])?))
    }

    fn read_up(&mut self) -> Result<i32, CustomError<T::Error>> {
        let oss = self.data.sampling as u8;
        let mut buffer = [0u8; 4];

        self.inner.write(&[RegisterMap::MeasurementControl as u8, Command::PressureCommand as u8 + (oss << 6)])?;

        self.inner.delay.delay_ms(match self.data.sampling {
            Sampling::UltraLowPower => 5,
            Sampling::Standard => 8,
            Sampling::HighResolution => 14,
            Sampling::UltraHigResolution => 26,
        });

        self.inner.write_range(&[RegisterMap::DataOutputStart as u8], &mut buffer[1..4])?;

        Ok(i32::from_be_bytes(buffer) >> (8 - oss))
    }

    pub fn temperature(&mut self) -> Result<f32, CustomError<T::Error>> {
        let ut = self.read_ut()?;
        let (temperature, _) = self.data.calculate_temperature(ut as i32);

        Ok(temperature)
    }

    pub fn read_all_data(&mut self) -> Result<(f32, i32, f32), CustomError<T::Error>> {
        let ut = self.read_ut()?;
        let up = self.read_up()?;

        let (temperature, b5) = self.data.calculate_temperature(ut as i32);

        let pressure = self.data.calculate_pressure(b5, up);
        let altitude = self.data.calculate_altitude(pressure);

        Ok((temperature, pressure, altitude))
    }

    pub fn get_pressure(&mut self) -> Result<i32, CustomError<T::Error>> {
        let ut = self.read_ut()?;
        let (_, b5) = self.data.calculate_temperature(ut as i32);
        let up = self.read_up()?;

        let pressure = self.data.calculate_pressure(b5, up);

        Ok(pressure)
    }

    pub fn altitude(&mut self) -> Result<f32, CustomError<T::Error>> {
        let pressure = self.get_pressure()?;
        let sea_level_pressure = 101_325f32;
        let p_sea_level_ratio: f32 = pressure as f32 / sea_level_pressure;
        let altitude = 44_330.0 * (1.0 - p_sea_level_ratio.powf(1.0 / 5.255));

        Ok(altitude)
    }

    pub fn set_address(&mut self, address: u8) {
        self.inner.address = address;
    }
}

impl<T: I2c, D: DelayNs> BMP180<T, D> {
    pub fn new(i2c: T, delay: D) -> Self {
        Self {
            address: 0b111_0_111,
            delay,
            i2c,
        }
    }

    pub fn initialize(mut self, sampling_rate: Sampling) -> Result<InitializedBMP180<T, D>, CustomError<T::Error>> {
        let mut buffer = [0u8; 22];

        self.i2c.write_read(self.address, &[RegisterMap::CalibrationOut as u8], &mut buffer)?;

        let calibration = Coefficients::new(buffer, sampling_rate)?;

        Ok(InitializedBMP180 { inner: self, data: calibration })
    }

    pub fn write_and_read_exactly<const BYTES: usize>(&mut self, command: &[u8]) -> Result<[u8; BYTES], CustomError<T::Error>> {
        let mut buffer = [0u8; BYTES];

        self.i2c.write_read(self.address, command, &mut buffer)?;

        Ok(buffer)
    }

    pub fn write_range(&mut self, command: &[u8], buffer: &mut [u8]) -> Result<(), CustomError<T::Error>> {
        self.i2c.write_read(self.address, command, buffer)?;

        Ok(())
    }

    pub fn write_single_byte(&mut self, command: &[u8]) -> Result<u8, CustomError<T::Error>> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, command, &mut buffer)?;

        Ok(buffer[0])
    }

    pub fn write(&mut self, command: &[u8]) -> Result<(), CustomError<T::Error>> {
        self.i2c.write(self.address, command)?;

        Ok(())
    }
}

use std::i16;

use embedded_hal::i2c::{ErrorType, I2c};
use esp_idf_svc::hal::delay::FreeRtos;

#[derive(Copy, Clone)]
pub enum Sampling {
    UltraLowPower = 0,
    Standard = 1,
    HighResolution = 2,
    UltraHigResolution = 3,
}

struct CalibrationCoefficients {
    oss: Sampling,
    ac1: i16,
    ac2: i16,
    ac3: i16,
    ac4: u16,
    ac5: u16,
    ac6: u16,
    b1: i16,
    b2: i16,
    mb: i16,
    mc: i16,
    md: i16,
}

impl CalibrationCoefficients {
    pub fn new(data: [u8; 22], oss: Sampling) -> Self {
        let ac1 = i16::from_be_bytes(data[0..2].try_into().unwrap());
        let ac2 = i16::from_be_bytes(data[2..4].try_into().unwrap());
        let ac3 = i16::from_be_bytes(data[4..6].try_into().unwrap());
        let ac4 = u16::from_be_bytes(data[6..8].try_into().unwrap());
        let ac5 = u16::from_be_bytes(data[8..10].try_into().unwrap());
        let ac6 = u16::from_be_bytes(data[10..12].try_into().unwrap());
        let b1 = i16::from_be_bytes(data[12..14].try_into().unwrap());
        let b2 = i16::from_be_bytes(data[14..16].try_into().unwrap());
        let mb = i16::from_be_bytes(data[16..18].try_into().unwrap());
        let mc = i16::from_be_bytes(data[18..20].try_into().unwrap());
        let md = i16::from_be_bytes(data[20..22].try_into().unwrap());

        Self {
            oss,
            ac1,
            ac2,
            ac3,
            ac4,
            ac5,
            ac6,
            b1,
            b2,
            mb,
            mc,
            md,
        }
    }

    fn calculate_temperature(&self, ut: i32) -> (f32, i32) {
        let x1 = (ut - self.ac6 as i32) * self.ac5 as i32 >> 15;
        let x2 = ((self.mc as i32) << 11) / (x1 + self.md as i32);
        let b5 = x1 + x2;
        let temperature = ((b5 + 8) >> 4) as f32 / 10.0;

        (temperature, b5)
    }

    fn calculate_pressure(&self, b5: i32, up: i32) -> i32 {
        let oss = self.oss as u8;
        let b6: i32 = b5 - 4000i32;

        let t = b6.pow(2) >> 12;
        let mut x1: i32 = (self.b2 as i32 * t) >> 11;
        let mut x2: i32 = (self.ac2 as i32 * b6) >> 11;
        let x3: u32 = (x1 + x2) as u32;
        let b3: i32 = (((self.ac1 as i32 * 4 + (x3 as i32)) << oss) + 2) / 4;
        x1 = (self.ac3 as i32 * b6) >> 13;
        x2 = (self.b1 as i32 * t) >> 16;
        let x3: i32 = (x1 + x2 + 2) >> 2;

        let b4: u32 = (self.ac4 as u32 * (x3 + 32768) as u32) >> 15;
        let b7: u32 = (up - b3) as u32 * (50000 >> oss);
        let p = if b7 < 0x80000000 {
            (b7 << 1) / b4
        } else {
            (b7 / b4) << 1
        } as i32;

        x1 = (p >> 8).pow(2);
        x1 = (x1 * 3038) >> 16;
        x2 = (-7357 * (p)) >> 16;

        (p) + ((x1 + x2 + 3791) >> 4)
    }

    fn calculate_altitude(&self, pressure: i32) -> f32 {
        let sea_level_pressure = 101_325f32;
        let p_sea_level_ratio: f32 = pressure as f32 / sea_level_pressure;
        let altitude = 44_330.0 * (1.0 - p_sea_level_ratio.powf(1.0 / 5.255));

        altitude
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

impl Into<u8> for RegisterMap {
    fn into(self) -> u8 {
        self as u8
    }
}

impl Into<u8> for Command {
    fn into(self) -> u8 {
        self as u8
    }
}

pub trait CommonActions<T: I2c> {
    fn id(&mut self) -> Result<u8, T::Error>;

    fn test_connection(&mut self) -> Result<(), &'static str> {
        match self.id() {
            Ok(0x55) => Ok(()),
            Err(Error) => Err("I2C error"),
            _ => Err("Unrecognized device identifier"),
        }
    }

    fn reset(&mut self) -> Result<(), T::Error>;
}

pub struct BMP180<T> {
    address: u8,
    i2c: T,
}

pub struct InitializedBMP180<T> {
    inner: BMP180<T>,
    data: CalibrationCoefficients,
}

impl<T: I2c> CommonActions<T> for InitializedBMP180<T> {
    fn id(&mut self) -> Result<u8, T::Error> {
        self.inner.write_single_byte(&[RegisterMap::ChipId as u8])
    }

    fn reset(&mut self) -> Result<(), T::Error> {
        self.inner.write(&[RegisterMap::SoftReset as u8, Command::SoftReset as u8])
    }
}

impl<T: I2c> CommonActions<T> for BMP180<T> {
    fn id(&mut self) -> Result<u8, T::Error> {
        self.write_single_byte(&[RegisterMap::ChipId as u8])
    }

    fn reset(&mut self) -> Result<(), T::Error> {
        self.write(&[RegisterMap::SoftReset as u8, Command::SoftReset as u8])?;
        FreeRtos::delay_ms(5);
        Ok(())
    }
}

struct Payload {}

impl<T: I2c> InitializedBMP180<T> {
    fn read_ut(&mut self) -> Result<u16, T::Error> {
        self.inner.write(&[RegisterMap::MeasurementControl as u8, Command::TemperatureCommand as u8])?;

        FreeRtos::delay_ms(5);

        Ok(u16::from_be_bytes(self.inner.write_and_read_exactly(&[RegisterMap::DataOutputStart as u8])?))
    }

    fn read_up(&mut self) -> Result<i32, T::Error> {
        let oss = self.data.oss as u8;
        let mut buffer = [0u8; 4];

        self.inner.write(&[RegisterMap::MeasurementControl as u8, Command::PressureCommand as u8 + (oss << 6)])?;

        FreeRtos::delay_ms(match self.data.oss {
            Sampling::UltraLowPower => 5,
            Sampling::Standard => 8,
            Sampling::HighResolution => 14,
            Sampling::UltraHigResolution => 26,
        });

        self.inner.write_range(&[RegisterMap::DataOutputStart as u8], &mut buffer[1..4])?;

        Ok(i32::from_be_bytes(buffer) >> (8 - oss))
    }

    pub fn temperature(&mut self) -> Result<f32, T::Error> {
        let ut = self.read_ut()?;
        let (temperature, _) = self.data.calculate_temperature(ut as i32);

        Ok(temperature)
    }

    pub fn read_all_data(&mut self) -> Result<(f32, i32, f32), T::Error> {
        let ut = self.read_ut()?;
        let up = self.read_up()?;

        let (temperature, b5) = self.data.calculate_temperature(ut as i32);

        let pressure = self.data.calculate_pressure(b5, up);
        let altitude = self.data.calculate_altitude(pressure);

        Ok((temperature, pressure, altitude))
    }

    pub fn get_pressure(&mut self) -> Result<i32, T::Error> {
        let ut = self.read_ut()?;
        let (_, b5) = self.data.calculate_temperature(ut as i32);
        let up = self.read_up()?;

        let pressure = self.data.calculate_pressure(b5, up);

        Ok(pressure)
    }

    pub fn altitude(&mut self) -> Result<f32, T::Error> {
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

impl<T: I2c> BMP180<T> {
    pub fn new(i2c: T) -> Self {
        Self {
            address: 0b111_0_111,
            i2c,
        }
    }

    pub fn initialize(mut self, sampling_rate: Sampling) -> Result<InitializedBMP180<T>, T::Error> {
        let mut buffer = [0u8; 22];

        self.i2c.write_read(self.address, &[RegisterMap::CalibrationOut as u8], &mut buffer)?;

        let calibration = CalibrationCoefficients::new(buffer, sampling_rate);

        Ok(InitializedBMP180 { inner: self, data: calibration })
    }

    pub fn write_and_read_exactly<const BYTES: usize>(&mut self, command: &[u8]) -> Result<[u8; BYTES], T::Error> {
        let mut buffer = [0u8; BYTES];

        self.i2c.write_read(self.address, command, &mut buffer)?;

        Ok(buffer)
    }

    pub fn write_range(&mut self, command: &[u8], buffer: &mut [u8]) -> Result<(), T::Error> {
        self.i2c.write_read(self.address, command, buffer)
    }

    pub fn write_single_byte(&mut self, command: &[u8]) -> Result<u8, T::Error> {
        let mut buffer = [0u8; 1];
        self.i2c.write_read(self.address, command, &mut buffer)?;

        Ok(buffer[0])
    }


    pub fn write(&mut self, command: &[u8]) -> Result<(), T::Error> {
        self.i2c.write(self.address, command)
    }

    // pub fn temperature(&mut self) {
    //     let mut ac1_response = [0u8; 22];
    //
    //     self.driver.write_read(self.address, &[0xAA], &mut ac1_response).unwrap();
    //
    //     let oss = Oss::UltraHighRes;
    //     let calibration = Calibration::new(ac1_response, oss);
    //
    //     loop {
    //         self.driver.write(self.address, &[RegisterMap::Control.into(), 0x2E]).unwrap();
    //         FreeRtos::delay_ms(5);
    //
    //         let mut temperature = [0u8; 2];
    //         self.driver.write_read(self.address, &[0xF6], &mut temperature).unwrap();
    //         let ut = u16::from_be_bytes(temperature);
    //         let (temperature, b5) = calibration.calculate_temperature(ut as i32);
    //
    //         self.driver.write(self.address, &[RegisterMap::Control.into(), 0x34 + ((oss as u8) << 6)]).unwrap();
    //         FreeRtos::delay_ms(26);
    //
    //         let mut pressure = [0u8; 4];
    //
    //         // self.driver.write_read(self.address, &[0xF6], &mut pressure[1..4]).unwrap();
    //
    //         self.driver.write_read(self.address, &[0xF6], &mut pressure[1..2]).unwrap();
    //         self.driver.write_read(self.address, &[0xF7], &mut pressure[2..3]).unwrap();
    //         self.driver.write_read(self.address, &[0xF8], &mut pressure[3..4]).unwrap();
    //
    //         let up = i32::from_be_bytes(pressure) >> (8 - (oss as u8));
    //         let pressure = calibration.calculate_pressure(b5, up);
    //         let sea_level_pressure = 101_325f32;
    //         //
    //         // let p_sea_level_ratio: f32 = pressure as f32 / sea_level_pressure as f32;
    //         // let altitude = 44_330.0 * (1.0 - libm::powf(p_sea_level_ratio, 1.0 / 5.255));
    //
    //         let p_sea_level_ratio: f32 = pressure as f32 / sea_level_pressure as f32;
    //         let altitude = 44_330.0 * (1.0 - p_sea_level_ratio.powf(1.0 / 5.255));
    //
    //         println!("Temperature: {:?}", temperature);
    //         println!("Pressure: {:?}", pressure);
    //         println!("Altitude: {:?}", altitude);
    //
    //         // FreeRtos::delay_ms(500);
    //     }
    // }

    // pub fn get_altitude(&mut self) {
    //     let pressure = self.get_pressure()?;
    //     let p_sea_level_ratio: f32 = pressure as f32 / self.sea_level_pressure as f32;
    //     let altitude = 44_330.0 * (1.0 - libm::powf(p_sea_level_ratio, 1.0 / 5.255));
    //
    //     altitude
    // }
}
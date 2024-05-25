use crate::error::CustomError;
use crate::Resolution;

pub struct Coefficients {
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

impl Coefficients {
    pub fn new<T>(data: [u8; 22]) -> Result<Self, CustomError<T>> {
        let instance = Self {
            ac1: i16::from_be_bytes(data[00..02].try_into().unwrap()),
            ac2: i16::from_be_bytes(data[02..04].try_into().unwrap()),
            ac3: i16::from_be_bytes(data[04..06].try_into().unwrap()),

            ac4: u16::from_be_bytes(data[06..08].try_into().unwrap()),
            ac5: u16::from_be_bytes(data[08..10].try_into().unwrap()),
            ac6: u16::from_be_bytes(data[10..12].try_into().unwrap()),

            b1: i16::from_be_bytes(data[12..14].try_into().unwrap()),
            b2: i16::from_be_bytes(data[14..16].try_into().unwrap()),

            mb: i16::from_be_bytes(data[16..18].try_into().unwrap()),
            mc: i16::from_be_bytes(data[18..20].try_into().unwrap()),
            md: i16::from_be_bytes(data[20..22].try_into().unwrap()),
        };

        let validate_i16 = |&byte| { byte == 0x00 || byte == 0xFFFFu16 as i16 };
        let validate_u16 = |&byte| { byte == 0x00 || byte == 0xFFFF };

        if [instance.ac1, instance.ac2, instance.ac3, instance.b1, instance.b2, instance.mb, instance.mc, instance.md]
            .iter()
            .any(validate_i16) { return Err(CustomError::InvalidCalibrationData); }

        if [instance.ac4, instance.ac5, instance.ac6]
            .iter()
            .any(validate_u16) { return Err(CustomError::InvalidCalibrationData); }

        Ok(instance)
    }

    pub fn calculate_temperature(&self, ut: i32) -> (f32, i32) {
        let x1 = (ut - self.ac6 as i32) * self.ac5 as i32 >> 15;
        let x2 = ((self.mc as i32) << 11) / (x1 + self.md as i32);
        let b5 = x1 + x2;
        let temperature = ((b5 + 8) >> 4) as f32 / 10.0;

        (temperature, b5)
    }

    pub fn calculate_pressure(&self, b5: i32, uncompensated_pressure: i32, oss: Resolution) -> i32 {
        let oss = oss as u8;
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
        let b7: u32 = (uncompensated_pressure - b3) as u32 * (50000 >> oss);
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

    pub fn calculate_altitude(&self, pressure: i32) -> f32 {
        44330.0 * (1.0 - (pressure as f32 / 101325.0).powf(1.0 / 5.255))
    }
}

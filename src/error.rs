use std::fmt::{Debug, Display, Formatter, Pointer};
use std::error::Error;

#[derive(Debug)]
pub enum CustomError<T> {
    InvalidCalibrationData,
    InvalidDeviceIdentifier,
    I2c(T),
}

impl<T> Display for CustomError<T> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            CustomError::InvalidCalibrationData => write!(formatter, "Accor"),
            CustomError::InvalidDeviceIdentifier => write!(formatter, "{}", self),
            CustomError::I2c(error) => error.fmt(formatter)
        }
    }
}

impl<T: Debug> Error for CustomError<T> {}

impl<T> From<T> for CustomError<T> {
    fn from(error: T) -> Self {
        CustomError::I2c(error)
    }
}

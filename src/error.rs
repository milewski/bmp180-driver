use std::error::Error;
use std::fmt::{Debug, Display, Formatter, Pointer};

/// Represents custom errors that can occur during BMP180 operations.
#[derive(Debug)]
pub enum CustomError<T> {
    /// Indicates that the calibration data read from the sensor is invalid or corrupted.
    InvalidCalibrationData,

    /// Indicates that the device identifier read from the sensor does not match the expected value.
    InvalidDeviceIdentifier,

    /// Represents an I2C communication error encapsulating the inner error type `T`.
    I2c(T),
}

impl<T> Display for CustomError<T> {
    fn fmt(&self, formatter: &mut Formatter<'_>) -> std::fmt::Result {
        match self {
            CustomError::InvalidCalibrationData => write!(formatter, "Invalid calibration coefficients."),
            CustomError::InvalidDeviceIdentifier => write!(formatter, "Invalid device identifier."),
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

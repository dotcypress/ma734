//! A platform agnostic Rust driver for the ma734, based on the
//! [`embedded-hal`](https://github.com/japaric/embedded-hal) traits.
//!
//! ## The Device
//!
//! The MA734 is a MagAlpha digital angle sensor
//! that detects the absolute angular position of a permanent magnet,
//! typically a diametrically magnetized cylinder on a rotating shaft.
//!
//! - [Details and datasheet](https://www.monolithicpower.com/en/ma734.html)

#![no_std]

use core::convert::Infallible;
use embedded_hal::{blocking::spi, digital::v2::OutputPin};

pub const MA734_MODE: embedded_hal::spi::Mode = embedded_hal::spi::MODE_0;

/// Driver for the MA734
#[derive(Debug)]
pub struct MA734<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> MA734<SPI, CS>
where
    SPI: spi::Transfer<u8>,
    CS: OutputPin,
{
    /// Initialize the MA734 driver.
    pub fn new(spi: SPI, cs: CS) -> Self {
        MA734 { spi, cs }
    }

    /// Realeses SPI bus and CS pin.
    pub fn release(self) -> (SPI, CS) {
        (self.spi, self.cs)
    }

    /// Read current angle.
    pub fn read_angle(&mut self) -> Result<u16, Error<SPI>> {
        let mut scratch = [0; 2];
        self.tx(|spi| spi.transfer(&mut scratch))?;
        Ok(u16::from_be_bytes(scratch))
    }

    /// Write register
    pub fn write_register(&mut self, reg: u8, val: u8) -> Result<u8, Error<SPI>> {
        self.cmd(0x80 | (reg & 0x1f), val).map(|resp| resp[3])
    }

    /// Read register
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<SPI>> {
        self.cmd(0x40 | (reg & 0x1f), 0x0).map(|resp| resp[3])
    }

    /// Store Register into the NVM
    pub fn store_register_into_nvm(&mut self, reg: u8) -> Result<(), Error<SPI>> {
        self.cmd(0xe0 | (reg & 0x1f), 0x0).map(|_| ())
    }

    /// Store All Registers into the NVM
    pub fn store_all_registers_into_nvm(&mut self) -> Result<(), Error<SPI>> {
        self.cmd(0xc0, 0x0).map(|_| ())
    }

    /// Restore All Registers from the NVM
    pub fn restore_all_registers_from_nvm(&mut self) -> Result<(), Error<SPI>> {
        self.cmd(0xa0, 0x0).map(|_| ())
    }

    /// Clear error flags
    pub fn clear_error_flags(&mut self) -> Result<(), Error<SPI>> {
        self.cmd(0x20, 0x00).map(|_| ())
    }

    fn cmd(&mut self, cmd: u8, arg: u8) -> Result<[u8; 4], Error<SPI>> {
        let mut scratch = [cmd, arg, 0, 0];
        self.tx(|spi| spi.transfer(&mut scratch))?;
        Ok(scratch)
    }

    fn tx<RES, TX: FnOnce(&mut SPI) -> Result<RES, SPI::Error>>(
        &mut self,
        tx: TX,
    ) -> Result<RES, Error<SPI>> {
        self.cs.set_low().map_err(|_| Error::PinError)?;
        let res = tx(&mut self.spi).map_err(Error::TransferError);
        self.cs.set_high().map_err(|_| Error::PinError).and(res)
    }
}

pub enum Error<SPI: spi::Transfer<u8>> {
    PinError,
    TransferError(<SPI as spi::Transfer<u8>>::Error),
}

impl<SPI: spi::Transfer<u8>> core::fmt::Debug for Error<SPI> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::PinError => write!(f, "GPIO Error"),
            Self::TransferError(_) => write!(f, "SPI Transfer Error"),
        }
    }
}

pub struct NoCS;

impl OutputPin for NoCS {
    type Error = Infallible;

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }

    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
}

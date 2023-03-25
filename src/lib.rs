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
        let mut buf = [0x00; 2];
        self.tx(|spi| spi.transfer(&mut buf))?;
        Ok(u16::from_be_bytes(buf))
    }

    /// Get zero position angle
    pub fn get_zero_angle(&mut self) -> Result<u16, Error<SPI>> {
        let angle = u16::from_le_bytes([self.read_register(0x00)?, self.read_register(0x01)?]);
        Ok(angle)
    }

    /// Set zero position angle
    pub fn set_zero_angle(&mut self, angle: u16) -> Result<(), Error<SPI>> {
        self.write_register(0x00, angle as _)?;
        self.write_register(0x01, (angle >> 8) as _)?;
        Ok(())
    }

    /// Get bias current trimming settings
    pub fn get_bias_current_trimming(&mut self) -> Result<BiasCurrentTrimming, Error<SPI>> {
        let val = self.read_register(0x02)?;
        let axis_trims = self.read_register(0x03)?;
        Ok(BiasCurrentTrimming {
            val,
            trim_x: axis_trims & 0b1 > 0,
            trim_y: axis_trims & 0b10 > 0,
        })
    }

    /// Set bias current trimming settings
    pub fn set_bias_current_trimming(
        &mut self,
        bct: BiasCurrentTrimming,
    ) -> Result<(), Error<SPI>> {
        self.write_register(0x02, bct.val)?;
        self.write_register(0x03, bct.trim_x as u8 | (bct.trim_y as u8) << 1)
            .map(|_| ())
    }

    /// Get angle change interrupt config
    pub fn get_angle_change_interrupt(&mut self) -> Result<AngleChangeInterrupt, Error<SPI>> {
        let aci = self.read_register(0x07)?;
        let threshold = self.read_register(0x08)?;
        let reference = self.read_register(0x0a)?;
        Ok(AngleChangeInterrupt {
            threshold,
            reference,
            hysteresis: aci & 0b11111,
            autoupdate: aci & 0b1000000 > 0,
            mode: if aci & 0b10000000 > 0 {
                IRQMode::LatchOff
            } else {
                IRQMode::Logic
            },
        })
    }

    /// Set angle change interrupt config
    pub fn set_angle_change_interrupt(
        &mut self,
        aci: AngleChangeInterrupt,
    ) -> Result<(), Error<SPI>> {
        let mode_bits = aci.hysteresis & 0b11111;
        let mode_bit = (aci.mode == IRQMode::LatchOff) as u8;
        let autoupdate_bit = aci.autoupdate as u8;
        self.write_register(0x07, mode_bits | autoupdate_bit << 6 | mode_bit << 7)?;
        self.write_register(0x08, aci.threshold)?;
        self.write_register(0x0a, aci.reference).map(|_| ())
    }

    /// Get magnetic field thresholds
    pub fn get_magnetic_thresholds(&mut self) -> Result<MagneticFieldThresholds, Error<SPI>> {
        let val = self.read_register(0x06)?;
        Ok(MagneticFieldThresholds {
            enable: val & 0b1 > 0,
            low: (val >> 2) & 0b111,
            high: (val >> 5) & 0b111,
        })
    }

    /// Set magnetic field thresholds
    pub fn set_magnetic_field_thresholds(
        &mut self,
        mft: MagneticFieldThresholds,
    ) -> Result<(), Error<SPI>> {
        let payload = mft.enable as u8 | (mft.low & 0b111) << 2 | (mft.high & 0b111) << 5;
        self.write_register(0x06, payload).map(|_| ())
    }

    /// Set rotation direction
    pub fn set_rotation_dir(&mut self, val: Dir) -> Result<(), Error<SPI>> {
        self.write_register(0x09, val.into()).map(|_| ())
    }

    /// Get rotation direction
    pub fn get_rotation_dir(&mut self) -> Result<Dir, Error<SPI>> {
        self.read_register(0x09).map(From::from)
    }

    /// Set filter window
    pub fn set_filter_window(&mut self, val: u8) -> Result<(), Error<SPI>> {
        self.write_register(0x0e, val << 4).map(|_| ())
    }

    /// Get filter window
    pub fn get_filter_window(&mut self) -> Result<u8, Error<SPI>> {
        self.read_register(0x0e).map(|w| w >> 4)
    }

    /// Write register
    pub fn write_register(&mut self, reg: u8, val: u8) -> Result<u8, Error<SPI>> {
        self.cmd(0x80 | (reg & 0x1f), val)
    }

    /// Read register
    pub fn read_register(&mut self, reg: u8) -> Result<u8, Error<SPI>> {
        self.cmd(0x40 | (reg & 0x1f), 0x00)
    }

    /// Store Register into the NVM
    pub fn store_register_into_nvm(&mut self, reg: u8) -> Result<(), Error<SPI>> {
        self.cmd(0xe0 | (reg & 0x1f), 0x00).map(|_| ())
    }

    /// Store All Registers into the NVM
    pub fn store_all_registers_into_nvm(&mut self) -> Result<(), Error<SPI>> {
        self.cmd(0xc0, 0x00).map(|_| ())
    }

    /// Restore All Registers from the NVM
    pub fn restore_all_registers_from_nvm(&mut self) -> Result<(), Error<SPI>> {
        self.cmd(0xa0, 0x00).map(|_| ())
    }

    /// Get magnetic flags
    pub fn get_magnetic_flags(&mut self) -> Result<MagneticFlags, Error<SPI>> {
        self.read_register(0x1b).map(From::from)
    }

    /// Get error flags
    pub fn get_error_flags(&mut self) -> Result<ErrorFlags, Error<SPI>> {
        self.read_register(0x1a).map(From::from)
    }

    /// Clear error flags
    pub fn clear_error_flags(&mut self) -> Result<(), Error<SPI>> {
        self.cmd(0x20, 0x00).map(|_| ())
    }

    fn cmd(&mut self, reg: u8, arg: u8) -> Result<u8, Error<SPI>> {
        let mut buf = [reg, arg];
        self.tx(|spi| spi.transfer(&mut buf))?;
        let mut buf = [0x00; 2];
        self.tx(|spi| spi.transfer(&mut buf))?;
        Ok(buf[1])
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

/// IRQ Mode
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IRQMode {
    LatchOff,
    Logic,
}

/// Angle Change Interrupt Config
#[derive(Clone, Copy, Debug)]
pub struct AngleChangeInterrupt {
    pub mode: IRQMode,
    pub autoupdate: bool,
    pub threshold: u8,
    pub reference: u8,
    pub hysteresis: u8,
}

impl AngleChangeInterrupt {
    pub fn new(
        mode: IRQMode,
        autoupdate: bool,
        threshold: u8,
        reference: u8,
        hysteresis: u8,
    ) -> Self {
        Self {
            mode,
            autoupdate,
            threshold,
            reference,
            hysteresis,
        }
    }
}

/// Magnetic Field Thresholds Config
#[derive(Clone, Copy, Debug)]
pub struct MagneticFieldThresholds {
    pub enable: bool,
    pub low: u8,
    pub high: u8,
}

impl MagneticFieldThresholds {
    pub fn new(enable: bool, low: u8, high: u8) -> Self {
        Self { enable, low, high }
    }
}

/// Bias Current Trimming Config
#[derive(Clone, Copy, Debug)]
pub struct BiasCurrentTrimming {
    pub val: u8,
    pub trim_x: bool,
    pub trim_y: bool,
}

impl BiasCurrentTrimming {
    pub fn new(val: u8, trim_x: bool, trim_y: bool) -> Self {
        Self {
            val,
            trim_x,
            trim_y,
        }
    }
}

/// Error flags
#[derive(Clone, Copy, Debug)]
pub struct ErrorFlags {
    pub nvm_error: bool,
    pub memory_error: bool,
    pub parity_error: bool,
}

impl ErrorFlags {
    pub fn new(nvm_error: bool, memory_error: bool, parity_error: bool) -> Self {
        Self {
            nvm_error,
            memory_error,
            parity_error,
        }
    }
}

impl From<u8> for ErrorFlags {
    fn from(val: u8) -> Self {
        Self {
            nvm_error: val & 0b10 > 0,
            memory_error: val & 0b100 > 0,
            parity_error: val & 0b1000 > 0,
        }
    }
}

/// Magnetic flags
#[derive(Clone, Copy, Debug)]
pub struct MagneticFlags {
    pub low: bool,
    pub high: bool,
}

impl MagneticFlags {
    pub fn new(low: bool, high: bool) -> Self {
        Self { low, high }
    }
}

impl From<u8> for MagneticFlags {
    fn from(val: u8) -> Self {
        Self {
            low: val & 0b1000000 > 0,
            high: val & 0b10000000 > 0,
        }
    }
}

/// Rotation Direction
#[derive(Clone, Copy, Debug)]
pub enum Dir {
    /// Clockwise
    CW,
    //Counterclockwise
    CCW,
}

impl From<Dir> for u8 {
    fn from(val: Dir) -> u8 {
        match val {
            Dir::CW => 0,
            Dir::CCW => 0x80,
        }
    }
}

impl From<u8> for Dir {
    fn from(val: u8) -> Self {
        if val & 0x80 == 0x80 {
            Self::CCW
        } else {
            Self::CW
        }
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

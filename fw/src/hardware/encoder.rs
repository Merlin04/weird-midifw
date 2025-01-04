use embedded_hal::i2c::I2c;
use super::i2c_mp::{with_i2c, I2cBankBus};

const AS5600_ADDR: u8 = 0x36;
const AS5600_STATUS: u8 = 0x0B;
const AS5600_MAGNET_HIGH: u8 = 0x08;
const AS5600_MAGNET_LOW: u8 = 0x10;
const AS5600_MAGNET_DETECT: u8 = 0x20;
const AS5600_OUT_ANGLE: u8 = 0x0E;

pub struct EncoderAS5600<I2C: I2c + 'static> {
    pub bank: &'static I2cBankBus<I2C>,
    pub index: u8,
    steps_per_inc: i16,
    step_count: i16,
    prev_angle: u16
}

#[derive(Debug)]
pub enum EncoderInitError {
    NoMagnetDetectedError,
    MagnetTooStrongError,
    MagnetTooWeakError,
    I2cError
}
impl core::fmt::Display for EncoderInitError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match *self {
            EncoderInitError::NoMagnetDetectedError => write!(f, "No magnet detected"),
            EncoderInitError::MagnetTooStrongError => write!(f, "Magnet too strong"),
            EncoderInitError::MagnetTooWeakError => write!(f, "Magnet too weak"),
            EncoderInitError::I2cError => write!(f, "I2C error"), 
        }
    }
}
impl core::error::Error for EncoderInitError {}

impl<I2C: I2c> EncoderAS5600<I2C> {
    pub fn new(bank: &'static I2cBankBus<I2C>, index: u8, incs_per_rev: Option<i16>) -> Result<EncoderAS5600<I2C>, EncoderInitError> {
        let mut e = EncoderAS5600 {
            bank,
            index,
            steps_per_inc: incs_per_rev.map_or(128, |i| 4096 / i),
            step_count: 0,
            prev_angle: 0
        };
        e.init()?;
        Ok(e)
    }

    fn init(&mut self) -> Result<(), EncoderInitError> {
        let status = self.read_register(AS5600_STATUS).map_err(|_| EncoderInitError::I2cError)?;
        self.prev_angle = self.read_angle().map_err(|_| EncoderInitError::I2cError)?;

        if (status & AS5600_MAGNET_DETECT) <= 1 {
            Err(EncoderInitError::NoMagnetDetectedError)
        } else if (status & AS5600_MAGNET_HIGH) > 1 {
            Err(EncoderInitError::MagnetTooStrongError)
        } else if (status & AS5600_MAGNET_LOW) > 1 {
            Err(EncoderInitError::MagnetTooWeakError)
        } else { Ok(()) }
    }

    fn read_register(&mut self, reg: u8) -> Result<u8, I2C::Error> {
        with_i2c::<I2C, _, _, _>(&self.bank, |bank, cs| {
            bank.select(self.index)?;
            let i2c = &mut *bank.borrow_ref_mut(cs);
            i2c.write(AS5600_ADDR, &[reg])?;
            let mut res: [u8; 1] = [0];
            i2c.read(AS5600_ADDR, &mut res)?;
            Ok(res[0])
        })
    }

    fn read_register_16(&mut self, reg: u8) -> Result<u16, I2C::Error> {
        with_i2c::<I2C, _, _, _>(&self.bank, |bank, cs| {
            bank.select(self.index)?;
            let i2c = &mut *bank.borrow_ref_mut(cs);
            i2c.write(AS5600_ADDR, &[reg])?;
            let mut res_bytes: [u8; 2] = [0, 0];
            i2c.read(AS5600_ADDR, &mut res_bytes)?;
            // index 0 is high byte, index 1 is low byte (big-endian)
            Ok(((res_bytes[0] as u16) << 8) | res_bytes[1] as u16)    
        })
    }

    fn read_angle(&mut self) -> Result<u16, I2C::Error> {
        let res = self.read_register_16(AS5600_OUT_ANGLE)?;
        Ok(res & 0x0FFF)
    }

    // should be called repeatedly, maybe every 2ms?
    pub fn process(&mut self) -> Result<i16, I2C::Error> {
        let angle = self.read_angle()?;
        let mut diff: i16 = i16::try_from(angle).unwrap() - i16::try_from(self.prev_angle).unwrap();
        if diff > 2048 {
            diff -= 4096;
        } else if diff < -2048 {
            diff += 4096;
        }
        self.step_count += diff;
        self.prev_angle = angle;

        let incs: i16 = self.step_count / self.steps_per_inc;
        if incs != 0 {
            self.step_count %= self.steps_per_inc;
        }

        Ok(incs)
    }
}

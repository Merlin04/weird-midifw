use core::cell::RefCell;

use critical_section::{CriticalSection, Mutex};
use embedded_hal::i2c::I2c;

pub struct MultiplexedI2c<I2C: I2c> {
    tca_cur: u8,
    i2c: I2C
}

impl<I2C: I2c> core::ops::Deref for MultiplexedI2c<I2C> {
    type Target = I2C;
    fn deref(&self) -> &Self::Target {
        &self.i2c
    }
}
impl<I2C: I2c> core::ops::DerefMut for MultiplexedI2c<I2C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        &mut self.i2c
    }
}

impl<I2C: I2c> MultiplexedI2c<I2C> {
    pub fn new(i2c: I2C) -> Result<MultiplexedI2c<I2C>, I2C::Error> {
        let mut m = MultiplexedI2c {
            i2c,
            tca_cur: 0
        };
        m.write(0x70, &[0])?;
        Ok(m)
    }

    pub fn tca_set(&mut self, index: u8, enabled: bool) -> Result<(), I2C::Error> {
        let bitmask: u8 = 0b1 << index;
        let new_tca: u8 = if enabled {
            self.tca_cur | bitmask
        } else {
            self.tca_cur & !bitmask
        };
        self.write(0x70, &[new_tca])?;
        self.tca_cur = new_tca;
        Ok(())
    }
}

pub type I2cBus<I2C> = Mutex<RefCell<MultiplexedI2c<I2C>>>;

pub struct I2cBank<I2C: I2c + 'static> {
    pub start: u8,
    pub len: u8,
    pub active: u8,
    pub i2c: &'static I2cBus<I2C>
}

pub type I2cBankBus<I2C> = Mutex<RefCell<I2cBank<I2C>>>;

impl<I2C: I2c> core::ops::Deref for I2cBank<I2C> {
    type Target = I2cBus<I2C>;
    fn deref(&self) -> &Self::Target {
        &self.i2c
    }
}

pub fn with_i2c<I2C: I2c, I: 'static, T, F: FnOnce(&mut I, CriticalSection<'_>) -> Result<T, I2C::Error>>(
    bus: &Mutex<RefCell<I>>, f: F
) -> Result<T, I2C::Error> {
    critical_section::with(|cs| {
        let i2c = &mut *bus.borrow_ref_mut(cs);
        f(i2c, cs)
    })
}

impl<I2C: I2c> I2cBank<I2C> {
    pub fn select(&mut self, index: u8) -> Result<(), I2C::Error> {
        with_i2c::<I2C, _, _, _>(&self.i2c, |i2c: &mut MultiplexedI2c<I2C>, _| {
            i2c.tca_set(self.start + self.active, false)?;
            i2c.tca_set(self.start + index, true)?;
            Ok(())
        })?;
        self.active = index;
        Ok(())
    }
}
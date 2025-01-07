use teensy4_bsp::{hal::gpio::{Input, Output, Port, Trigger::RisingEdge}, pins};

use crate::utils::create_trait;

// the actual grid of keys is 9*6
// 6*2 rows + 1 for ctrl keys
pub const N_ROWS: usize = 13;
// 9 cols
pub const N_COLS: usize = 9;

pub const N_OUTPUTS: usize = N_COLS;
pub const N_INPUTS: usize = N_ROWS;

type MatrixIn0 = Input<pins::t41::P35>;
type MatrixIn1 = Input<pins::t41::P36>;
type MatrixIn2 = Input<pins::t41::P37>;
type MatrixIn3 = Input<pins::t41::P38>;
type MatrixIn4 = Input<pins::t41::P39>;
type MatrixIn5 = Input<pins::t41::P40>;
type MatrixIn6 = Input<pins::t41::P41>;
type MatrixIn7 = Input<pins::t41::P14>;
type MatrixIn8 = Input<pins::t41::P15>;
type MatrixIn9 = Input<pins::t41::P25>;
type MatrixIn10 = Input<pins::t41::P24>;
type MatrixIn11 = Input<pins::t41::P12>;
type MatrixIn12 = Input<pins::t41::P7>;

pub type MatrixInPins = (MatrixIn0, MatrixIn1, MatrixIn2, MatrixIn3, MatrixIn4, MatrixIn5, MatrixIn6, MatrixIn7, MatrixIn8, MatrixIn9, MatrixIn10, MatrixIn11, MatrixIn12);

type MatrixOut0 = Output<pins::t41::P32>;
type MatrixOut1 = Output<pins::t41::P31>;
type MatrixOut2 = Output<pins::t41::P30>;
type MatrixOut3 = Output<pins::t41::P29>;
type MatrixOut4 = Output<pins::t41::P28>;
type MatrixOut5 = Output<pins::t41::P27>;
type MatrixOut6 = Output<pins::t41::P26>;
type MatrixOut7 = Output<pins::t41::P33>;
type MatrixOut8 = Output<pins::t41::P34>;

pub type MatrixOutPins = (MatrixOut0, MatrixOut1, MatrixOut2, MatrixOut3, MatrixOut4, MatrixOut5, MatrixOut6, MatrixOut7, MatrixOut8);

create_trait!(Input<P>, InputTrait, {
    fn is_set() -> bool;
});

create_trait!(Output<P>, OutputTrait, {
    fn set() -> ();
    fn clear() -> ();
});

pub trait MatrixInPinsExt {
    fn as_array(&self) -> [&dyn InputTrait; N_ROWS];
}
impl MatrixInPinsExt for MatrixInPins {
    fn as_array(&self) -> [&dyn InputTrait; N_ROWS] {
        [
            &self.0,
            &self.1,
            &self.2,
            &self.3,
            &self.4,
            &self.5,
            &self.6,
            &self.7,
            &self.8,
            &self.9,
            &self.10,
            &self.11,
            &self.12
        ]
    }    
}

pub trait MatrixOutPinsExt {
    fn as_array(&self) -> [&dyn OutputTrait; N_COLS];
}
impl MatrixOutPinsExt for MatrixOutPins {
    fn as_array(&self) -> [&dyn OutputTrait; N_COLS] {
        [
            &self.0,
            &self.1,
            &self.2,
            &self.3,
            &self.4,
            &self.5,
            &self.6,
            &self.7,
            &self.8
        ]
    }    
}

pub fn enable_matrix_interrupts(gpio1: &mut Port<1>, gpio2: &mut Port<2>, matrix_in_pins: &MatrixInPins) {
    gpio2.set_interrupt(&matrix_in_pins.0, Some(RisingEdge));
    gpio2.set_interrupt(&matrix_in_pins.1, Some(RisingEdge));
    gpio2.set_interrupt(&matrix_in_pins.2, Some(RisingEdge));

    gpio1.set_interrupt(&matrix_in_pins.3, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.4, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.5, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.6, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.7, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.8, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.9, Some(RisingEdge));
    gpio1.set_interrupt(&matrix_in_pins.10, Some(RisingEdge));

    gpio2.set_interrupt(&matrix_in_pins.11, Some(RisingEdge));
    gpio2.set_interrupt(&matrix_in_pins.12, Some(RisingEdge));
}

pub fn disable_matrix_interrupts(gpio1: &mut Port<1>, gpio2: &mut Port<2>, matrix_in_pins: &MatrixInPins) {
    gpio2.set_interrupt(&matrix_in_pins.0, None);
    gpio2.set_interrupt(&matrix_in_pins.1, None);
    gpio2.set_interrupt(&matrix_in_pins.2, None);

    gpio1.set_interrupt(&matrix_in_pins.3, None);
    gpio1.set_interrupt(&matrix_in_pins.4, None);
    gpio1.set_interrupt(&matrix_in_pins.5, None);
    gpio1.set_interrupt(&matrix_in_pins.6, None);
    gpio1.set_interrupt(&matrix_in_pins.7, None);
    gpio1.set_interrupt(&matrix_in_pins.8, None);
    gpio1.set_interrupt(&matrix_in_pins.9, None);
    gpio1.set_interrupt(&matrix_in_pins.10, None);
    
    gpio2.set_interrupt(&matrix_in_pins.11, None);
    gpio2.set_interrupt(&matrix_in_pins.12, None);
}

#[derive(PartialEq)]
pub enum KeyType {
    Upper,
    Lower,
    Ctrl
}

pub const fn key_type(row: u8, _col: u8) -> KeyType {
    if row == 12 {
        KeyType::Ctrl
    } else if row % 2 == 0 {
        KeyType::Upper
    } else {
        KeyType::Lower
    }
}
// workaround for https://github.com/rust-embedded-community/usbd-midi/issues/8

use usbd_midi::data::{byte::{u4::U4, u7::U7}, midi::{channel::Channel, message::raw::{Payload, Raw}}, usb_midi::{cable_number::CableNumber, code_index_number::CodeIndexNumber}};

pub struct NoteMessage {
    pub on: bool,
    pub chan: Channel,
    pub note: U7,
    pub vel: U7
}

const NOTE_OFF_MASK: u8 = 0b1000_0000;
const NOTE_ON_MASK: u8 = 0b1001_0000;

impl From<NoteMessage> for Raw {
    fn from(value: NoteMessage) -> Self {
        let payload = Payload::DoubleByte(value.note, value.vel);
        let status = (if value.on { NOTE_ON_MASK } else { NOTE_OFF_MASK }) | u8::from(value.chan);
        Raw { status, payload }
    }
}

pub struct NoteMessageEventPacket {
    pub cable_number: CableNumber,
    pub message: NoteMessage
}

impl From<NoteMessageEventPacket> for [u8; 4] {
    fn from(value: NoteMessageEventPacket) -> Self {
        let message = value.message;
        let cable_number = U4::from(value.cable_number);
        let index_number = {
            let code_index = if message.on { CodeIndexNumber::NOTE_ON } else { CodeIndexNumber::NOTE_OFF };
            U4::from(code_index)
        };
        let header = U4::combine(cable_number, index_number);

        let raw_midi = Raw::from(message);
        let status = raw_midi.status;

        match raw_midi.payload {
            Payload::Empty => [header, status, 0, 0],
            Payload::SingleByte(byte) => [header, status, byte.into(), 0],
            Payload::DoubleByte(byte1, byte2) => [header, status, byte1.into(), byte2.into()],
        }
    }
}
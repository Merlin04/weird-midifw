use core::cell::RefCell;

use critical_section::Mutex;
use fugit::Instant;
use rtic_monotonics::{systick::{Systick, *}, Monotonic};
use rtic_sync::channel::Sender;
use rust_fsm::*;
use velocity_sense::Output;

state_machine! {
    #[derive(Debug)]

    pub velocity_sense(Initial)

    Initial(TopPress) => TopPressed [DoTopPress],
    TopPressed => {
        TopRelease => Initial [DoTopTap],
        BottomPress => BottomPressed [CalcVelocityAndNoteOn],
        Timeout => TimedOut [VelocitySoftAndNoteOn]
    },
    BottomPressed => {
        TopRelease => Initial [SendNoteOff], // unexpected
        BottomRelease => BottomReleased [SendNoteOff]
    },
    TimedOut => {
        TopRelease => Initial [SendNoteOff],
        BottomPress => ExtraBottomPressed [SendNoteOn]
    },
    BottomReleased => {
        BottomPress => ExtraBottomPressed [SendNoteOn],
        TopRelease => Initial
    },
    ExtraBottomReleased(BottomPress) => BottomReleased [SendNoteOff]
}

const VELOCITY_DEFAULT: u8 = 127;
const VELOCITY_SOFT: u8 = 64;

const PRESS_TIMEOUT: u32 = 30;

#[derive(Debug)]
pub struct KeyEvent {
    pub row: u8,
    pub col: u8,
    pub velocity: u8,
    pub on: bool
}

pub const KEY_EVENT_CAPACITY: usize = 20; // ??
type KeyEventSender = Sender<'static, KeyEvent, KEY_EVENT_CAPACITY>;

pub struct Key/*<F> where F: FnMut(u8, u8) -> ()*/ {
    machine: velocity_sense::StateMachine,
    pub row: u8,
    pub col: u8,
    velocity: u8,
    top_press_time: Instant<u32, 1, 1000>, // return type of Systick::now
    sender: KeyEventSender,
    start_timeout_callback: &'static Mutex<RefCell<&'static mut dyn FnMut(u8, u8) -> ()>>
}
impl/*<F>*/ Key/*<F> where F: FnMut(u8, u8) -> ()*/ {
    async fn send_event(&mut self, on: bool) {
        self.sender.send(KeyEvent {
            row: self.row,
            col: self.col,
            velocity: self.velocity,
            on
        }).await.unwrap();
    }
    fn velocity_soft(&mut self) {
        self.velocity = VELOCITY_SOFT;
    }
    fn calc_velocity(&mut self) {

    }
    async fn top_tap(&mut self) {
        self.velocity = VELOCITY_DEFAULT;
        self.send_event(true).await;
        self.send_event(false).await; // TODO does this actually work?
    }
    async fn do_top_press(&mut self) {
        // record top press time
        self.top_press_time = Systick::now();
        // start timeout
        // unsure if this is the best way to do this?
        // (self.start_timeout_callback)(self.row, self.col);
        critical_section::with(|cs| {
            let start_timeout_callback = &mut *self.start_timeout_callback.borrow_ref_mut(cs);
            start_timeout_callback(self.row, self.col);
        });
    }
    
    pub async fn process_event(&mut self, event: velocity_sense::Input) {
        let result = self.machine.consume(&event);
        match result {
            Ok(Some(output)) => match output {
                Output::CalcVelocityAndNoteOn => {
                    self.calc_velocity();
                    self.send_event(true).await;
                },
                Output::DoTopTap => self.top_tap().await,
                Output::SendNoteOff => self.send_event(false).await,
                Output::SendNoteOn => self.send_event(true).await,
                Output::VelocitySoftAndNoteOn => {
                    self.velocity_soft();
                    self.send_event(true).await;
                },
                Output::DoTopPress => self.do_top_press().await
            },
            Err(_) => defmt::warn!("velocity state machine received event it can't handle"),
            _ => ()
        };

    }

    pub fn new(row: u8, col: u8, sender: KeyEventSender, start_timeout_callback: &'static Mutex<RefCell<&mut dyn FnMut(u8, u8)>>) -> Self {
        Key {
            machine: velocity_sense::StateMachine::new(),
            row,
            col,
            velocity: 0,
            top_press_time: Systick::now(),
            sender,
            start_timeout_callback
        }
    }
}
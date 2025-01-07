use rtic_monotonics::{systick::{Systick, *}, Monotonic};
use rtic_sync::channel::Sender;
use rust_fsm::*;
use velocity_sense::Output;

use crate::utils::{RangeMappable, SystickInstant};

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
    ExtraBottomPressed(BottomRelease) => BottomReleased [SendNoteOff]
}

const VELOCITY_DEFAULT: u8 = 200;
const VELOCITY_SOFT: u8 = 100;

#[derive(Debug)]
pub struct KeyEvent {
    pub row: u8,
    pub col: u8,
    pub velocity: u8,
    pub on: bool
}

pub const KEY_EVENT_CAPACITY: usize = 20; // ??
type KeyEventSender = Sender<'static, KeyEvent, KEY_EVENT_CAPACITY>;

const VELOCITY_TIMEOUT_MILLIS: u32 = 100;

pub struct Key {
    machine: velocity_sense::StateMachine,
    pub row: u8,
    pub col: u8,
    velocity: u8,
    top_press_time: SystickInstant,
    sender: KeyEventSender,
    pub timeout_on: Option<SystickInstant>
}
impl Key {
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
        self.timeout_on = None;
        let delay = Systick::now() - self.top_press_time;
        // rough velocity curve
        // similar to melodicade mx
        // TODO: test this and adjust as necessary
        self.velocity = f64::from(delay.to_millis()).map_range_array([
            (8., 255.),
            (20., 220.),
            (50., 188.),
            (100., 150.)
        ]) as u8;
    }
    async fn top_tap(&mut self) {
        self.timeout_on = None;
        self.velocity = VELOCITY_DEFAULT;
        self.send_event(true).await;
        self.send_event(false).await; // TODO does this actually work?
    }
    async fn do_top_press(&mut self) {
        // record top press time
        self.top_press_time = Systick::now();
        // start timeout
        self.timeout_on = Some(self.top_press_time + VELOCITY_TIMEOUT_MILLIS.millis());
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

    pub fn new(row: u8, col: u8, sender: KeyEventSender) -> Self {
        Key {
            machine: velocity_sense::StateMachine::new(),
            row,
            col,
            velocity: 0,
            top_press_time: Systick::now(),
            sender,
            timeout_on: None
        }
    }
}
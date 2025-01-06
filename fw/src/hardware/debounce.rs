/*
 * Copyright (c) 2021 The ZMK Contributors
 * Copyright (c) Merlin04
 *
 * SPDX-License-Identifier: MIT
 */

use bitfield::bitfield;

bitfield! {
    pub struct Debounce(u16);
    pub pressed, set_pressed: 0;
    pub changed, set_changed: 1;
    counter, set_counter: 15, 2;
}

const DEBOUNCE_COUNTER_BITS: u8 = 14;
const DEBOUNCE_COUNTER_MAX: u16 = (1 << DEBOUNCE_COUNTER_BITS) - 1;

// https://zmk.dev/docs/features/debouncing
// instant activate
const INST_DEBOUNCE_PRESS_MS: u32 = 0;
const INST_DEBOUNCE_RELEASE_MS: u32 = 1;

impl Debounce {
    fn get_threshold(&self) -> u32 {
        if self.pressed() { INST_DEBOUNCE_RELEASE_MS } else { INST_DEBOUNCE_PRESS_MS }
    }

    fn increment_counter(&mut self, elapsed_ms: u16) {
        if self.counter() + elapsed_ms > DEBOUNCE_COUNTER_MAX {
            self.set_counter(DEBOUNCE_COUNTER_MAX);
        } else {
            self.set_counter(self.counter() + elapsed_ms);
        }
    }

    fn decrement_counter(&mut self, elapsed_ms: u16) {
        if self.counter() < elapsed_ms {
            self.set_counter(0);
        } else {
            self.set_counter(self.counter() - elapsed_ms);
        }
    }

    /// Debounces one switch.
    /// `active`: Is the switch currently pressed?
    /// `elapsed_ms`: Time elapsed since the previous update in milliseconds.
    /// `config`: Debounce settings.
    pub fn update(&mut self, active: bool, elapsed_ms: u16) {
        // This uses a variation of the integrator debouncing described at
        // https://www.kennethkuhn.com/electronics/debounce.c
        // Every update where "active" does not match the current state, we increment
        // a counter, otherwise we decrement it. When the counter reaches a
        // threshold, the state flips and we reset the counter.
        self.set_changed(false);

        if self.pressed() == active {
            self.decrement_counter(elapsed_ms);
            return;
        }

        let flip_threshold: u32 = self.get_threshold();

        if u32::from(self.counter()) < flip_threshold {
            self.increment_counter(elapsed_ms);
            return;
        }

        self.set_pressed(!self.pressed());
        self.set_counter(0);
        self.set_changed(true);
    }


    /// Returns whether the switch is either latched as pressed or it is potentially
    /// pressed but the debouncer has not yet made a decision. If this returns true,
    /// the kscan driver should continue to poll quickly.
    pub fn is_active(&self) -> bool {
        self.pressed() || self.counter() > 0
    }
}
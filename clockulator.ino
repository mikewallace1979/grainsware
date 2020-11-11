/*

A probabilistic clock divider/multiplier for the Ginko Synthese Grains eurorack module.

Derived from code by a773 (atte.dk):
- https://github.com/attejensen/a773_grains/blob/784c28658dfbdefe98246051a63527a0c8f1063b/mult_div/mult_div.ino

Released under the GPL licence.

Guaranteed 100% not bug free, i.e.: it's going to have bugs.

User guide:

# Input 1 / Upper pot

- Probability that a division will actually trigger.
- Fully counter-clockwise nothing will be sent to the output.
- Fully clockwise every division will be sent to the output.

# Input 2 / Middle pot

- Size of the resulting divisions relative to the incoming pulse.
- From fully CCW to fully CW: 4, 2, 1, 0.25.

# Input 3

- Clock input.
- Triggers or gates should both work.
- Only tested with the clock from the Arturia Keystep but can confirm
  it works for me.

# Lower pot

- Defines what to send to the output.
- Fully CCW: Random analogue voltages via the Grains filtered PWM output.
             I'd hoped this would be usable for pitch but it's not stable enough.
             It works ok when fed into modulation inputs so if you just want
             envelope decay to switch to a new random value on every clock pulse
             or similar then this has got you covered.
- Left of 12-o-clock: Triggers.
- Right of 12-o-clock: Gates.

# Gotchas

The main gotcha is that something about the clock output doesn't play
nicely triggering envelopes - it kinda works but not always. If you
buffer it through something else it's fine. I don't know what's going
on there but I'm guessing it's related to the output being low-pass
filtered?

The other gotcha is that the clock div/mult logic is really basic
and only cares about the next interval.

Also the pots are not linear but the mapping to values is linear so
bear that in mind (or file a PR... :P).

*/

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>

#define TRIGGER_LENGTH 10
#define UPPER_POT       2
#define MIDDLE_POT      1
#define LOWER_POT       0
#define CLOCK_IN        3
#define RND_PIN         4
#define UPPER_POT_MAX   500
#define MIDDLE_POT_MAX  500
#define LOWER_POT_MAX   500
#define PWM_PIN         11
#define GATE_THRESHOLD  32

boolean in_clock_high = false;
long last_trigger_in = 0;
long next_trigger_out = 0;
long last_trigger_out = 0;

int gate_length = TRIGGER_LENGTH;
int interval = 0;
boolean out_high = false;

enum mode {
  mode_random_cv,
  mode_trigger,
  mode_gate
};

void setup() {
  randomSeed(analogRead(RND_PIN));
  analogWrite(PWM_PIN, random(255));
}

long get_time() {
  return millis();
}

void trigger(mode current_mode) {
  if (!should_trigger()) {
    return;
  }
  out_high = true;
  if (current_mode == mode_random_cv) {
    analogWrite(PWM_PIN, random(255));
  } else {
    analogWrite(PWM_PIN, 255);
  }
}

mode get_mode() {
  switch (map(analogRead(LOWER_POT), 0, LOWER_POT_MAX, 0, 2)) {
    case 0:
      return mode_random_cv;
    case 1:
      return mode_trigger;
    case 2:
      return mode_gate;
  }
}

float get_clock_factor() {
  switch(map(analogRead(UPPER_POT), 0, UPPER_POT_MAX, 0, 3)) {
    case 0:
      return 4;
    case 1:
      return 2;
    case 2:
      return 1;
    case 3:
      return 0.5;
  }
}

int get_unc_factor() {
  return map(analogRead(MIDDLE_POT), 0, MIDDLE_POT_MAX, 0, 256);
}

boolean should_trigger() {
  return random(255) < get_unc_factor();
}

void loop() 
{
  mode current_mode = get_mode();
  float clock_factor = get_clock_factor();
  int gate = analogRead(CLOCK_IN);
  long current_time = get_time();
  boolean clock_identity = clock_factor > 0.5 && clock_factor < 2;
  int trigger_length = current_mode == mode_gate ? gate_length : TRIGGER_LENGTH;

  // Determine whether any outgoing gate/trigger needs to be zero'd
  if (out_high && current_mode != mode_random_cv && (current_time - last_trigger_out) > trigger_length) {
    out_high = false;
    analogWrite(PWM_PIN, 0);
  }

  // Determine if we should trigger and do so
  if (!clock_identity && interval > 0 && next_trigger_out < current_time) {
    trigger(current_mode);
    if (last_trigger_out > 0) {
      gate_length = (current_time - last_trigger_out) / 2;
    }
    last_trigger_out = current_time;
    next_trigger_out = current_time + (int) (interval * clock_factor);
  }

  // Update the state based on incoming clock
  if (gate > GATE_THRESHOLD) {
    if (!in_clock_high) {
      // Rising edge
      if (clock_identity) {
        // If the factor is 1 then short circuit the clock tracking and just trigger
        trigger(current_mode);
        if (last_trigger_out > 0) {
          gate_length = (current_time - last_trigger_out) / 2;
        }
        last_trigger_out = current_time;
      }
      // Update the interval
      if (last_trigger_in > 0) {
        interval = current_time - last_trigger_in;
      }
      last_trigger_in = current_time;
      in_clock_high = true;
    }
  } else {
    if (in_clock_high) {
      // Falling edge
      in_clock_high = false;
    }
  }
}

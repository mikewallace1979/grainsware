/*

A probabilistic clock divider for the Ginko Synthese Grains eurorack module.

Derived from code by a773 (atte.dk):
- https://github.com/attejensen/a773_grains/blob/784c28658dfbdefe98246051a63527a0c8f1063b/mult_div/mult_div.ino

Released under the GPL licence.

Guaranteed 100% not bug free, i.e.: it's going to have bugs.

User guide:

# Input 1 / Upper pot

- Defines what to send to the output.
- Fully CCW: Random analogue voltages via the Grains filtered PWM output.
             I'd hoped this would be usable for pitch but it's not stable enough.
             It works ok when fed into modulation inputs so if you just want
             envelope decay to switch to a new random value on every clock pulse
             or similar then this has got you covered.
- Left of 12-o-clock: Triggers.
- Right of 12-o-clock: Gates.

# Input 2 / Middle pot

- Probability that a division will actually trigger.
- Fully counter-clockwise nothing will be sent to the output.
- Fully clockwise every division will be sent to the output.

# Input 3

- Clock input.
- Triggers or gates should both work.
- Only tested with the clock from the Arturia Keystep but can confirm
  it works for me.

# Lower pot

- Size of the resulting divisions relative to the incoming pulse.
- From fully CCW to fully CW: 16, 8, 4, 2, 1.

# Gotchas

The main gotcha is that something about the clock output doesn't play
nicely triggering envelopes - it kinda works but not always. If you
buffer it through something else it's fine. I don't know what's going
on there but I'm guessing it's related to the output being low-pass
filtered?

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
byte last_clock_factor = 1;
byte skip = last_clock_factor;

byte clock_factor = 1;
int gate = 0;
long current_time = millis ();
int trigger_length = TRIGGER_LENGTH;


int gate_length = TRIGGER_LENGTH;
int interval = 0;
boolean out_high = false;

void setup() {
  randomSeed(analogRead(RND_PIN));
  analogWrite(PWM_PIN, 0);
}

long get_time() {
  return millis();
}

void trigger() {
  if (!should_trigger()) {
    return;
  }
  out_high = true;
  analogWrite(PWM_PIN, 255);
}

// Gate length as a percentage of the current clock interval 
int get_gate_size() {
  return map(analogRead(UPPER_POT), 0, UPPER_POT_MAX, 1, 100);
}

byte get_clock_factor() {
  switch(map(analogRead(LOWER_POT), 0, LOWER_POT_MAX, 0, 4)) {
    case 0:
      return 16;
    case 1:
      return 8;
    case 2:
      return 4;
    case 3:
      return 2;
    case 4:
      return 1;
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
  clock_factor = get_clock_factor();
  gate = analogRead(CLOCK_IN);
  current_time = get_time();

  // Determine whether any outgoing gate/trigger needs to be zero'd
  if (out_high && (current_time - last_trigger_out) > gate_length) {
    out_high = false;
    analogWrite(PWM_PIN, 0);
  }

  // Update the state based on incoming clock
  if (gate > GATE_THRESHOLD) {
    if (!in_clock_high) {
      // Rising edge
      skip -= 1;
      if (skip == 0 || last_clock_factor != clock_factor) {
        skip = clock_factor;
        // Maybe trigger
        trigger();
        if (last_trigger_out > 0) {
          gate_length = (current_time - last_trigger_out) * (get_gate_size() / 100);
        }
        last_trigger_out = current_time;
      }
      in_clock_high = true;
    }
  } else {
    if (in_clock_high) {
      // Falling edge
      in_clock_high = false;
    }
  }
  last_clock_factor = clock_factor;
}

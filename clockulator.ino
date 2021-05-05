/*
Alternate firmware for the ginky synthese grains eurorack module
Derived from code by a773 (atte.dk) and released under the GPL licence
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

byte get_clock_factor() {
  switch(map(analogRead(UPPER_POT), 0, UPPER_POT_MAX, 0, 4)) {
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
  mode current_mode = get_mode();
  clock_factor = get_clock_factor();
  gate = analogRead(CLOCK_IN);
  current_time = get_time();
  trigger_length = current_mode == mode_gate ? gate_length : TRIGGER_LENGTH;

  // Determine whether any outgoing gate/trigger needs to be zero'd
  if (out_high && current_mode != mode_random_cv && (current_time - last_trigger_out) > trigger_length) {
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
        trigger(current_mode);
        if (last_trigger_out > 0) {
          gate_length = (current_time - last_trigger_out) / 2;
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

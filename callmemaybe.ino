/*
CV_IN_{1,2,3,4} min 0, mid 410, max 820

Derived from:
 * https://github.com/elkayem/midi2cv/blob/master/midi2cv.ino
 * https://github.com/attejensen/a773_grains/blob/master/mult_div/mult_div.ino
 * Example firmware for the Ginkosynthese Tool at https://www.ginkosynthese.com/product/586905/tool-diy-kit

Totally GPL v3.

*/

#include <SPI.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>

#define CV_CCW     0
#define CV_MID     410
#define CV_CW      820

#define CV_IN_1    A0
#define CV_IN_2    A1
//#define CV_IN_3    A2
//#define CV_IN_4    A3
#define CLK_IN_1   A4
//#define CLK_IN_2   A5
#define RND_PIN    A6
#define MOSI       11
#define DAC         8
#define LED         6
#define CLK_OUT_1   2
//#define CLK_OUT_2   3
//#define CLK_OUT_3   4
//#define CLK_OUT_4   5

#define GATE_THRESHOLD 32
#define SEQ_MAX_LENGTH 8
#define MAX_V_OUT 4095

#define DEBUG false

#define TRIANGLE 0
#define SAWTOOTH 1
#define RESET_INC 128

int gate = 0;
int last_gate = 0;
int cv_in_1 = 0;
int cv_in_2 = 0;
int delay_ms = 410;
int seq_rand[SEQ_MAX_LENGTH];
unsigned int seq_ptr = 0;
unsigned int seq_length = SEQ_MAX_LENGTH + 1;
bool cycle_mode = false;
unsigned int cycle_ptr = 0;

int v_out = 0;

volatile int lfo_shape = TRIANGLE;
volatile byte lfo_inc = 0;
volatile bool lfo_rising = true;
volatile bool lfo_active = false;
volatile bool lfo_reset = false;
volatile bool lfo_reset_request = false;

volatile int v_out_2 = 0;
volatile int next_v_out_2 = 0;
volatile int v_reset = 0;

bool clk_out_1_high = false;
volatile bool clk_out_1_high_request = false;
volatile bool clk_out_1_low_request = false;
volatile bool loop_lfo = true;

void setup() {
  // Interrupt magic from multi-tool firmware
  cli();                                // disable global interrupts
  TCCR2A = 0;                           // set entire TCCR1A register to 0
  TCCR2B = 0;                           // same for TCCR1B
  TCCR2B =  0b00000011;                 // scaling to a 256th of the CPU freq = 31250 Hz
  // enable Timer1 overflow interrupt:
  TIMSK2 |= (1 << TOIE1);               // interupt enable
  TCNT2 = 0;                            // timer pre-loading
  sei();                                // re-enable global interupts
  // End of interrupt magic

  if (DEBUG) {
    Serial.begin(9600);
  }
  
  randomSeed(analogRead(RND_PIN));
  
  pinMode(DAC, OUTPUT);
  pinMode(CLK_OUT_1, OUTPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(DAC, HIGH);
  digitalWrite(CLK_OUT_1, LOW);
  digitalWrite(LED, HIGH);
  SPI.begin();
}

void loop() {
  last_gate = gate;
  gate = analogRead(CLK_IN_1);
  
  cv_in_2 = map(analogRead(CV_IN_2), 0, CV_CW, 8, 1024);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (cv_in_2 >= 512) {
      cv_in_2 = cv_in_2 - 512;
      lfo_shape = SAWTOOTH;
    } else {
      lfo_shape = TRIANGLE;
    }
    lfo_inc = cv_in_2 >> 2;

    if (clk_out_1_high_request && !clk_out_1_high) {
      digitalWrite(CLK_OUT_1, HIGH);
      clk_out_1_high = true;
      clk_out_1_high_request = false;
    }
    if (clk_out_1_low_request && clk_out_1_high) {
      digitalWrite(CLK_OUT_1, LOW);
      clk_out_1_high = false;
      clk_out_1_low_request = false;
    }
  }
  // Detect rising edge
  if (is_rising_edge(last_gate, gate)) {
    //analogWrite(LED, 255);
    // Trigger LFO
    // If it's already active we need to send a clock too
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  
      if (lfo_active) {
        digitalWrite(CLK_OUT_1, HIGH);
        clk_out_1_high = true;
      }
      lfo_active = true;
      lfo_rising = true;
      lfo_reset_request = true;
    }
    
    seq_length = map(analogRead(CV_IN_1), 0, CV_CW, SEQ_MAX_LENGTH, 1);

    if (cycle_mode == false && seq_length < SEQ_MAX_LENGTH) {
      // State: Transitioning to cycle
      cycle_ptr = seq_ptr;
      cycle_mode = true;
    } else if (cycle_mode == true &&
        seq_length >= SEQ_MAX_LENGTH &&
        cycle_ptr == seq_ptr) {
      // State: Transitioning out of cycle
      cycle_mode = false;
    }

    if (cycle_mode) {
      // State: In cycle
      v_out = seq_rand[seq_ptr % SEQ_MAX_LENGTH];
      if (seq_ptr == cycle_ptr) {
        // Beginning of cycle - could send a clock out here
        seq_ptr -= seq_length;
      } else {
        seq_ptr++;
      }
    } else {
      // State: Out of cycle
      v_out = random(MAX_V_OUT);
      seq_rand[seq_ptr % SEQ_MAX_LENGTH] = v_out;
      seq_ptr++;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {  
      set_voltage(v_out, 0, false);
    }
  } else if (is_falling_edge(last_gate, gate)) {
    // Nothing to do right now
    //analogWrite(LED, 0);
  }
  if (DEBUG) {
    print_debug();
  }
}

void set_voltage(unsigned int v, int dac, bool led) {
  setVoltage(DAC, dac, 1, v);
  if (led) {
    analogWrite(LED, map(v, 0, v_out, 0, 255));
  }
}

boolean is_high(int gate) {
  return gate > GATE_THRESHOLD;
}

boolean is_low(int gate) {
  return gate < GATE_THRESHOLD;
}

boolean is_rising_edge(int last_gate, int gate) {
  return (is_low(last_gate) && is_high(gate));
}

boolean is_falling_edge(int last_gate, int gate) {
  return (is_high(last_gate) && is_low(gate));
}

// ** This code was lifted straight from elkayem's midicv project **
//
// setVoltage -- Set DAC voltage output
// dacpin: chip select pin for DAC.  Note and velocity on DAC1, pitch bend and CC on DAC2
// channel: 0 (A) or 1 (B).  Volt/octave channel 0, 0-5v channel 1.
// gain: 0 = 1X, 1 = 2X.  
// mV: integer 0 to 4095.  If gain is 1X, mV is in units of half mV (i.e., 0 to 2048 mV).
// If gain is 2X, mV is in units of mV
void setVoltage(int dacpin, bool channel, bool gain, unsigned int mV)
{
  unsigned int command = channel ? 0x9000 : 0x1000;

  command |= gain ? 0x0000 : 0x2000;
  command |= (mV & 0x0FFF);
  
  SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));
  digitalWrite(dacpin,LOW);
  SPI.transfer(command>>8);
  SPI.transfer(command&0xFF);
  digitalWrite(dacpin,HIGH);
  SPI.endTransaction();
}

void print_debug() {
  Serial.print("cv_in_1 = ");
  Serial.println(cv_in_1);
}

/*
 * Run an LFO on output 2.
 * Eventually it will be possible to select loop/one-shot via a front panel switch.
 * All variables are declared volatile.
 * Might be worth getting rid of the call to millis() and instead
 * tracking that in the main loop.
 */
ISR(TIMER2_OVF_vect)
{
  next_v_out_2 = 0;
  if (lfo_reset_request && !lfo_reset) {
    lfo_reset = true;
    lfo_reset_request = false;
    v_reset = v_out_2;
    v_out_2 = 0;
  }
  if (lfo_active) {
    if (lfo_rising) {
      if (lfo_shape == TRIANGLE) {
        next_v_out_2 = v_out_2 + lfo_inc;
      } else if (lfo_shape == SAWTOOTH) {
        next_v_out_2 = v_out_2 + RESET_INC;
      }
      if (next_v_out_2 > MAX_V_OUT) {
        // End of LFO rising cycle
        clk_out_1_low_request = true;
        next_v_out_2 = v_out_2 - lfo_inc;
        lfo_rising = false;
      }
    } else {
      next_v_out_2 = v_out_2 - lfo_inc;
      if (next_v_out_2 < 0) {
        // End of LFO cycle
        clk_out_1_high_request = true;

        if (loop_lfo) {
          next_v_out_2 = v_out_2 + lfo_inc;
        } else {
          next_v_out_2 = 0;
          lfo_active = false;
        }
        lfo_rising = true;
      }
    }
    if (lfo_reset) {
      if (v_reset - RESET_INC <= v_out_2) {
        lfo_reset = false;
      } else {
        v_reset -= RESET_INC;
        set_voltage(v_reset, 1, true);
      }
    }
    v_out_2 = next_v_out_2;
    if (!lfo_reset) {
      set_voltage(v_out_2, 1, true);
    }
  }
}

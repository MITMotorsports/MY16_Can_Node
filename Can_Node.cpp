#include <Arduino.h>

#include "Can_Node.h"

#include "Can_Controller.h"
#include "Pins.h"

const uint8_t ANALOG_MESSAGE_ID = 0x01;
const uint8_t ANALOG_MESSAGE_PERIOD = 100;

const uint8_t RPM_MESSAGE_ID = 0x10;
const uint8_t RPM_MESSAGE_PERIOD = 100;

const uint16_t STARBOARD_THROTTLE_LOWER_BOUND = 246;
const uint16_t STARBOARD_THROTTLE_UPPER_BOUND = 885;
const uint16_t PORT_THROTTLE_LOWER_BOUND = 246;
const uint16_t PORT_THROTTLE_UPPER_BOUND = 879;

const uint16_t BRAKE_LOWER_BOUND = 104;
const uint16_t BRAKE_UPPER_BOUND = 900;

const uint16_t STEERING_POT_RIGHT_BOUND = 275;
const uint16_t STEERING_POT_LEFT_BOUND = 960;

volatile uint32_t sboard_curr = 0;
volatile uint32_t port_curr = 0;
volatile uint32_t sboard_old = 0;
volatile uint32_t port_old = 0;

volatile uint32_t sboard_rpm = 0;
volatile uint32_t port_rpm = 0;

Task sendAnalogCanMessageTask(ANALOG_MESSAGE_PERIOD, sendAnalogCanMessage);
Task sendRpmCanMessageTask(RPM_MESSAGE_PERIOD, sendRpmCanMessage);

// Implementation
void setup() {
  Serial.begin(115200);
  CAN().begin();

  pinMode(PORT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(STARBOARD_ENCODER_PIN, INPUT_PULLUP);

  pinMode(STARBOARD_THROTTLE_PIN, INPUT);
  pinMode(PORT_THROTTLE_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);
  pinMode(STEERING_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(STARBOARD_ENCODER_PIN), sboard_click, RISING);
  attachInterrupt(digitalPinToInterrupt(PORT_ENCODER_PIN), port_click, RISING);

  //disable interrupts
  cli();

  //Link to datasheet with most of the magic numbers on it
  //http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_datasheet.pdf

  //enable the timer2 - 8bit timer
  TIMSK2 |= 1<<TOIE2;
  //set the count to 0
  TCNT2 = 0;
  //CA22 is timer 2 prescalar of 1024
  //CS20 enables timer2
  TCCR2B |= (1<<CS22) | (1<<CS20);

  //enable interrupts
  sei();

  SoftTimer.add(&sendAnalogCanMessageTask);
  SoftTimer.add(&sendRpmCanMessageTask);

  Serial.println("Started Can Node");
}

void sboard_click() {
  sboard_old = sboard_curr;
  sboard_curr = micros();
  // Serial.println("right");
}

void port_click() {
  port_old = port_curr;
  port_curr = micros();

  // Serial.println("left");
}


//interrupt routine for timer2 overflow.
//timer 2 is an 8bit timer with a 1024 scaling
//Will run at 8MHz/256/1024 so ~30Hz
//also am only pretty sure of 8MHz, if it's in fact 16 just some constant changing.
//TIMER2_OVF_vect is triggered every time timer2 overflows
ISR(TIMER2_OVF_vect) {

  // Micros timer has a resolution of 8 microseconds error:
  // this is safe because smallest reasonable time between clicks is 1 millisecond.

  // Micros timer overflows at 2^32 microseconds = 4000 seconds
  //timer1 is at 125,000 Hz (8 MHz / 64) so one unit is 8*10^-6
  //one minute is 7,500,000 timer units
  //8 MHz / 64 * 60 = MAGIC because that's the number of timer clicks per minute

  // including stdint.h doesn't give this for some reason so we def it here
  #define UINT32_MAX 0xFFFFFFFF
  // 1000000 mics in a second, 60 secs in a min.
  // #define MICROS_PER_MIN 60000000
  // number of teeth.
  // #define CLICKS_PER_REV 23

  // Precomputed MICROS_PER_MIN / CLICKS_PER_REV
  #define SHORTCUT 2608696

  uint32_t local_sboard_curr = sboard_curr;
  uint32_t local_port_curr = port_curr;
  uint32_t local_sboard_old = sboard_old;
  uint32_t local_port_old = port_old;

  //allow this interrupt to be interrupted
  sei();

  uint32_t sboard_micros_per_click = local_sboard_curr - local_sboard_old;
  uint32_t port_micros_per_click = local_port_curr - local_port_old;

  if (local_sboard_old > local_sboard_curr) {
    // Micros has overflowed so this is safe from overflow
    sboard_micros_per_click = (UINT32_MAX - local_sboard_old) + local_sboard_curr;
  }
  if (local_port_old > local_port_curr) {
    // Micros has overflowed so this is safe from overflow
    port_micros_per_click = (UINT32_MAX - local_port_old) + local_port_curr;
  }

  // Convert clicks into RPM.
  //   (Micros / min) * (clicks / micro) * (revs / click)
  //     -> micros_per_min / micros_per_click / clicks_per_rev

  if (sboard_micros_per_click != 0) {
    sboard_rpm = SHORTCUT / sboard_micros_per_click;
  }
  if (port_micros_per_click != 0) {
    port_rpm = SHORTCUT / port_micros_per_click;
  }

  #undef MICROS_PER_MIN
  #undef CLICKS_PER_REV
  #undef UINT32_MAX
}

uint8_t readingToCan(uint32_t reading, const uint16_t lower_bound, const uint16_t upper_bound) {
  // Ensure reading is within expected range
  reading = max(reading, lower_bound);
  reading = min(reading, upper_bound);

  // Make reading between 0 and diff
  const uint16_t diff = upper_bound - lower_bound;
  reading = reading - lower_bound;

  // Now scale from [0:diff] to [0:255].
  // Note: it's critical that reading be a 32 bit int because otherwise this line will cause overflow!
  reading = reading * 255;
  reading = reading / diff;

  // Finally do some bounding for paranoia.
  // This is probably a no-op but it's a cheap one so why not.
  reading = min(reading, 255);
  reading = max(reading, 0);

  // This cast is safe because we have asserted reading is in range [0:255] in previous lines
  uint8_t short_val = (uint8_t)(reading);
  return short_val;
}

void sendRpmCanMessage(Task*) {
  cli();
  const uint16_t curr_starboard_rpm = sboard_rpm;
  const uint16_t curr_port_rpm = port_rpm;
  sei();

  Frame message = {.id=RPM_MESSAGE_ID, .body={lowByte(curr_starboard_rpm), highByte(curr_starboard_rpm), lowByte(curr_port_rpm), highByte(curr_port_rpm)}, .len=4};
  CAN().write(message);
}

void sendAnalogCanMessage(Task*) {
  const int16_t starboard_throttle_raw = analogRead(STARBOARD_THROTTLE_PIN);
  const int16_t port_throttle_raw = analogRead(PORT_THROTTLE_PIN);

  const int16_t brake_raw = analogRead(BRAKE_PIN);
  const int16_t steering_raw = analogRead(STEERING_PIN);

  // Serial.print("throttle_right_raw: ");
  // Serial.print(starboard_throttle_raw);
  // Serial.print(", throttle_left_raw: ");
  // Serial.print(port_throttle_raw);
  // Serial.print(", brake_raw: ");
  // Serial.print(brake_raw);
  // Serial.print(", steering_raw: ");
  // Serial.println(steering_raw);

  const uint8_t starboard_throttle_scaled = readingToCan(
    starboard_throttle_raw,
    STARBOARD_THROTTLE_LOWER_BOUND,
    STARBOARD_THROTTLE_UPPER_BOUND
  );

  const uint8_t port_throttle_scaled = readingToCan(
    port_throttle_raw,
    PORT_THROTTLE_LOWER_BOUND,
    PORT_THROTTLE_UPPER_BOUND
  );

  const uint8_t brake_scaled = readingToCan(
    brake_raw,
    BRAKE_LOWER_BOUND,
    BRAKE_UPPER_BOUND
  );

  const uint8_t steering_scaled = readingToCan(
    steering_raw,
    STEERING_POT_RIGHT_BOUND,
    STEERING_POT_LEFT_BOUND
  );

  // Serial.print("throttle_right: ");
  // Serial.print(starboard_throttle_scaled);
  // Serial.print(", throttle_left: ");
  // Serial.print(port_throttle_scaled);
  // Serial.print(", brake: ");
  // Serial.print(brake_scaled);
  // Serial.print(", steering: ");
  // Serial.println(steering_scaled);

  Frame message = {.id=ANALOG_MESSAGE_ID, .body={starboard_throttle_scaled, port_throttle_scaled, brake_scaled, steering_scaled}, .len=4};
  CAN().write(message);
}

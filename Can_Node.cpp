#include <Arduino.h>

#include "Can_Node.h"

#include "Can_Controller.h"
#include "Pins.h"

const uint8_t ANALOG_MESSAGE_ID = 0x01;
const uint8_t ANALOG_MESSAGE_PERIOD = 100;
Task sendAnalogCanMessageTask(ANALOG_MESSAGE_PERIOD, sendAnalogCanMessage);

const uint8_t RPM_MESSAGE_ID = 0x10;
const uint8_t RPM_MESSAGE_PERIOD = 50;
Task sendRpmCanMessageTask(RPM_MESSAGE_PERIOD, sendRpmCanMessage);

const uint16_t STARBOARD_THROTTLE_LOWER_BOUND = 246;
const uint16_t STARBOARD_THROTTLE_UPPER_BOUND = 885;
const uint16_t PORT_THROTTLE_LOWER_BOUND = 246;
const uint16_t PORT_THROTTLE_UPPER_BOUND = 879;

const uint16_t BRAKE_LOWER_BOUND = 104;
const uint16_t BRAKE_UPPER_BOUND = 900;

const uint16_t STEERING_POT_RIGHT_BOUND = 275;
const uint16_t STEERING_POT_LEFT_BOUND = 960;

volatile uint16_t sboard_curr = 0;
volatile uint16_t port_curr = 0;
volatile uint16_t sboard_old = 0;
volatile uint16_t port_old = 0;

uint32_t sboard_rpm = 0;
uint32_t port_rpm = 0;

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

  //disable interrupts
  cli();

  attachInterrupt(digitalPinToInterrupt(STARBOARD_ENCODER_PIN), sboard_click, RISING);
  attachInterrupt(digitalPinToInterrupt(PORT_ENCODER_PIN), port_click, RISING);

  //Link to datasheet with most of the magic numbers on it
  //http://www.atmel.com/Images/Atmel-42735-8-bit-AVR-Microcontroller-ATmega328-328P_datasheet.pdf

  //enable the timer2 - 8bit timer
  TIMSK2 = 1<<TOIE2;
  //set the count to 0
  TCNT2 = 0;
  //CA22 is timer 2 prescalar of 1024
  //CS20 enables timer2
  TCCR2B = (1<<CS22) | (1<<CS20);

  //enable timer 1 - 16bit timer
  TIMSK1 = 1<<TOIE1;
  //set the count to 0
  TCNT1 = 0;
  //CS10 starts timer 1
  //CS11 is timer 1 prescalar of 64
  TCCR1B = (1<<CS10) | (1<<CS11);

  //enable interrupts
  sei();

  SoftTimer.add(&sendAnalogCanMessageTask);
  SoftTimer.add(&sendRpmCanMessageTask);

  Serial.println("Started Can Node");

}

void sboard_click()
{
  sboard_old = sboard_curr;
  sboard_curr = TCNT1;
}

void port_click()
{
  port_old = port_curr;
  port_curr= TCNT1;
}

//interrupt routine for timer2 overflow.
//timer 2 is an 8bit timer with a 1024 scaling
//Will run at 8MHz/256/1024 so ~30Hz
//also am only pretty sure of 8MHz, if it's in fact 16 just some constant changing.
//TIMER2_OVF_vect is triggered every time timer2 overflows
ISR(TIMER2_OVF_vect)
{
  //timer1 is at 125,000 Hz (8 MHz / 64) so one unit is 8*10^-6
  //one minute is 7,500,000 timer units
  //8 MHz / 64 * 60 = MAGIC because that's the number of timer clicks per minute
  #define MAGIC 7500000
  //23 teeth on the wheel, 23 clicks per rotation, 1 semirotation is 1/23 rotation
  #define TEETH 23

  uint16_t local_sboard_curr = sboard_curr;
  uint16_t local_port_curr = port_curr;
  uint16_t local_sboard_old = sboard_old;
  uint16_t local_port_old = port_old;
  //allow this interrupt to be interrupted
  sei();
  //make it rarer that it requres more logic to handle
  TCNT1 = 0;
  //using longs is safer because we need signed and the ability to interpret > 16 bits
  int32_t sboard_delta = local_sboard_curr - local_sboard_old;
  int32_t port_delta = local_port_curr - local_port_old;
  //make sure that it's the absolute difference regardless of overflows
  //assuming 16bit integers. I'm pretty sure that's what this is.
  sboard_delta = sboard_delta >= 0 ? sboard_delta : (local_sboard_curr - (local_sboard_old - UINT_MAX));
  port_delta = port_delta >= 0 ? port_delta : (local_port_curr - (local_port_old - UINT_MAX));
  //converts clicks into RPM
  //proper math is that one click per tick times MAGIC (7,500,000) / teeth (23) will net RPM.
  uint32_t sboard_rpm = sboard_delta == 0 ? 0 : MAGIC / sboard_delta / TEETH;
  uint32_t port_rpm = port_delta == 0 ? 0 : MAGIC / port_delta / TEETH;

  Serial.print("port rpm:");
  Serial.println(port_rpm);
  Serial.print("starboard rpm:");
  Serial.println(sboard_rpm);

  #undef MAGIC
  #undef TEETH
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
  const uint16_t curr_starboard_rpm = sboard_rpm;
  const uint16_t curr_port_rpm = port_rpm;
  (void)curr_port_rpm;
  (void)curr_starboard_rpm;
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

  Frame message = {.id=1, .body={starboard_throttle_scaled, port_throttle_scaled, brake_scaled, steering_scaled}, .len=4};
  CAN().write(message);
}


#include <Arduino.h>

#include "Can_Node.h"

#include "Can_Controller.h"
#include "Pins.h"

const uint8_t ANALOG_MESSAGE_ID = 0x01;
const uint8_t ANALOG_MESSAGE_PERIOD = 100;

const uint16_t STARBOARD_THROTTLE_LOWER_BOUND = 246;
const uint16_t STARBOARD_THROTTLE_UPPER_BOUND = 885;
const uint16_t PORT_THROTTLE_LOWER_BOUND = 246;
const uint16_t PORT_THROTTLE_UPPER_BOUND = 879;

const uint16_t BRAKE_LOWER_BOUND = 104;
const uint16_t BRAKE_UPPER_BOUND = 900;

const uint16_t STEERING_POT_LEFT_BOUND = 0;
const uint16_t STEERING_POT_RIGHT_BOUND = 1023;

Task sendAnalogCanMessageTask(ANALOG_MESSAGE_PERIOD, sendAnalogCanMessage);

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

  SoftTimer.add(&sendAnalogCanMessageTask);

  Serial.println("Started Can Node");
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
  // Serial.println(brake_raw);

  Serial.print("steering_raw: ");
  Serial.println(steering_raw);

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

  // Serial.print("throttle_right: ");
  // Serial.print(starboard_throttle_scaled);
  // Serial.print(", throttle_left: ");
  // Serial.print(port_throttle_scaled);
  // Serial.print(", brake: ");
  // Serial.println(brake_scaled);

  Frame message = {.id=1, .body={starboard_throttle_scaled, port_throttle_scaled, brake_scaled, brake_scaled}, .len=4};
  CAN().write(message);
}

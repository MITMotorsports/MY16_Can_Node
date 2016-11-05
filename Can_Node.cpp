#include <Arduino.h>

#include "Can_Node.h"

#include "Can_Controller.h"
#include "Pins.h"

const unsigned char ANALOG_MESSAGE_ID = 0x01;
const int ANALOG_MESSAGE_PERIOD = 100;
const float STARBOARD_THROTTLE_SCALE = 0.4061;
const float PORT_THROTTLE_SCALE = STARBOARD_THROTTLE_SCALE;
const int STARBOARD_THROTTLE_OFFSET = -97;
const int PORT_THROTTLE_OFFSET = -97;

// We apply the scaling m followed by the offset b
// (so a raw offset of 100 is a scaled offset of 100m)
//
// not pushed = 105 raw = 0 in scaled world
// pushed normal regular = 250 = 50 in scaled world
// pushed normal foot = 400 raw = 100 in scaled world
// pushed super full = 900 raw = 255+ in scaled world
const float BRAKE_SCALE = 0.31;
const float BRAKE_OFFSET = -33;

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

int truncateToByte(int val) {
  val = min(val, 255);
  val = max(val, 0);
  return val;
}

unsigned char readingToCan(const int val, const float scale, const int offset=0) {
  const float scaled_val = ((float)val) * scale;
  const int offset_val = round(scaled_val) + offset;
  const int bounded_val = truncateToByte(offset_val);
  unsigned char short_val = (unsigned char)(bounded_val);
  return short_val;
}

void sendAnalogCanMessage(Task*) {
  const int starboard_throttle_raw = analogRead(STARBOARD_THROTTLE_PIN);
  const int port_throttle_raw = analogRead(PORT_THROTTLE_PIN);

  const int brake_raw = analogRead(BRAKE_PIN);

  Serial.print("throttle_right_raw: ");
  Serial.print(starboard_throttle_raw);
  Serial.print(", throttle_left_raw: ");
  Serial.print(port_throttle_raw);
  Serial.print(", brake_raw: ");
  Serial.println(brake_raw);

  const unsigned char starboard_throttle_scaled = readingToCan(
    starboard_throttle_raw,
    STARBOARD_THROTTLE_SCALE,
    STARBOARD_THROTTLE_OFFSET
  );
  const unsigned char port_throttle_scaled = readingToCan(
    port_throttle_raw,
    PORT_THROTTLE_SCALE,
    PORT_THROTTLE_OFFSET
  );

  const uint8_t brake_scaled = readingToCan(
    brake_raw,
    BRAKE_SCALE,
    BRAKE_OFFSET
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

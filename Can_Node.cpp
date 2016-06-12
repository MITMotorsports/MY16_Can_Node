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

const unsigned char RPM_MESSAGE_ID = 0x10;
const int RPM_MESSAGE_PERIOD = 100;
const int RPM_READING_PERIOD = 100;
const unsigned int MOVING_AVG_WIDTH = RPM_MESSAGE_PERIOD / RPM_READING_PERIOD;
const int CLICKS_PER_REVOLUTION = 22;
const float REVOLUTIONS_PER_CLICK = 1.0 / CLICKS_PER_REVOLUTION;
const unsigned long MICROS_PER_MIN = 60000000;
unsigned int portRpms[MOVING_AVG_WIDTH];
unsigned int starboardRpms[MOVING_AVG_WIDTH];
unsigned int rpmIndex = 0;
unsigned long lastRpmTime = 0;
volatile unsigned int starboardClicks = 0;
volatile unsigned int portClicks = 0;

Task recordRpmTask(RPM_READING_PERIOD, recordRpm);
Task sendRpmCanMessageTask(RPM_MESSAGE_PERIOD, sendRpmCanMessage);
Task sendAnalogCanMessageTask(ANALOG_MESSAGE_PERIOD, sendAnalogCanMessage);

void logStarboardEncoderClick() {
  starboardClicks++;
  Serial.print("Right click ");
  Serial.println(starboardClicks);
}

void logPortEncoderClick() {
  portClicks++;
  Serial.print("Left click ");
  Serial.println(portClicks);
}

void resetClicksAndTimer(const unsigned long curr) {
  lastRpmTime = curr;
  starboardClicks = 0;
  portClicks = 0;
}

// Implementation
void setup() {
  Serial.begin(115200);
  CAN().begin();

  pinMode(STARBOARD_THROTTLE_PIN, INPUT);
  pinMode(PORT_THROTTLE_PIN, INPUT);
  pinMode(BRAKE_PIN, INPUT);

  pinMode(PORT_ENCODER_PIN, INPUT_PULLUP);
  pinMode(STARBOARD_ENCODER_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PORT_ENCODER_PIN), logPortEncoderClick, CHANGE);
  attachInterrupt(digitalPinToInterrupt(STARBOARD_ENCODER_PIN), logStarboardEncoderClick, CHANGE);

  // SoftTimer.add(&recordRpmTask);
  SoftTimer.add(&sendAnalogCanMessageTask);

  resetClicksAndTimer(micros());
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

  uint8_t brake_implausible = 0;
  if (brake_raw < 50) {
    // Pull-down resistor must be active; call it implausible
    brake_implausible = 1;
  }
  const uint8_t brake_scaled = readingToCan(
    brake_raw,
    BRAKE_SCALE,
    BRAKE_OFFSET
  );

  Serial.print("throttle_right: ");
  Serial.print(starboard_throttle_scaled);
  Serial.print(", throttle_left: ");
  Serial.print(port_throttle_scaled);
  Serial.print(", brake: ");
  Serial.println(brake_scaled);

  Frame message = {.id=1, .body={starboard_throttle_scaled, port_throttle_scaled, brake_scaled, brake_scaled}, .len=4};
  CAN().write(message);
}

unsigned int toRpm(const unsigned long clicks, const unsigned long micros) {
  // IMPORTANT: don't change the order of these operations,
  // otherwise overflow might occur due to 32-bit resolution
  const float revs = clicks * REVOLUTIONS_PER_CLICK;
  const float revsPerMinute = (revs / micros) * MICROS_PER_MIN;
  return round(revsPerMinute);
}

void recordRpm(Task*) {
  return;
  // Cache values all at once for most accurate reading
  const unsigned long currTime = micros();
  const unsigned int cachedStarboardClicks = starboardClicks;
  const unsigned int cachedPortClicks = portClicks;

  // Once every ~70 minutes, micros() overflows back to zero.
  const bool timeOverflowed = currTime < lastRpmTime;

  // Go ahead and reset now so that interrupts can get back to work
  resetClicksAndTimer(currTime);

  if(timeOverflowed) {
    //Timer overflowed, do nothing this cycle
    return;
  }

  // Perform actual RPM calculations
  const unsigned long dt = currTime - lastRpmTime;
  const unsigned int starboardRpm = toRpm(cachedStarboardClicks, dt);
  const unsigned int portRpm = toRpm(cachedPortClicks, dt);

  // Record result, overwrite oldest existing record
  starboardRpms[rpmIndex] = starboardRpm;
  portRpms[rpmIndex] = portRpm;
  rpmIndex = (rpmIndex + 1) % MOVING_AVG_WIDTH;
}

void sendRpmCanMessage(Task*) {
  // Count total rpms
  unsigned long totStarboardRpm = 0;
  unsigned long totPortRpm = 0;
  for(unsigned int i = 0; i < MOVING_AVG_WIDTH; i++) {
    totStarboardRpm += starboardRpms[i];
    totPortRpm += portRpms[i];
  }

  // Average rpms
  unsigned int avgStarboardRpm = totStarboardRpm / MOVING_AVG_WIDTH;
  unsigned int avgPortRpm = totPortRpm / MOVING_AVG_WIDTH;

  // Generate and send message
  Frame rpmMessage = {
    .id=0x10,
    .body={
      highByte(avgStarboardRpm),
      lowByte(avgStarboardRpm),
      highByte(avgPortRpm),
      lowByte(avgPortRpm)
    },
    .len=4
  };
  // CAN().write(rpmMessage);
  (void)rpmMessage;
}

#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

const int STARBOARD_THROTTLE_PIN = A0;
const int PORT_THROTTLE_PIN = A1;

const int BRAKE_PIN = 17; //A3, PC3, PCINT11, pin_2-something

// TODO change once we know exactly which pins these go on
const int STARBOARD_ENCODER_PIN = 2;
const int PORT_ENCODER_PIN = 3;

#endif // PINS_H

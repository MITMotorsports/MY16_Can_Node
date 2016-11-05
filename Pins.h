#ifndef PINS_H
#define PINS_H

#include <Arduino.h>

const int STARBOARD_ENCODER_PIN = 2; //INT0, PD2, PCINT18, pin_4
const int PORT_ENCODER_PIN = 3; //INT1, PD3, PCINT19, pin_5

const int STARBOARD_THROTTLE_PIN = 14; //A0, PC0, PCINT8, pin_23
const int PORT_THROTTLE_PIN = 15; //A1, PC1, PCINT9, pin_24

const int BRAKE_PIN = 17; //A3, PC3, PCINT11, pin_26
const int STEERING_PIN = 18; // A4, PC4, PCINT12, pin_27

#endif // PINS_H

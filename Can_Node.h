#ifndef CAN_NODE_H
#define CAN_NODE_H

#include <SoftTimer.h>
#include <DelayRun.h>
#include <Debouncer.h>

#include <avr/interrupt.h>
#include <limits.h>
#include <stdint.h>

// Prototypes
uint8_t readingToCan(const uint32_t reading, const uint16_t lower_bound, const uint16_t upper_bound);

void sendAnalogCanMessage(Task*);
void sendRpmCanMessage(Task*);

void sboard_click();
void port_click();

#endif // CAN_NODE_H

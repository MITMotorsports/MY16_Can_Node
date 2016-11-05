#ifndef CAN_NODE_H
#define CAN_NODE_H

#include <SoftTimer.h>
#include <DelayRun.h>
#include <Debouncer.h>

// Prototypes
int truncateToByte(int val);
unsigned char readingToCan(const int val, const float scale, const int offset);

void sendAnalogCanMessage(Task*);

#endif // CAN_NODE_H

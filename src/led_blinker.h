#pragma once

#include <stdint.h>


static const  uint8_t blink_5[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 9, 0 };
static const  uint8_t blink_4[] = { 3, 3, 3, 3, 3, 3, 3, 9, 0, 0, 0 };
static const  uint8_t blink_3[] = { 3, 3, 3, 3, 3, 9, 0, 0, 0, 0, 0 };
static const  uint8_t blink_2[] = { 3, 3, 3, 9, 0, 0, 0, 0, 0, 0, 0 };
static const  uint8_t blink_1[] = { 3, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static const  uint8_t blink_working[] = { 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static const  uint8_t blink_start[] = { 15, 15, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
static const  uint8_t blink_stop[] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 };

class LedBlinker
{
public:

    static const int MaxPatternLength = 11;

    LedBlinker();

    // Set the pattern (array of tick counts, 0 terminates)
    void setPattern(const uint8_t newPattern[MaxPatternLength]);

    // Call this every 100ms
    void tick();

    // Override this method to control the hardware LED output
    void applyLed(bool on);

private:
    uint8_t pattern[MaxPatternLength];
    int currentIndex;
    int remainingTicks;
    bool ledOn;
};
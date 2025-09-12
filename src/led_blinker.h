#pragma once

#include <stdint.h>


static const  uint8_t blink_5[] = { 2, 3, 2, 3, 2, 3, 2, 3, 2, 9, 0 };
static const  uint8_t blink_4[] = { 2, 3, 2, 3, 2, 3, 2, 9, 0, 0, 0 };
static const  uint8_t blink_3[] = { 2, 3, 2, 3, 2, 9, 0, 0, 0, 0, 0 };
static const  uint8_t blink_2[] = { 2, 3, 2, 9, 0, 0, 0, 0, 0, 0, 0 };
static const  uint8_t blink_1[] = { 2, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
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
    void loadPattern(const uint8_t source[MaxPatternLength]);

    // Call this every 100ms
    void tick();

    // Override this method to control the hardware LED output
    void applyLed(bool on);

private:
    uint8_t pattern[MaxPatternLength];        // Active pattern
    uint8_t pendingPattern[MaxPatternLength]; // Pending pattern
    bool hasPendingPattern = false;

    int currentIndex;
    int remainingTicks;
    bool ledOn;
};
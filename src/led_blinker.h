#pragma once

#include <stdint.h>

static const  int8_t solid_start[] =   {  1, 0 };
static const  int8_t blink_1[] =       { -2, 9, 0 };
static const  int8_t blink_2[] =       { -2, 2, -2, 9, 0};
static const  int8_t blink_3[] =       { -2, 2, -2, 2, -2, 9, 0};
static const  int8_t blink_eager[] =   { -2, 2, 0 };
static const  int8_t blink_working[] = { -10, 10, 0 };
static const  int8_t blink_stop[] =    { -1, 1, 0 };

class LedBlinker
{
public:

    static const int MaxPatternLength = 11;

    LedBlinker();

    // Set the pattern (array of tick counts, 0 terminates)
    void setPattern(const int8_t* newPattern);
    void loadPattern(const int8_t* source);

    // Call this every 100ms
    void tick();

    // Override this method to control the hardware LED output
    void applyLed(bool on);

private:
    int8_t pattern[MaxPatternLength];        // Active pattern
    int8_t pendingPattern[MaxPatternLength]; // Pending pattern
    bool hasPendingPattern = false;

    int currentIndex;
    int remainingTicks;
    bool ledOn;
};

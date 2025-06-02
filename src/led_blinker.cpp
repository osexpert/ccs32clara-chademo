#include "led_blinker.h"

LedBlinker::LedBlinker()
{
    for (int i = 0; i < MaxPatternLength; ++i)
        pattern[i] = 0;

    currentIndex = 0;
    remainingTicks = 0;
    ledOn = false;
}

// Set the pattern (array of tick counts, 0 terminates)
void LedBlinker::setPattern(const uint8_t newPattern[MaxPatternLength])
{
    // Copy new pattern
    for (int i = 0; i < MaxPatternLength; ++i) {
        pattern[i] = newPattern[i];
    }

    // Default to index 0
    currentIndex = 0;

    // If LED is currently ON and pattern starts with ON (even index),
    // jump to last OFF index to force a visible blink
    if ((currentIndex % 2 == 0) && ledOn) {
        int lastValidIndex = MaxPatternLength - 1;
        while (lastValidIndex > 0 && pattern[lastValidIndex] == 0) {
            --lastValidIndex;
        }

        // Jump to last OFF phase if available
        if (lastValidIndex % 2 == 1) {
            currentIndex = lastValidIndex;
        }
    }

    remainingTicks = pattern[currentIndex];
    ledOn = (currentIndex % 2 == 0);
    applyLed(ledOn);
}

// Call this every 100ms
void LedBlinker::tick()
{
    if (remainingTicks > 1)
    {
        --remainingTicks;
    }
    else
    {
        ++currentIndex;
        if (currentIndex >= MaxPatternLength || pattern[currentIndex] == 0) {
            currentIndex = 0;
        }
        remainingTicks = pattern[currentIndex];
        ledOn = (currentIndex % 2 == 0);
        applyLed(ledOn);
    }
}

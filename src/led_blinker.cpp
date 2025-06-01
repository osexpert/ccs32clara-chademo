#include "led_blinker.h"

StatusIndicator::StatusIndicator()
{
    for (int i = 0; i < MaxPatternLength; ++i)
        pattern[i] = 0;

    currentIndex = 0;
    remainingTicks = 0;
    ledOn = false;
}

// Set the pattern (array of tick counts, 0 terminates)
void StatusIndicator::setPattern(const uint8_t newPattern[MaxPatternLength])
{
    for (int i = 0; i < MaxPatternLength; ++i) {
        pattern[i] = newPattern[i];
    }
    currentIndex = 0;
    remainingTicks = pattern[0];
    ledOn = (currentIndex % 2 == 0);
    applyLed(ledOn);
}

// Call this every 100ms
void StatusIndicator::tick()
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

#include "led_blinker.h"

LedBlinker::LedBlinker()
{
    for (int i = 0; i < MaxPatternLength; ++i) {
        pattern[i] = 0;
        pendingPattern[i] = 0;
    }

    currentIndex = 0;
    remainingTicks = 0;
    ledOn = false;
    hasPendingPattern = false;
}

void LedBlinker::setPattern(const uint8_t newPattern[MaxPatternLength])
{
    // Always overwrite the pending pattern
    for (int i = 0; i < MaxPatternLength; ++i)
        pendingPattern[i] = newPattern[i];

    hasPendingPattern = true;
}

void LedBlinker::loadPattern(const uint8_t source[MaxPatternLength])
{
    for (int i = 0; i < MaxPatternLength; ++i)
        pattern[i] = source[i];

    currentIndex = 0;
    remainingTicks = pattern[currentIndex];
    ledOn = (currentIndex % 2 == 0);
    applyLed(ledOn);
}

// Call this every 100ms
void LedBlinker::tick()
{
    if (remainingTicks > 1) {
        --remainingTicks;
    }
    else {
        ++currentIndex;

        if (currentIndex >= MaxPatternLength || pattern[currentIndex] == 0) {
            if (hasPendingPattern) {
                loadPattern(pendingPattern);
                hasPendingPattern = false;
                return;
            }

            currentIndex = 0;
        }

        remainingTicks = pattern[currentIndex];
        ledOn = (currentIndex % 2 == 0);
        applyLed(ledOn);
    }
}
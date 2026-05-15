#include "led_blinker.h"
//#include <cstdlib> // for abs()

static inline int8_t absi8(int8_t v) { return v < 0 ? -v : v; }

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

void LedBlinker::setPattern(const int8_t* newPattern)
{
    for (int i = 0; i < MaxPatternLength; ++i) {
        pendingPattern[i] = newPattern[i];
        if (newPattern[i] == 0) break;
    }

    hasPendingPattern = true;
}

void LedBlinker::loadPattern(const int8_t* source)
{
    for (int i = 0; i < MaxPatternLength; ++i) {
        pattern[i] = source[i];
        if (source[i] == 0) break;
    }

    currentIndex = 0;
    remainingTicks = absi8(pattern[currentIndex]);
    ledOn = (pattern[currentIndex] > 0);
    applyLed(ledOn);
}

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

        remainingTicks = absi8(pattern[currentIndex]);  // tick count is magnitude
        ledOn = (pattern[currentIndex] > 0);          // sign controls LED state
        applyLed(ledOn);
    }
}
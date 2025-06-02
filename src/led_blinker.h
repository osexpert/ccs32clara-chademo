#pragma once

#include <stdint.h>


static const  uint8_t blink_five[] = { 3, 3, 3, 3, 3, 3, 3, 3, 3, 9, 0 }; // 3x short ONs, 1s OFF
static const  uint8_t blink_four[] = { 3, 3, 3, 3, 3, 3, 3, 9, 0, 0, 0 }; // 3x short ONs, 1s OFF
static const  uint8_t blink_three[] = { 3, 3, 3, 3, 3, 9, 0, 0, 0, 0, 0 }; // 3x short ONs, 1s OFF
static const  uint8_t blink_two[] = { 3, 3, 3, 9, 0, 0, 0, 0, 0, 0, 0 }; // 3x short ONs, 1s OFF
static const  uint8_t blink_one[] = { 3, 9, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 3x short ONs, 1s OFF
static const  uint8_t blink_working[] = { 5, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 3x short ONs, 1s OFF
static const  uint8_t blink_stop[] = { 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0 }; // 3x short ONs, 1s OFF

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
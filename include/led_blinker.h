#pragma once

#include <stdint.h>

class LedBlinker {
public:
    LedBlinker(uint32_t onDurationMs, uint32_t offDurationMs);

    void setOnOffDuration(uint32_t on_durationMs, uint32_t off_durationMs);

    // Call this with milliseconds elapsed since last tick
    void tick(uint32_t ms);

    bool getState() const; // true = ON, false = OFF

private:
    uint32_t onDuration;
    uint32_t offDuration;
    uint32_t timeInState;
    bool isOn;

    void onLedChange(bool newState);
};
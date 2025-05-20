#include "led_blinker.h"


LedBlinker::LedBlinker(uint32_t onDurationMs, uint32_t offDurationMs)
    : onDuration(onDurationMs), offDuration(offDurationMs),
    timeInState(0), isOn(false) {
}

void LedBlinker::setOnOffDuration(uint32_t on_durationMs, uint32_t off_durationMs) {
    onDuration = on_durationMs;
    offDuration = off_durationMs;
}


void LedBlinker::tick(uint32_t ms) {
    timeInState += ms;

    if (isOn && timeInState >= onDuration) {
        isOn = false;
        timeInState = 0;
        onLedChange(isOn);
    }
    else if (!isOn && timeInState >= offDuration) {
        isOn = true;
        timeInState = 0;
        onLedChange(isOn);
    }
}

bool LedBlinker::getState() const {
    return isOn;
}


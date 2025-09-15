#pragma once

#include <stdbool.h>
#include <stdint.h>

// Call once at startup. If button is pressed, mode select will activate.
void special_modes_init(bool button_pressed);

// Call every 100 ms from your handler.
void special_modes_tick_100ms(bool button_state);

// Returns true while still in mode-select.
bool special_modes_selection_pending(void);

enum SpecialMode
{
	None = 0,
	Discharge = 1,
	LongerPrecharge = 2,
	Unwelding = 3
};

// Returns the selected mode number (press count), or 0 if none yet.
bool special_modes_is_selected(SpecialMode mode);

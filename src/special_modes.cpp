#include "special_modes.h"

extern void led_on(void);
extern void led_off(void);
extern void special_mode_selected(enum SpecialMode);

#define MODE_TIMEOUT_TICKS    100   // 10s
#define PRESS_GAP_TICKS       20    // 2s
#define FINAL_FLASH_OFF_TICKS 2     // 200ms
#define FINAL_FLASH_ON_TICKS  3     // 300ms

typedef enum {
    MODE_WAIT_PRESS,       // counting presses
    MODE_FINAL_FLASHES     // showing total presses
} mode_state_t;

static bool mode_active = false;
static mode_state_t mode_state = MODE_WAIT_PRESS;

static uint8_t press_count = 0;
static bool button_released = true;

static uint16_t idle_ticks = 0;
static uint16_t gap_ticks = 0;

// Flashing control
static uint8_t flashes_done = 0;
static uint8_t flash_phase = 0; // 0=OFF, 1=ON
static uint8_t flash_ticks = 0;

static uint8_t selected_mode = 0;

// ------------------------------------------------------

void special_modes_init(bool button_pressed)
{
    if (button_pressed) {
        mode_active = true;
        mode_state = MODE_WAIT_PRESS;
        press_count = 0;
        idle_ticks = 0;
        gap_ticks = 0;
        led_on();
        button_released = false; // wait for release before counting
    }
    else {
        mode_active = false;
        selected_mode = 0;
        led_off();
        button_released = true;
    }
}

void special_modes_tick_100ms(bool button)
{
    if (!mode_active) return;

    idle_ticks++;

    switch (mode_state) {

    case MODE_WAIT_PRESS:
        // Detect presses
        if (button) {
            if (button_released) {
                press_count++;
                gap_ticks = 0;
                idle_ticks = 0;
                button_released = false;
            }
        }
        else {
            button_released = true;
        }

        // Reflect negative button state on LED, but not until first press
        if (button && press_count > 0) {
            led_off();
        }
        else {
            led_on();
        }

        // Count inactivity to start final flash sequence
        if (press_count > 0) {
            gap_ticks++;
            if (gap_ticks >= PRESS_GAP_TICKS) {
                // Start final flashes
                mode_state = MODE_FINAL_FLASHES;
                flashes_done = 0;
                flash_ticks = 0;
                flash_phase = 0; // start OFF
                led_off();
            }
        }

        // Global timeout
        if (idle_ticks >= MODE_TIMEOUT_TICKS) {
            mode_active = false;
            selected_mode = 0;
            led_off();
        }
        break;

    case MODE_FINAL_FLASHES:
        flash_ticks++;
        if ((flash_phase == 0 && flash_ticks >= FINAL_FLASH_OFF_TICKS) ||
            (flash_phase == 1 && flash_ticks >= FINAL_FLASH_ON_TICKS)) {

            flash_ticks = 0;

            if (flash_phase == 0) {
                led_on();
                flash_phase = 1;
            }
            else {
                flashes_done++;
                if (flashes_done < press_count) {
                    led_off();
                    flash_phase = 0;
                }
                else {
                    // All flashes done => finalize
                    selected_mode = press_count;
                    press_count = 0; // reset for potentional reuse
                    mode_active = false;
                    led_off();
                    special_mode_selected((SpecialMode)selected_mode);
                }
            }
        }
        break;
    }
}

bool special_modes_selection_pending(void)
{
    return mode_active;
}

enum SpecialMode special_modes_get_selected(void)
{
    return (SpecialMode)selected_mode;
}

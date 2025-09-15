#include "special_modes.h"

extern void led_on(void);
extern void led_off(void);
extern void special_mode_selected(enum SpecialMode);

#define MODE_TIMEOUT_TICKS        100   // 10s
#define PRESS_GAP_TICKS           20    // 2s
#define FINAL_FLASH_OFF_TICKS     2     // 200ms
#define FINAL_FLASH_ON_TICKS      3     // 300ms
#define INTER_COMMAND_PAUSE_TICKS 6    // 0.6s between commands
#define LONG_PRESS_TICKS          6     // 0.6s hold = new command
#define MAX_COMMANDS              3

typedef enum {
    MODE_WAIT_PRESS,         // counting presses for current command
    MODE_FINAL_NEXT_COMMAND, // load next command for flashing
    MODE_FINAL_FLASHING,     // blink it out
    MODE_FINAL_INTER_PAUSE   // wait between commands
} mode_state_t;

static bool mode_active = false;
static mode_state_t mode_state = MODE_WAIT_PRESS;

static uint8_t press_count = 0;
static bool button_released = true;

static uint16_t idle_ticks = 0;
static uint16_t gap_ticks = 0;
static uint16_t press_ticks = 0;

static uint8_t command_list[MAX_COMMANDS];
static uint8_t command_index = 0;      // how many commands collected
static uint8_t current_command = 0;    // index of command being flashed

// Flashing control
static uint8_t flash_phase = 0; // 0=OFF, 1=ON
static uint8_t flash_ticks = 0;
static uint8_t current_flashes_remaining = 0;
static uint8_t inter_pause_ticks = 0; // << declared and used in pause state

// ------------------------------------------------------

void special_modes_init(bool button_pressed)
{
    if (button_pressed) {
        mode_active = true;
        mode_state = MODE_WAIT_PRESS;
        press_count = 0;
        idle_ticks = 0;
        gap_ticks = 0;
        press_ticks = 0;
        command_index = 0;
        current_command = 0;
        flash_phase = 0;
        flash_ticks = 0;
        current_flashes_remaining = 0;
        inter_pause_ticks = 0;
        led_on();
        button_released = false; // wait for release before counting
    }
    else {
        mode_active = false;
        command_index = 0;
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
        // Detect presses and long-press-stored-commands
        if (button) {
            if (button_released) {
                // new press edge
                press_count++;
                gap_ticks = 0;
                idle_ticks = 0;
                press_ticks = 0;
                button_released = false;
            }
            else {
                // button held
                press_ticks++;
                if (press_count > 0 && press_ticks >= LONG_PRESS_TICKS) {
                    // store previous command and prepare for next
                    if (command_index < MAX_COMMANDS) {
                        command_list[command_index++] = press_count - 1; // important: need to remove the initial press_count added in the if (button_released) first time around
                    }
                    press_count = 0;
                    gap_ticks = 0;
                    idle_ticks = 0;
                    press_ticks = 0;
                    led_on(); // indicate ready for next command
                }
            }
        }
        else {
            button_released = true;
            press_ticks = 0;
        }

        // Reflect button state on LED (only meaningful when press_count>0)
        if (press_count > 0) {
            if (button) led_off();
            else led_on();
        }

        // Inactivity ends current command & starts final sequence
        if (press_count > 0) {
            gap_ticks++;
            if (gap_ticks >= PRESS_GAP_TICKS) {
                if (command_index < MAX_COMMANDS) {
                    command_list[command_index++] = press_count;
                }
                press_count = 0;
                mode_state = MODE_FINAL_NEXT_COMMAND;
                current_command = 0;
            }
        }

        // Global timeout cancels everything
        if (idle_ticks >= MODE_TIMEOUT_TICKS) {
            mode_active = false;
            command_index = 0;
        }
        break;

    case MODE_FINAL_NEXT_COMMAND:
        if (current_command < command_index) {
            // prepare to flash this command
            current_flashes_remaining = command_list[current_command];
            flash_phase = 0;
            flash_ticks = 0;
            led_off();
            mode_state = MODE_FINAL_FLASHING;
        }
        else {
            // all done -> call callbacks in entered order
            mode_active = false;
            for (uint8_t i = 0; i < command_index; i++) {
                special_mode_selected((SpecialMode)command_list[i]);
            }
        }
        break;

    case MODE_FINAL_FLASHING:
        flash_ticks++;
        if ((flash_phase == 0 && flash_ticks >= FINAL_FLASH_OFF_TICKS) ||
            (flash_phase == 1 && flash_ticks >= FINAL_FLASH_ON_TICKS)) {

            flash_ticks = 0;

            if (flash_phase == 0) {
                // ON phase start
                led_on();
                flash_phase = 1;
            }
            else {
                // ON phase end -> count one blink done
                led_off();
                flash_phase = 0;

                if (current_flashes_remaining > 0) current_flashes_remaining--;

                if (current_flashes_remaining == 0) {
                    // finished this command
                    led_on();
                    current_command++;
                    if (current_command < command_index) {
                        // start pause before next command
                        inter_pause_ticks = 0;
                        mode_state = MODE_FINAL_INTER_PAUSE;
                    }
                    else {
                        // done with all commands -> finalize next iteration of MODE_FINAL_NEXT_COMMAND
                        mode_state = MODE_FINAL_NEXT_COMMAND;
                    }
                }
            }
        }
        break;

    case MODE_FINAL_INTER_PAUSE:
        inter_pause_ticks++;
        if (inter_pause_ticks >= INTER_COMMAND_PAUSE_TICKS) {
            mode_state = MODE_FINAL_NEXT_COMMAND;
        }
        break;
    }
}

bool special_modes_selection_pending(void)
{
    return mode_active;
}

bool special_modes_is_selected(enum SpecialMode m)
{
    for (uint8_t i = 0; i < command_index; i++) {
        if (command_list[i] == (uint8_t)m) return true;
    }
    return false;
}

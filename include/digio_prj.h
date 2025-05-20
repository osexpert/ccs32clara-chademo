#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    /* init:1, select: 0, not selected: 1 */ \
    DIG_IO_ENTRY(spi_cs_out,       GPIOB, 0x1000/*GPIO12*/,  PinMode::OUTPUT)   \
\
    /* init:1, 0 = clock low, 1 = clock high */ \
    DIG_IO_ENTRY(spi_clock_out,       GPIOB, 0x4000/*GPIO14*/,  PinMode::OUTPUT)   \
\
    /* init:1,  1 = bit set, 0 = bit not set */ \
    DIG_IO_ENTRY(spi_mosi_out,       GPIOD, 0x800/*GPIO11*/,  PinMode::OUTPUT)   \
\
    /* 1 = bit set */ \
    DIG_IO_ENTRY(spi_miso_in,       GPIOB, 0x200/*GPIO9*/,  PinMode::INPUT_PU)   \
\
    /* QCA7000’s IRQ_N (data ready to be read) */ \
    DIG_IO_ENTRY(spi_irq_n_in, GPIOB, 0x8000/*GPIO15*/, PinMode::INPUT_PU)   \
\
/*    DIG_IO_ENTRY(led_alive,     GPIOB, GPIO7,  PinMode::OUTPUT)   \ */ \
\
    /* init:1, after ChargeParameter_MaxV:0, BCB loop: 0101, right before stop charging:1. Conclusion: 0 = c_state, 1 = b_state */ \
    DIG_IO_ENTRY(state_c_out_inverted,  GPIOE, 0x8/*GPIO3*/,  PinMode::OUTPUT)   \
\
/*     DIG_IO_ENTRY(red_out,     GPIOB, GPIO2,  PinMode::OUTPUT)   \ */ \
/*    DIG_IO_ENTRY(green_out,   GPIOB, GPIO10, PinMode::OUTPUT)   \ */ \
/*    DIG_IO_ENTRY(blue_out,    GPIOB, GPIO11, PinMode::OUTPUT)   \ */ \
\
        /* init:0, after_precharge:1, unit_off:0 */ \
        DIG_IO_ENTRY(contactor_out, GPIOC, 0x10/*GPIO4*/,  PinMode::OUTPUT)   \
\
        /*    DIG_IO_ENTRY(trigger_wakeup,  GPIOB, GPIO0,  PinMode::OUTPUT)   \ */ \
\
/*    DIG_IO_ENTRY(keep_power_on,   GPIOB, GPIO1,  PinMode::OUTPUT)   \ */ \
\
        /* 0 = pressed, 1 = not pressed */ \
        DIG_IO_ENTRY(stop_button_in_inverted, GPIOD, 0x8/*GPIO3*/, PinMode::INPUT_PD)   \
\
        /* car switch(k) 0 = car ready, 1 = not ready. checked together with car CAR_STATUS_READY_TO_CHARGE being set */ \
        DIG_IO_ENTRY(switch_k_in_inverted, GPIOC, 0x400 /*GPIO10*/, PinMode::INPUT_PU)   \
\
        /* init to 1, set to 0 when ready, set to 1 at end of stop charging  */ \
        DIG_IO_ENTRY(switch_d2_out_inverted, GPIOC, 0x800 /*GPIO11*/, PinMode::OUTPUT)   \
\
        /* init to 1, off:0 (next last thing to be done, last is E0x6=1 aka.E0x4=E0x2=1). Guessing this is power on? */ \
        DIG_IO_ENTRY(power_on_out, GPIOB, 0x100 /*GPIO8*/, PinMode::OUTPUT)   \
\
        /* init:1 wdog: toggle, set to 0 in chademo loop together with C0x8=1, set to 0 in wdog loop if power delivery active?, power off:1
        Guess: 0 is LED ON 
        wdog make it blink every 0.5 seconds, UNLESS charging, then it is contant. TODO: the logic is flawed...because what is done in chademo is overwritten in wdog anyways...
        */ \
        DIG_IO_ENTRY(internal_led_out, GPIOE, 0x2/*GPIO1*/, PinMode::OUTPUT)   \
    \
        /* init:1 wdog: toggle, power off:1 
        Guess: 0 is LED ON. 
        power button led
        wdog make it blink every 0.5 seconds */ \
        DIG_IO_ENTRY(power_led_out, GPIOE, 0x4/*GPIO2*/, PinMode::OUTPUT)   \
    \
        /* init:0, set to 1 in chademo loop, together with charging led? after recieved car contactor closed. This does not correspond to anything in the spec...
        
        */ \
        DIG_IO_ENTRY(chademo_unknown_out, GPIOC, 0x8/*GPIO3*/, PinMode::OUTPUT)   \

#endif // PinMode_PRJ_H_INCLUDED

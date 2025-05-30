#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    /* init:1, select: 0, not selected: 1 */ \
    DIG_IO_ENTRY(spi_cs_out,       GPIOB, 0x1000/*GPIO12*/,  PinMode::OUTPUT, GPIO_OSPEED_100MHZ)   \
\
    /* init:1, 0 = clock low, 1 = clock high */ \
    DIG_IO_ENTRY(spi_clock_out,       GPIOB, 0x4000/*GPIO14*/,  PinMode::OUTPUT, GPIO_OSPEED_100MHZ)   \
\
    /* init:1,  1 = bit set, 0 = bit not set */ \
    DIG_IO_ENTRY(spi_mosi_out,       GPIOD, 0x800/*GPIO11*/,  PinMode::OUTPUT, GPIO_OSPEED_100MHZ)   \
\
    /* 1 = bit set */ \
    DIG_IO_ENTRY(spi_miso_in,       GPIOB, 0x200/*GPIO9*/,  PinMode::INPUT_PU, 0)   \
\
    /* QCA7000’s IRQ_N (data ready to be read) */ \
    DIG_IO_ENTRY(spi_irq_n_in, GPIOB, 0x8000/*GPIO15*/, PinMode::INPUT_PU, 0)   \
\
    /* init:1, after ChargeParameter_MaxV:0, BCB loop: 0101, right before stop charging:1. Conclusion: 0 = c_state, 1 = b_state */ \
    DIG_IO_ENTRY(state_c_out_inverted,  GPIOE, 0x8/*GPIO3*/,  PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
        /* init:0, after_precharge:1, unit_off:0 Don't know what it is....I am guessing it is leftovers from a contactor that was left out...
            OR....it could be a relay to enable charging from the high voltage transformer.....hard to say...  
            but measuring between ground and d1 seems like this is it. but why set this in precharge logic? */ \
        DIG_IO_ENTRY(switch_d1_out, GPIOC, 0x10/*GPIO4*/,  PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
        /* 0 = pressed, 1 = not pressed */ \
        DIG_IO_ENTRY(stop_button_in_inverted, GPIOD, 0x8/*GPIO3*/, PinMode::INPUT_PD, 0)   \
\
        /* car switch(k) 0 = car ready, 1 = not ready. checked together with car CAR_STATUS_READY_TO_CHARGE being set */ \
        DIG_IO_ENTRY(switch_k_in_inverted, GPIOC, 0x400 /*GPIO10*/, PinMode::INPUT_PU, 0) \
\
        /* init to 1, set to 0 when ready, set to 1 at end of stop charging  
        measured: it does something with d2 when measured with diode measurer againt ground.... its not a relay, but something else.
        */ \
        DIG_IO_ENTRY(switch_d2_out_inverted, GPIOC, 0x800 /*GPIO11*/, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
        /* init to 1, off:0 (next last thing to be done, last is both leds on). keep power on. */ \
        DIG_IO_ENTRY(power_on_out, GPIOB, 0x100 /*GPIO8*/, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
        /* init:1 (led off) wdog: toggle, set to 0 (on) in chademo loop together with contactor_out=1, set to 0 (on) in wdog loop if power delivery active?, power off:1 (led off) */ \
        DIG_IO_ENTRY(external_led_out_inverted, GPIOE, 0x2/*GPIO1*/, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
    \
        /* init:1 (led off) wdog: toggle, power off:1 (led off), always just blinking */ \
        DIG_IO_ENTRY(internal_led_out_inverted, GPIOE, 0x4/*GPIO2*/, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
    \
        /* init:0, set to 1 in chademo loop, together with charging led? after recieved car contactor closed. This does not correspond to anything in the spec... */ \
        DIG_IO_ENTRY(contactor_out, GPIOC, 0x8/*GPIO3*/, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \

#endif // PinMode_PRJ_H_INCLUDED

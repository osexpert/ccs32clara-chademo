#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

/* Here you specify generic IO pins, i.e. digital input or outputs.
 * Inputs can be floating (INPUT_FLT), have a 30k pull-up (INPUT_PU)
 * or pull-down (INPUT_PD) or be an output (OUTPUT)
*/

#define DIG_IO_LIST \
    /* init:1, select:0, not selected:1 */ \
    DIG_IO_ENTRY(spi_cs_out, GPIOB, GPIO12, PinMode::OUTPUT, GPIO_OSPEED_100MHZ)   \
\
    /* init:1, clock low:0, clock high:1 */ \
    DIG_IO_ENTRY(spi_clock_out, GPIOB, GPIO14, PinMode::OUTPUT, GPIO_OSPEED_100MHZ)   \
\
    /* init:1, bit set:1, bit not set:0 */ \
    DIG_IO_ENTRY(spi_mosi_out, GPIOD, GPIO11, PinMode::OUTPUT, GPIO_OSPEED_100MHZ)   \
\
    /* bit set:1 */ \
    DIG_IO_ENTRY(spi_miso_in, GPIOB, GPIO9, PinMode::INPUT_PU, 0)   \
\
    /* QCA7000’s IRQ_N (data ready to be read) */ \
    DIG_IO_ENTRY(spi_irq_n_in, GPIOB, GPIO15, PinMode::INPUT_PU, 0)   \
\
    /* init:1, c_state:0, b_state:1 */ \
    DIG_IO_ENTRY(state_c_out_inverted, GPIOE, GPIO3, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
    /* init:0, after_precharge:1, unit_off:0 */ \
    DIG_IO_ENTRY(switch_d1_out, GPIOC, GPIO4, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
    /* pressed:0, not pressed:1 */ \
    DIG_IO_ENTRY(stop_button_in_inverted, GPIOD, GPIO3, PinMode::INPUT_PD, 0)   \
\
    /* car ready:0, not ready:1 */ \
    DIG_IO_ENTRY(switch_k_in_inverted, GPIOC, GPIO10, PinMode::INPUT_PU, 0) \
\
    /* init:1, car ready:0, end of stop charging:1 */ \
    DIG_IO_ENTRY(switch_d2_out_inverted, GPIOC, GPIO11, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
    /* init:1, off:0 */ \
    DIG_IO_ENTRY(power_on_out, GPIOB, GPIO8, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
    /* init:1 (led off), power off:1 (led off) */ \
    DIG_IO_ENTRY(external_led_out_inverted, GPIOE, GPIO1, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
    /* init:1 (led off), power off:1 (led off) */ \
    DIG_IO_ENTRY(internal_led_out_inverted, GPIOE, GPIO2, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)   \
\
    /* init:0, after car contactor closed:1 */ \
    DIG_IO_ENTRY(contactor_out, GPIOC, GPIO3, PinMode::OUTPUT, GPIO_OSPEED_2MHZ)

#endif // PinMode_PRJ_H_INCLUDED

/*
 * This file is part of the stm32-template project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include "hwinit.h"
#include "my_string.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   //RCC_CLOCK_SETUP();
   rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

   //The reset value for PRIGROUP (=0) is not actually a defined
   //value. Explicitly set 16 preemtion priorities
   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_GPIOE);

   rcc_periph_clock_enable(RCC_ADC1);

   rcc_periph_clock_enable(RCC_CAN1);

//   rcc_periph_clock_enable(RCC_SPI1); //QCA comms
}

/*
 * systick_setup(void)
 *
 * This function sets up the 1khz "system tick" count. The SYSTICK counter is a
 * standard feature of the Cortex-M series.
 */
void systick_setup(void)
{
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(168000 - 1); // 1ms
    //systick_set_reload(1680000); // for 168 MHz core clock -> 10ms

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();
}



/**
* Enable Timer refresh and break interrupts
*/
//void nvic_setup(void)
//{
//   nvic_set_priority(NVIC_TIM4_IRQ, 0xe << 4); //second lowest priority
//   nvic_enable_irq(NVIC_TIM4_IRQ); //Scheduler
//
//
//   // no idea....
////   nvic_enable_irq(NVIC_DMA1_CHANNEL2_IRQ); //SPI RX complete
////   nvic_set_priority(NVIC_DMA1_CHANNEL2_IRQ, 0xd << 4); //third lowest priority
//}

//void rtc_setup()
//{
//   //Base clock is HSE/128 = 8MHz/128 = 62.5kHz
//   //62.5kHz / (624 + 1) = 100Hz
//   rtc_auto_awake(RCC_HSE, 624); //10ms tick
//   rtc_set_counter_val(0);
//}

extern volatile uint32_t system_millis;

/**
* returns the number of milliseconds since startup
*/
uint32_t rtc_get_ms(void) 
{
    //return rtc_get_counter_val()*10; /* The RTC has a 10ms tick (see rtc_setup()). So we multiply by 10 to get the milliseconds. */
    return ::system_millis;
}

void can_setup(void) {

    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_GPIOD);

    // Configure GPIOD pin 0 as AF9, Pull-Up, Very High Speed, Alternate function push-pull. Can RX.
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO0);
    gpio_set_af(GPIOD, GPIO_AF9, GPIO0);
//    gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);

    // Configure GPIOD pin 1 as AF9, No Pull, Very High Speed, Alternate function push-pull. Can TX.
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_set_af(GPIOD, GPIO_AF9, GPIO1);
  //  gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO1);


             //CAN1 RX and TX IRQs
    nvic_enable_irq(NVIC_CAN1_RX0_IRQ); //CAN RX
    nvic_set_priority(NVIC_CAN1_RX0_IRQ, 0xf << 4); //lowest priority
    nvic_enable_irq(NVIC_CAN1_RX1_IRQ); //CAN RX
    nvic_set_priority(NVIC_CAN1_RX1_IRQ, 0xf << 4); //lowest priority
    nvic_enable_irq(NVIC_CAN1_TX_IRQ); //CAN TX
    nvic_set_priority(NVIC_CAN1_TX_IRQ, 0xf << 4); //lowest priority


    can_reset(CAN1);

    // Configure CAN for 500 kbps assuming APB1 clock is 42 MHz
    can_init(CAN1,
        false,           // TTCM (Time Triggered Communication Mode)
        true,            // ABOM (Automatic Bus-Off Management)
        false,           // AWUM (Auto Wakeup Mode)
        false,           // NART (No Automatic Retransmission)
        false,           // RFLM (Receive FIFO Locked Mode)
        false,           // TXFP (Transmit FIFO Priority)
        CAN_BTR_SJW_1TQ,
        CAN_BTR_TS1_11TQ,
        CAN_BTR_TS2_2TQ,
        6, // Baudrate prescaler (for 500 kbps)
        false, // loopback
        false // silent
        );              

    // Accepts 0x100, 0x101, 0x102 — standard IDs, data frames
    can_filter_id_mask_32bit_init(
        0,                      // Filter bank 0
        (0x100 << 21),          // Match ID = 0x100, standard, data frame
        (0x7FC << 21),          // Mask to ignore bottom 2 bits of ID
        0,                      // Assign to FIFO 0
        true                    // Enable filter
    );

    can_enable_irq(CAN1, CAN_IER_FMPIE0);
    can_enable_irq(CAN1, CAN_IER_FMPIE1);

    // Don't use irq's. Polling seems to work fine and dropped messages should not matter.

    // Enable interrupts for FIFO 0 message received TODO currently polling
//    can_enable_interrupt(CAN1, CAN_IER_FMPIE0);

    // Enable CAN1 RX0 interrupt in the NVIC TODO currently polling
//    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
}

void usart1_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);

    // PA9 = USART1_TX
    gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_af(GPIOA, GPIO_AF7, GPIO9);

    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    usart_enable(USART1);
}
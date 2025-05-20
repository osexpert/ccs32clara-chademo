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
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/crc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/flash.h>
#include <libopencm3/stm32/desig.h>
#include "hwdefs.h"
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
uint32_t rtc_get_ms(void) {
    //return rtc_get_counter_val()*10; /* The RTC has a 10ms tick (see rtc_setup()). So we multiply by 10 to get the milliseconds. */
    return ::system_millis;
}




void can_setup(void) {

    // Enable CAN1 clock

    rcc_periph_clock_enable(RCC_CAN1);
    can_reset(CAN1);

    // CAN initialization with 500 kbps baud rate

    // Configure CAN for 500 kbps assuming APB1 clock is 42 MHz

    /*int can_init(uint32_t canport, bool ttcm, bool abom, bool awum, bool nart,
        bool rflm, bool txfp, uint32_t sjw, uint32_t ts1, uint32_t ts2,
        uint32_t brp, bool loopback, bool silent);*/

    can_init(CAN1,
        false,           // TTCM (Time Triggered Communication Mode)
        true,            // ABOM (Automatic Bus-Off Management)
        false,           // AWUM (Auto Wakeup Mode)
        false,           // NART (No Automatic Retransmission)
        false,           // RFLM (Receive FIFO Locked Mode)
        false,           // TXFP (Transmit FIFO Priority)
        CAN_BTR_SJW_1TQ,
        CAN_BTR_TS1_13TQ,
        CAN_BTR_TS2_2TQ,
        6, // Baudrate prescaler (for 500 kbps)
        false, // loopback
        false // silent?
        );              


    // Accepts 0x100, 0x101, 0x102 — standard IDs, data frames
    can_filter_id_mask_32bit_init(
        0,                      // Filter bank 0
        (0x100 << 21),          // Match ID = 0x100, standard, data frame
        (0x7FC << 21),          // Mask to ignore bottom 2 bits of ID
        0,                      // Assign to FIFO 0
        true                    // Enable filter
    );


    // Enable interrupts for FIFO 0 message received TODO currently polling
//    can_enable_interrupt(CAN1, CAN_IER_FMPIE0);

    // Enable CAN1 RX0 interrupt in the NVIC TODO currently polling
//    nvic_enable_irq(NVIC_CAN1_RX0_IRQ);
}

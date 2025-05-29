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
#include "main.h"
#include "my_string.h"

/**
* Start clocks of all needed peripherals
*/
void clock_setup(void)
{
   rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);

   //The reset value for PRIGROUP (=0) is not actually a definedvalue. Explicitly set 16 preemtion priorities
//   SCB_AIRCR = SCB_AIRCR_VECTKEY | SCB_AIRCR_PRIGROUP_GROUP16_NOSUB;
   //NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_4); // Equivalent to GROUP16_NOSUB
   // removed, dont know what it does...

   rcc_periph_clock_enable(RCC_GPIOA);
   rcc_periph_clock_enable(RCC_GPIOB);
   rcc_periph_clock_enable(RCC_GPIOC);
   rcc_periph_clock_enable(RCC_GPIOD);
   rcc_periph_clock_enable(RCC_GPIOE);

   rcc_periph_clock_enable(RCC_ADC1);

   rcc_periph_clock_enable(RCC_CAN1);
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
//    systick_set_reload(168000 - 1); // 1ms
    systick_set_reload(rcc_ahb_frequency / 1000 - 1); // 1ms
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
    return system_millis;
}

void can_setup(void) {

    rcc_periph_clock_enable(RCC_CAN1);
    rcc_periph_clock_enable(RCC_GPIOD);

    // Configure GPIOD pin 0 as AF9, Pull-Up, Very High Speed, Alternate function push-pull. Can RX.
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_PULLUP, GPIO0);
    gpio_set_af(GPIOD, GPIO_AF9, GPIO0);
    //gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO0);

    // Configure GPIOD pin 1 as AF9, No Pull, Very High Speed, Alternate function push-pull. Can TX.
    gpio_mode_setup(GPIOD, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO1);
    gpio_set_af(GPIOD, GPIO_AF9, GPIO1);
    // TODO: what is def speed??? figure out and print it
    gpio_set_output_options(GPIOD, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO1);


    nvic_set_priority(NVIC_CAN1_RX0_IRQ, IRQ_PRIORITY_CAN); //lowest priority
    nvic_enable_irq(NVIC_CAN1_RX0_IRQ); //CAN RX

//    nvic_enable_irq(NVIC_CAN1_TX_IRQ); //CAN TX
  //  nvic_set_priority(NVIC_CAN1_TX_IRQ, 0xf << 4); //lowest priority


    can_reset(CAN1);


//#define CAN_BTR_TS1_7TQ  (0x6 << 16)  // TS1 bits = 6 (TimeSeg1=7)
//#define CAN_BTR_TS2_6TQ  (0x5 << 20)  // TS2 bits = 5 (TimeSeg2=6)


    // Configure CAN for 500 kbps assuming APB1 clock is 42 MHz
    can_init(CAN1,
        false,           // TTCM (Time Triggered Communication Mode)
        true,            // ABOM (Automatic Bus-Off Management)
        false,           // AWUM (Auto Wakeup Mode)
        false,           // NART (No Automatic Retransmission)
        false,           // RFLM (Receive FIFO Locked Mode)
        false,           // TXFP (Transmit FIFO Priority)
        CAN_BTR_SJW_1TQ,
        CAN_BTR_TS1_7TQ,//        CAN_BTR_TS1_11TQ, // fff = 0x000a0000
        CAN_BTR_TS2_6TQ,//      CAN_BTR_TS2_2TQ, // 0x00100000
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

    // all
    //can_filter_id_mask_32bit_init(
    //    0, // Filter bank 0
    //    0x00000000,             // Match any ID, IDE=0, RTR=0
    //    0x00000006,             // Mask IDE and RTR bits (bits 3 and 2)
    //    0, // Assign to FIFO 0
    //    true // enable
    //);


    // all known
    //uint16_t ids[] = { 0x100, 0x101, 0x102, 0x110, 0x200, 0x201, 0x202, 0x700 };
    //int n = sizeof(ids) / sizeof(ids[0]);

    //for (int i = 0; i < n; i++) {
    //    can_filter_id_mask_32bit_init(
    //        i,                  // Filter bank number
    //        ((uint32_t)ids[i]) << 21, // ID shifted for 32-bit mode
    //        0x7FF << 21,        // Mask for exact ID match (11 bits)
    //        0,                  // FIFO 0
    //        true                // Enable filter
    //    );
    //}


    can_enable_irq(CAN1, CAN_IER_FMPIE0);
}

#if USART_DMA

#define TX_BUF_SIZE 200
static char tx_buf[TX_BUF_SIZE];
static volatile int tx_buf_pos = 0;
static volatile bool tx_dma_busy = false;

#endif

void usart1_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
#if USART_DMA
    rcc_periph_clock_enable(RCC_DMA2); // USART1 TX uses DMA2 Stream7 Channel4
#endif
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

#if USART_DMA
    // DMA setup
    dma_stream_reset(DMA2, DMA_STREAM7);
    dma_channel_select(DMA2, DMA_STREAM7, DMA_SxCR_CHSEL_4);

    dma_set_priority(DMA2, DMA_STREAM7, DMA_SxCR_PL_HIGH);
    dma_set_memory_size(DMA2, DMA_STREAM7, DMA_SxCR_MSIZE_8BIT);
    dma_set_peripheral_size(DMA2, DMA_STREAM7, DMA_SxCR_PSIZE_8BIT);
    dma_enable_memory_increment_mode(DMA2, DMA_STREAM7);
    //dma_set_transfer_direction(DMA2, DMA_STREAM7, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
    dma_set_transfer_mode(DMA2, DMA_STREAM7, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);

    // When using DMA to transmit data via USART, the DMA controller must know where to write each byte of data.
    // In this case, we're transmitting via USART1, and transmit data register for USART1 is USART1_DR.
    dma_set_peripheral_address(DMA2, DMA_STREAM7, (uint32_t)&USART1_DR);

    dma_enable_transfer_complete_interrupt(DMA2, DMA_STREAM7);

    nvic_set_priority(NVIC_DMA2_STREAM7_IRQ, IRQ_PRIORITY_LOG); // lowest priority
    nvic_enable_irq(NVIC_DMA2_STREAM7_IRQ);
#endif
}

#if USART_DMA

extern "C" void dma2_stream7_isr(void)
{
    if (dma_get_interrupt_flag(DMA2, DMA_STREAM7, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA2, DMA_STREAM7, DMA_TCIF);
        dma_disable_stream(DMA2, DMA_STREAM7);
        tx_dma_busy = false;

        // If active buffer has data, flush again
        //if (tx_buf_pos[tx_buf_active] > 0) {
        //    usart1_dma_flush();
        //}
    }
}

//#define TX_BUF_SIZE 250
//
//static char tx_buf[2][TX_BUF_SIZE];
//static volatile uint8_t tx_buf_active = 0;    // Index of buffer being written to
//static volatile uint8_t tx_dma_busy = false;
//static volatile size_t tx_buf_pos[2] = { 0 };

static void usart1_dma_flush(void)
{
    if (tx_dma_busy) return;

    uint8_t sending_buf = tx_buf_active;
    size_t len = tx_buf_pos[sending_buf];

    if (len == 0) return;

    // Switch to other buffer for continued logging
    tx_buf_active ^= 1;

    tx_dma_busy = true;
    tx_buf_pos[sending_buf] = 0;

    dma_disable_stream(DMA2, DMA_STREAM7);  // Just in case
    dma_set_memory_address(DMA2, DMA_STREAM7, (uint32_t)tx_buf[sending_buf]);
    dma_set_number_of_data(DMA2, DMA_STREAM7, len);
    dma_enable_stream(DMA2, DMA_STREAM7);
    usart_enable_tx_dma(USART1);
}

int usart1_dma_putchar(char c)
{
    uint8_t active = tx_buf_active;

    if (tx_buf_pos[active] < TX_BUF_SIZE) {
        tx_buf[active][tx_buf_pos[active]++] = c;
    }

    if (c == '\n' || tx_buf_pos[active] == TX_BUF_SIZE) {
        if (!tx_dma_busy) {
            usart1_dma_flush();
        }
    }

    return (int)c;
}
#endif

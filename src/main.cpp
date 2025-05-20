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
#include <stdint.h>
#include <stddef.h>

#include "libopencm3/cm3/cortex.h"
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/itm.h>
#include <libopencm3/cm3/dwt.h>
#include <libopencm3/cm3/scs.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/tpiu.h>
#include <libopencm3/stm32/dbgmcu.h>

#include <libopencmsis/core_cm3.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>

#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "my_string.h"

#include "configuration.h"
#include "connMgr.h"
#include "hardwareInterface.h"
#include "homeplug.h"
#include "ipv6.h"
#include "modemFinder.h"
#include "myHelpers.h"
#include "pevStateMachine.h"
#include "qca7000.h"
#include "tcp.h"
#include "udpChecksum.h"
#include "pushbutton.h"
#include "wakecontrol.h"
#include "chademoCharger.h"

#define __DSB()  __asm__ volatile ("dsb" ::: "memory")
#define __ISB()  __asm__ volatile ("isb" ::: "memory")
//#define __disable_irq() __asm__ volatile ("cpsid i")
//#define __enable_irq()  __asm__ volatile ("cpsie i")

#define PRINT_JSON 0

/* to solve linker warning, see https://openinverter.org/forum/viewtopic.php?p=64546#p64546 */
extern "C" void __cxa_pure_virtual()
{
    while (1);
}

//enum PowerOffReason
//{
//    StopButton
//};

static ChademoCharger* chademoCharger;

//static Stm32Scheduler* scheduler;

//static bool powerOffStarted;


void read_pending_can_messages()
{
    uint32_t id;
    bool is_extended;
    bool is_rtr;
    uint8_t filter_match_index;
    uint8_t dlc;
    uint8_t data[8];
    uint16_t ts;

    // Check if a message is received in FIFO 0
    while (can_fifo_pending(CAN1, 0) > 0) 
    {
        // Read the message from FIFO 0
        // Call the receive function
        uint32_t result = can_receive(
            0,           // CAN port (e.g., 0 for CAN1)
            0,           // FIFO (usually 0 or 1)
            true,        // release = true → frees the mailbox after reading
            &id,
            &is_extended,
            &is_rtr,
            &filter_match_index,
            &dlc,
            data,
            &ts
        );

        if (result)
        {
            chademoCharger->HandleChademoMessage(id, data);
        }
        // Process the message (e.g., echo it back)
        //can_transmit(CAN1, id, ext, rtr, len, data);
    }
}


static int buttonPressedCont;

void power_off(int r)
{
    printf("power off request %d\r\n", r);

    // If charging, we need to stop it first. Just keep holding the button and eventually it should stop
    // (unless can is just suddenly dead so the param is never updated, and same for the charger end...)
    //
    bool lockedCcs = Param::Get(Param::LockState);
    bool lockedCha = chademoCharger->IsChargingPlugLocked();
    if (lockedCha || lockedCcs)
        //Param::Get(Param::EvseCurrent) > 5 || Param::Get(Param::EVTargetCurrent) > 5)
    {
        printf("power off request denied, connector is logically locked ccs:%d cha:%d... pending stop charging. just keep pressing stop button.\r\n", lockedCcs, lockedCha);

        bool buttonPressed30Seconds = buttonPressedCont > 60;
        if (buttonPressed30Seconds)
            printf("Stop pressed for 30sec...ignoring the lock....\r\n");
        else
            return;
    }

    printf("Power off now. Bye!\r\n");

    DigIo::contactor_out.Clear();

    DigIo::power_on_out.Clear();

    DigIo::internal_led_out.Set();
    DigIo::power_led_out.Set();

    while (1)
    {
        __WFI();
    }
}

static bool onceIn100;

static void Ms100Task(void)
{
    if (!onceIn100)
    {
        onceIn100 = true;
        printf("in task 100\r\n");
    }

//    iwdg_reset();

    read_pending_can_messages();

    chademoCharger->RunStateMachine();

    //This sets a fixed point value WITHOUT calling the parm_Change() function
   // Param::SetFloat(Param::cpuload, cpuLoad / 10);
    //Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

    //Watchdog
    //int wd = Param::GetInt(Param::CanWatchdog);
    //if (wd < CAN_TIMEOUT)
    //{
    //    Param::SetInt(Param::CanWatchdog, ++wd);
    //}
    //else
    //{
    //    /* we rely on the CAN input, so we set the error indication if the CAN messages timed out. */
    //    ErrorMessage::Post(ERR_CANTIMEOUT);
    //}

    //CAN bus a sleep !!!to decide
    //Param::SetInt(Param::CanAwake, (rtc_get_counter_val() - can->GetLastRxTimestamp()) < 200);

    //Param::SetInt(Param::CanAwake, (rtc_get_ms() - can->GetLastRxTimestamp()) < 200);

    //wakecontrol_mainfunction();

    //if (DigIo::stop_button_in_inverted.Get() == false)
    //{
    //    power_off(1);// PowerOffReason::StopButton);
    //}
}




static bool onceIn500;

static void Ms500Task()
{
    if (!onceIn500)
    {
        onceIn500 = true;
        printf("in task 500\r\n");
    }

    //if (!powerOffStarted)
    {
        DigIo::power_led_out.Toggle();
        DigIo::internal_led_out.Toggle();

        // dog
        // leds
    }

    iwdg_reset();

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        power_off(1);// PowerOffReason::StopButton);

        buttonPressedCont++;
    }
    else
    {
        buttonPressedCont = 0;
    }
}

static bool onceIn30;

static void Ms30Task()
{
    if (!onceIn30) {
        onceIn30 = true;
        printf("in task 30\r\n");
    }

    spiQCA7000checkForReceivedData();
    connMgr_Mainfunction(); /* ConnectionManager */
    modemFinder_Mainfunction();
    runSlacSequencer();
    runSdpStateMachine();
    tcp_Mainfunction();
    pevStateMachine_Mainfunction();

//    pushbutton_handlePushbutton();
//    Param::SetInt(Param::ButtonPushed, pushbutton_isPressed500ms());

    //ErrorMessage::SetTime(rtc_get_counter_val());
    ErrorMessage::SetTime(rtc_get_ms());
}

static void SetMacAddress()
{
    uint8_t mac[6];

    mac[0] = 2; //locally administered
    mac[1] = DESIG_UNIQUE_ID0 & 0xFF;
    *((uint32_t*)&mac[2]) = DESIG_UNIQUE_ID2;

    addToTrace(MOD_HOMEPLUG, "Our MAC address: ", mac, 6);

    //more info: https://community.st.com/t5/stm32-mcus/how-to-obtain-and-use-the-stm32-96-bit-uid/ta-p/621443

    setOurMac(mac);
}

/** This function is called when the user changes a parameter */
void Param::Change(Param::PARAM_NUM paramNum)
{
//    static bool enableReceived = false;

    switch (paramNum)
    {
    case Param::EVTargetCurrent:
        //Charge current is the single most important item that must be constantly updated
        //by the BMS or VCU. Whenever it is updated we feed the dog
        //When it is no longer updated the dog will bark and stop the charge session
        //if (enableReceived)
        //Param::SetInt(Param::CanWatchdog, 0);
        //enableReceived = false; //this will be set back to true once enable is received again
        break;

    // OBS: enableReceived is false by default...so enable must be set somewhere?????????????????????????????????????????????????????????????????????????????????
    case Param::enable:
        //by the BMS or VCU. Whenever this AND ChargeCurrent is updated we feed the dog
        //When it is no longer updated the dog will bark and stop the charge session
        //enableReceived = true;
        break;

    case Param::EvseMaxCurrent:
        chademoCharger->SetChargerSetMaxCurrent(Param::GetInt(Param::EvseMaxCurrent));
        break;
    case Param::EvseMaxVoltage:
        chademoCharger->SetChargerSetMaxVoltage(Param::GetInt(Param::EvseMaxVoltage));
        break;
    default:
        //Handle general parameter changes here. Add paramNum labels for handling specific parameters
        break;
    }
}

static void PrintTrace()
{
    static int lastState = 0;
    static uint32_t lastSttPrint = 0;
    int state = Param::GetInt(Param::opmode);
    const char* label = pevSttLabels[state];

    //if ((Param::GetInt(Param::logging) & MOD_PEV) && ((rtc_get_counter_val() - lastSttPrint) >= 100 || lastState != state))
    if ((Param::GetInt(Param::logging) & MOD_PEV) && ((rtc_get_ms() - lastSttPrint) >= 100 || lastState != state))
    {
        lastSttPrint = rtc_get_ms();// counter_val();
        printf("[%u] In state %s. TcpRetries %u\r\n", rtc_get_ms(), label, tcp_getTotalNumberOfRetries());
        lastState = state;
    }
}

static void Ms1000Task()
{
    // ccs stuff?
    // print more stuff about voltages etc..
    PrintTrace();

    // TODO: voltages.
}

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
//extern "C" void tim4_isr(void)
//{
//    printf("tim4_isr \r\n");
//
//    // Optional: clear update flag if you're using UIE
////    if (timer_get_flag(TIM4, TIM_SR_UIF)) {
//  //      timer_clear_flag(TIM4, TIM_SR_UIF);
//    //}
//
//    scheduler->Run();
//}


#define TIM4_DESIRED_PERIOD_MS 10
#define TIM4_TICK_HZ 1000000  // 1 MHz = 1 tick = 1 µs

//void tim4_setup(void)
//{
//    rcc_periph_clock_enable(RCC_TIM4);
//
//    //timer_reset(TIM4);
//    rcc_periph_reset_pulse(RST_TIM4);
//
//    // TODO: initial gpio values???
//
//    // Prescaler: 168 MHz -> 1 MHz timer clock
//    timer_set_prescaler(TIM4, (rcc_apb1_frequency * 2 / TIM4_TICK_HZ) - 1);
//
//    // Auto-reload value for 10ms = 10,000 ticks @ 1 MHz
//    timer_set_period(TIM4, 10000 - 1);
//
//    // Enable update interrupt (optional — if you're not using CCx match only)
//    timer_enable_irq(TIM4, TIM_DIER_UIE);
//
//    // Also enable CCx channels for your scheduler to use
//    timer_enable_irq(TIM4, TIM_DIER_CC1IE | TIM_DIER_CC2IE | TIM_DIER_CC3IE | TIM_DIER_CC4IE);
//    timer_set_oc_mode(TIM4, TIM_OC1, TIM_OCM_FROZEN); timer_enable_oc_output(TIM4, TIM_OC1);
//    timer_set_oc_mode(TIM4, TIM_OC2, TIM_OCM_FROZEN); timer_enable_oc_output(TIM4, TIM_OC2);
//    timer_set_oc_mode(TIM4, TIM_OC3, TIM_OCM_FROZEN); timer_enable_oc_output(TIM4, TIM_OC3);
//    timer_set_oc_mode(TIM4, TIM_OC4, TIM_OCM_FROZEN); timer_enable_oc_output(TIM4, TIM_OC4);
//
//    // Start timer
//    timer_enable_counter(TIM4);
//
//    // Enable interrupt
//    nvic_enable_irq(NVIC_TIM4_IRQ);
//}

/* Set STM32 system clock to 168 MHz. */
//static void clock_setup(void)
//{
//    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
//
//    /* Enable GPIOG clock. */
//    rcc_periph_clock_enable(RCC_GPIOG);
//}

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;

/* Called when systick fires */
extern "C" void sys_tick_handler(void)
{
    system_millis += 1;
}

uint32_t get_system_millis()
{
    return system_millis;
}

/* sleep for delay milliseconds */
//static void msleep(uint32_t delay)
//{
//    uint32_t wake = system_millis + delay;
//    while (wake > system_millis);
//}

void usart1_setup(void)
{
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_USART1);
#if USE_USART_DMA
    rcc_periph_clock_enable(RCC_DMA2); // DMA2 needed for USART1 TX
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

#if USE_USART_DMA
    // tx dma
    usart_enable_tx_dma(USART1);

    {
        dma_stream_reset(DMA2, DMA_STREAM7);
        dma_set_transfer_mode(DMA2, DMA_STREAM7, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
        dma_set_peripheral_address(DMA2, DMA_STREAM7, (uint32_t)&USART_DR(USART1));
        dma_set_peripheral_size(DMA2, DMA_STREAM7, DMA_SxCR_PSIZE_8BIT);
        dma_set_memory_size(DMA2, DMA_STREAM7, DMA_SxCR_MSIZE_8BIT);
        dma_enable_memory_increment_mode(DMA2, DMA_STREAM7);
        dma_channel_select(DMA2, DMA_STREAM7, DMA_SxCR_CHSEL_4);

      /*  dma_stream_reset(DMA2, hw->streamrx);
        dma_set_transfer_mode(DMA2, hw->streamrx, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
        dma_set_peripheral_address(DMA2, hw->streamrx, (uint32_t)&USART_DR(usart));
        dma_set_peripheral_size(DMA2, hw->streamrx, DMA_SxCR_PSIZE_8BIT);
        dma_set_memory_size(DMA2, hw->streamrx, DMA_SxCR_MSIZE_8BIT);
        dma_enable_memory_increment_mode(DMA2, hw->streamrx);
        dma_channel_select(DMA2, hw->streamrx, hw->channel);
        dma_set_memory_address(DMA2, hw->streamrx, (uint32_t)inBuf);
        dma_set_number_of_data(DMA2, hw->streamrx, bufSize);
        dma_enable_stream(DMA2, hw->streamrx);*/
    }
#endif

    usart_enable(USART1);
}

//void dma2_stream7_setup(void)
//{
//    dma_stream_reset(DMA2, DMA_STREAM7);
//
//    // Set channel 4 for USART1_TX
//    dma_channel_select(DMA2, DMA_STREAM7, DMA_SxCR_CHSEL_4);
//
//    dma_set_priority(DMA2, DMA_STREAM7, DMA_SxCR_PL_HIGH);
//
//    dma_disable_peripheral_increment_mode(DMA2, DMA_STREAM7);
//
//    dma_set_peripheral_address(DMA2, DMA_STREAM7, (uint32_t)&USART1_DR);
//}


#if USE_USART_DMA
void usart1_dma_send(const void* data, uint16_t len, bool firstSend)
{
    while (!dma_get_interrupt_flag(DMA2, DMA_STREAM7, DMA_TCIF) && !firstSend);

    dma_disable_stream(DMA2, DMA_STREAM7);
    dma_set_number_of_data(DMA2, DMA_STREAM7, curIdx + 1);
    dma_set_memory_address(DMA2, DMA_STREAM7, (uint32_t)outBuf[curBuf]);
    dma_clear_interrupt_flags(DMA2, DMA_STREAM7, DMA_TCIF);
    dma_enable_stream(DMA2, DMA_STREAM7);
}
#endif

/*
 * systick_setup(void)
 *
 * This function sets up the 1khz "system tick" count. The SYSTICK counter is a
 * standard feature of the Cortex-M series.
 */
static void systick_setup(void)
{
    /* clock rate / 1000 to get 1mS interrupt rate */
    systick_set_reload(168000 - 1); // 1ms
    //systick_set_reload(1680000); // for 168 MHz core clock -> 10ms

    systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    systick_counter_enable();
    /* this done last */
    systick_interrupt_enable();

//    If the bootloader used SysTick(common in USB DFU), it might be left running or misconfigured.
//        Always do:
    //systick_interrupt_disable();
    //systick_counter_disable();
    //systick_clear();  // optional but good hygiene
    //systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
    //systick_set_reload(...);
    //systick_counter_enable();
    //systick_interrupt_enable();



}


void iwdg_configure(uint16_t period_ms)
{
    // Set watchdog timeout period
    iwdg_set_period_ms(period_ms);

    // Start the watchdog
    iwdg_start();
}

//static void tim_setup(void)
//{
//    /* Enable TIM2 clock. */
//    rcc_periph_clock_enable(RCC_TIM2);
//
//    /* Enable TIM2 interrupt. */
//    nvic_enable_irq(NVIC_TIM2_IRQ);
//
//    /* Reset TIM2 peripheral to defaults. */
//    rcc_periph_reset_pulse(RST_TIM2);
//
//    /* Timer global mode:
//     * - No divider
//     * - Alignment edge
//     * - Direction up
//     * (These are actually default values after reset above, so this call
//     * is strictly unnecessary, but demos the api for alternative settings)
//     */
//    timer_set_mode(TIM2, TIM_CR1_CKD_CK_INT,
//        TIM_CR1_CMS_EDGE, TIM_CR1_DIR_UP);
//
//    /*
//     * Please take note that the clock source for STM32 timers
//     * might not be the raw APB1/APB2 clocks.  In various conditions they
//     * are doubled.  See the Reference Manual for full details!
//     * In our case, TIM2 on APB1 is running at double frequency, so this
//     * sets the prescaler to have the timer run at 5kHz
//     */
//    timer_set_prescaler(TIM2, ((rcc_apb1_frequency * 2) / 5000));
//
//    /* Disable preload. */
//    timer_disable_preload(TIM2);
//    timer_continuous_mode(TIM2);
//
//    /* count full range, as we'll update compare value continuously */
//    timer_set_period(TIM2, 65535);
//
//    /* Set the initual output compare value for OC1. */
//    timer_set_oc_value(TIM2, TIM_OC1, frequency_sequence[frequency_sel++]);
//
//    /* Counter enable. */
//    timer_enable_counter(TIM2);
//
//    /* Enable Channel 1 compare interrupt to recalculate compare values */
//    timer_enable_irq(TIM2, TIM_DIER_CC1IE);
//}



#define MAX_TASKS 4

typedef void (*task_func_t)(void);

typedef struct {
    task_func_t task;
    uint32_t period_ms;
    uint32_t next_run;
    uint8_t enabled;
} scheduler_task_t;

static scheduler_task_t tasks[MAX_TASKS];
static uint8_t task_count = 0;
//extern volatile uint32_t system_millis;  // from your systick driver

// Add a task to the scheduler
int scheduler_add_task(task_func_t func, uint32_t period_ms)
{
    if (task_count >= MAX_TASKS) {
        return -1; // no space
    }
    tasks[task_count].task = func;
    tasks[task_count].period_ms = period_ms;
    tasks[task_count].next_run = system_millis + period_ms;
    tasks[task_count].enabled = 1;
    task_count++;
    return 0;
}

// Run scheduler loop (call this in main)
void scheduler_run(void)
{
    uint32_t now = system_millis;
    for (uint8_t i = 0; i < task_count; i++) {
        if (tasks[i].enabled && ((int32_t)(now - tasks[i].next_run) >= 0)) {
            tasks[i].task();
            tasks[i].next_run += tasks[i].period_ms;
        }
    }
}

extern "C" int main(void)
{
    // set new vector table in a safe way (bootloader does not set it)
    __disable_irq();                   // Disable all interrupts
    SCB->VTOR = 0x08020000;           // Set new vector table
    __DSB();                          // Ensure memory side effects complete
    __ISB();                          // Flush pipeline so VTOR takes effect
    __enable_irq();                   // Re-enable interrupts

    clock_setup(); //Must always come first?

    iwdg_configure(5000);

    DIG_IO_CONFIGURE(DIG_IO_LIST);

    DigIo::power_on_out.Set();
    DigIo::spi_cs_out.Set();
    DigIo::spi_clock_out.Set();
    DigIo::state_c_out_inverted.Set();
    DigIo::contactor_out.Clear();
    DigIo::chademo_unknown_out.Clear();
    DigIo::spi_mosi_out.Set();

    DigIo::power_led_out.Set();
    DigIo::internal_led_out.Set();

    usart1_setup();

    printf("ccs32clara-chademo v0.1\r\n");

    printf("rcc_ahb_frequency:%d, rcc_apb1_frequency:%d, rcc_apb2_frequency:%d\r\n", rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency);
    // rcc_ahb_frequency:168000000, rcc_apb1_frequency:42000000, rcc_apb2_frequency:84000000

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        // power down at startup
        power_off(3);
    }

    systick_setup();


    //ANA_IN_CONFIGURE(ANA_IN_LIST);
//    AnaIn::Start(); //Starts background ADC conversion via DMA

    wakecontrol_init(); //Make sure board stays awake

    hardwareInterface_setStateB();

    SetMacAddress();

    qca7000setup();

    demoQCA7000();

    ChademoCharger cc;
    chademoCharger = &cc;

    scheduler_add_task(Ms30Task, 30);
    scheduler_add_task(Ms100Task, 100);
    scheduler_add_task(Ms500Task, 500);
    scheduler_add_task(Ms1000Task, 1000);

    Param::SetInt(Param::LockState, LOCK_OPEN); //Assume lock open
  //  Param::SetInt(Param::VehicleSideIsoMonAllowed, 1); /* isolation monitoring on vehicle side is allowed per default */
//    Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation

    printf("begin WFI loop\r\n");

    while(1)
    {
        scheduler_run();

        __WFI();
    }

    return 0;
}


void ChademoCharger::CanSend(int id, bool ext, bool rtr, int len, uint8_t* data)
{
    can_transmit(CAN1, id, ext, rtr, len, data);
};

void ChademoCharger::SetSwitchD2(bool set)
{
    if (set)
        DigIo::switch_d2_out_inverted.Clear(); // 0: on
    else
        DigIo::switch_d2_out_inverted.Set(); // 1: off
};

bool ChademoCharger::GetSwitchK()
{
    return DigIo::switch_k_in_inverted.Get() == false; // inverted...0 = on
};

bool ChademoCharger::IsChargingStoppedByAdapter()
{
    return DigIo::stop_button_in_inverted.Get() == false;
};

//bool ChademoCharger::AdapterStopBeforeCharging()
//{
//    return DigIo::stop_button_in_inverted.Get() == false;
//};

void ChademoCharger::NotifyAdapterGpioStuffAfterContactorClosed()
{
    // wonder what this is...
    DigIo::chademo_unknown_out.Set();
    // DigIo::internal_led_out.Clear(); pointless
};

void ChademoCharger::StopPowerDelivery()
{
    // TODO: also lower the voltage?

    Param::Set(Param::enable, false);
};

bool ChademoCharger::IsChargingStoppedByCharger()
{
    return Param::GetInt(Param::StopReason) != _stopreasons::STOP_REASON_NONE;
}

#if USE_USART_DMA
static const int bufSize = 128;
static bool firstSend = true;
static char outBuf[2][bufSize]; //double buffering
static uint8_t curBuf;
static int lastIdx;
static uint32_t curIdx;
#endif

extern "C" void putchar(char c)
{
//    

#if USE_USART_DMA
    if (c == '\n' || curIdx == (bufSize - 1))
    {
        outBuf[curBuf][curIdx] = c;

        usart1_dma_send(outBuf[curBuf], curIdx + 1, firstSend);

        curBuf = !curBuf; //switch buffers
        firstSend = false; //only needed once so we don't get stuck in the while loop above
        curIdx = 0;
    }
    else
    {
        outBuf[curBuf][curIdx] = c;
        curIdx++;
    }
#endif
    //trace_send_blocking8(31, c);
//    dma_send_usart1(&c, 1);

    usart_send_blocking(USART1, c);

//    while (!(USART1_SR & USART_SR_TXE));  // wait until TXE (transmit buffer empty)
  //  USART1_DR = c;
    // TODO
};


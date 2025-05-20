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
//#include <stddef.h>

//#include "libopencm3/cm3/cortex.h"
#include <libopencm3/stm32/usart.h>
//#include <libopencm3/stm32/timer.h>
//#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
//#include <libopencm3/stm32/desig.h>
#include <libopencm3/cm3/systick.h>
//#include <libopencm3/cm3/nvic.h>
//#include <libopencm3/cm3/itm.h>
//#include <libopencm3/cm3/dwt.h>
//#include <libopencm3/cm3/scs.h>
//#include <libopencm3/cm3/scb.h>
//#include <libopencm3/cm3/tpiu.h>
//#include <libopencm3/stm32/dbgmcu.h>

#include <libopencmsis/core_cm3.h>

//#include <libopencm3/stm32/rcc.h>
//#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/gpio.h>
//#include <libopencm3/stm32/dma.h>

#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
//#include "anain.h"
//#include "param_save.h"
//#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
//#include "my_string.h"

//#include "configuration.h"
#include "connMgr.h"
#include "hardwareInterface.h"
#include "homeplug.h"
#include "ipv6.h"
#include "modemFinder.h"
#include "myHelpers.h"
#include "pevStateMachine.h"
#include "qca7000.h"
#include "tcp.h"
//#include "udpChecksum.h"
//#include "pushbutton.h"
//#include "wakecontrol.h"
#include "chademoCharger.h"
#include "led_blinker.h"

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


// need volatile???
volatile static int stopButtonPressedCounter = 0;

static ChademoCharger* chademoCharger;
static LedBlinker* ledBlinker;

void LedBlinker::onLedChange(bool set)
{
    if (set)
    {
        DigIo::internal_led_out_inverted.Clear();
        DigIo::external_led_out_inverted.Clear();
    }
    else
    {
        DigIo::internal_led_out_inverted.Set();
        DigIo::external_led_out_inverted.Set();
    }
}

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
    }
}

void power_off(int i)
{
    printf("Power off now %d. Bye!\r\n", i);

    DigIo::switch_d1_out.Clear();

    // stop can?

    DigIo::power_on_out.Clear();

    DigIo::external_led_out_inverted.Set(); // led off
    DigIo::internal_led_out_inverted.Set(); // led off

    while (1)
    {
        __WFI();
    }
}

void power_off_check()
{
    // fixme: power off after charging?

    if (stopButtonPressed)
    {
//        printf("stopButtonPressed\r\n", r);

        bool buttonPressed30Seconds = stopButtonPressedCounter > 60; // 30 seconds
        if (buttonPressed30Seconds)
        {
            printf("Stop pressed for 30sec...\r\n");
            power_off(1);
        }

        bool lockedCcs = Param::Get(Param::LockState) == LOCK_CLOSED;
        bool lockedCha = chademoCharger->IsChargingPlugLocked();

        if (lockedCha || lockedCcs)
            //Param::Get(Param::EvseCurrent) > 5 || Param::Get(Param::EVTargetCurrent) > 5)
        {
            printf("power off request denied, connector is logically locked ccs:%d cha:%d... pending stop charging. just keep pressing stop button.\r\n", lockedCcs, lockedCha);
            return;
        }

        printf("Both unlocked so power off");
        power_off(2);
    }
}

enum GlobalState
{
    Init,
    AmpsAsked, // > 0
    AmpsDelivered, // > 5
    AmpsDeliveryDone // <= 5 (obs can be temp drop of amps...should use a different logic...
};

//static GlobalState _globalState = GlobalState::Init;

static bool onceIn100;

static void Ms100Task(void)
{
    if (!onceIn100)
    {
        onceIn100 = true;
        printf("in task 100\r\n");
    }

    ledBlinker->tick(100); // 100 ms tick

    read_pending_can_messages();
    chademoCharger->RunStateMachine();

    iwdg_reset();

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        stopButtonPressed = true; // one way street
        stopButtonPressedCounter++;

        ledBlinker->setOnOffDuration(300, 300); // short blinking
    }
    else
    {
        stopButtonPressedCounter = 0;
    }

    if (!chargingAccomplished && Param::Get(Param::EvseCurrent) > 5) // 5 amps
    {
        // dont care about chademo, if 5A is delivered it has to go somewhere:-)
        chargingAccomplished = true; // one way street
    }

    if (!stopButtonPressed && chargingAccomplished)
    {
        ledBlinker->setOnOffDuration(1200, 1200); // long blinking
    }

    // mirror these values (Change method is only called for some params...)
    chademoCharger->SetChargerSetMaxCurrent(Param::GetInt(Param::EvseMaxCurrent));
    chademoCharger->SetChargerSetMaxVoltage(Param::GetInt(Param::EvseMaxVoltage));

    power_off_check();

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

static void Ms30Task()
{
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
    (void)paramNum;
    // This does not suit my neads. I guess I could change so it triggered on all params. But I did a manual mirror in 100ms instead.

//    static bool enableReceived = false;

    //switch (paramNum)
    //{
    //case Param::EVTargetCurrent:
    //    //Charge current is the single most important item that must be constantly updated
    //    //by the BMS or VCU. Whenever it is updated we feed the dog
    //    //When it is no longer updated the dog will bark and stop the charge session
    //    //if (enableReceived)
    //    //Param::SetInt(Param::CanWatchdog, 0);
    //    //enableReceived = false; //this will be set back to true once enable is received again
    //    break;

    //// OBS: enableReceived is false by default...so enable must be set somewhere?????????????????????????????????????????????????????????????????????????????????
    //case Param::enable:
    //    //by the BMS or VCU. Whenever this AND ChargeCurrent is updated we feed the dog
    //    //When it is no longer updated the dog will bark and stop the charge session
    //    //enableReceived = true;
    //    break;

    //case Param::EvseMaxCurrent:
    //    // FIXME: Since set with SetFloat.....Change is not called!!!!!!!!!!!!!! What logic is this?????????????????????
    //    break;
    //case Param::EvseMaxVoltage:
    //    // FIXME: Since set with SetFloat.....Change is not called!!!!!!!!!!!!!! What logic is this?????????????????????
    //    break;
}

static void PrintTrace()
{
    static int lastState = 0;
    static uint32_t lastSttPrint = 0;
    int state = Param::GetInt(Param::opmode);
    const char* label = pevSttLabels[state];

    //if ((Param::GetInt(Param::logging) & MOD_PEV) && ((rtc_get_counter_val() - lastSttPrint) >= 100 || lastState != state))
    // I guess the >= 100 does not matter now that we call it every 1000ms.
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
}


void iwdg_configure(uint16_t period_ms)
{
    // Set watchdog timeout period
    iwdg_set_period_ms(period_ms);

    // Start the watchdog
    iwdg_start();
}

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
    DigIo::switch_d1_out.Clear();
    DigIo::contactor_out.Clear();
    DigIo::spi_mosi_out.Set();

    DigIo::internal_led_out_inverted.Set(); // led off
    DigIo::external_led_out_inverted.Set(); // led off

    usart1_setup();

    printf("ccs32clara-chademo v0.1\r\n");

    printf("rcc_ahb_frequency:%d, rcc_apb1_frequency:%d, rcc_apb2_frequency:%d\r\n", rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency);
    // rcc_ahb_frequency:168000000, rcc_apb1_frequency:42000000, rcc_apb2_frequency:84000000

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        // emergency power down at startup
        power_off(3);
    }

    systick_setup();


    //ANA_IN_CONFIGURE(ANA_IN_LIST);
//    AnaIn::Start(); //Starts background ADC conversion via DMA

    //wakecontrol_init(); //Make sure board stays awake

    hardwareInterface_setStateB();

    SetMacAddress();

    qca7000setup();

    demoQCA7000();

    ChademoCharger cc;
    chademoCharger = &cc;

    LedBlinker lb(600, 600); // medium blibking
    ledBlinker = &lb;

    scheduler_add_task(Ms30Task, 30);
    scheduler_add_task(Ms100Task, 100);
//    scheduler_add_task(Ms500Task, 500);
    scheduler_add_task(Ms1000Task, 1000);
    

    Param::SetInt(Param::LockState, LOCK_OPEN); //Assume lock open
    Param::SetInt(Param::VehicleSideIsoMonAllowed, 1); /* isolation monitoring on vehicle side is allowed per default */
    Param::Change(Param::PARAM_LAST); //Call callback one for general parameter propagation????

    printf("begin WFI loop\r\n");

    while(1)
    {
        // TODO: to save mem, we could do this only every 10ms instead of every 1ms as currently.
        scheduler_run();

        __WFI();
    }

    return 0;
}



extern "C" void putchar(char c)
{
    usart_send_blocking(USART1, c);
};

//double __aeabi_f2d(float f) {
//    return (double)f;
//}
// 

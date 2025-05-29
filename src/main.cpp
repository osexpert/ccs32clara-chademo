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
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/can.h>
//#include <libopencm3/stm32/memorymap.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/systick.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/gpio.h>

#include "params.h"
#include "digio.h"
#include "hwinit.h"
#include "errormessage.h"
#include "printf.h"
#include "connMgr.h"
#include "hardwareInterface.h"
#include "homeplug.h"
#include "ipv6.h"
#include "modemFinder.h"
#include "myHelpers.h"
#include "pevStateMachine.h"
#include "qca7000.h"
#include "tcp.h"
#include "chademoCharger.h"
#include "led_blinker.h"
#include "my_fp.h"
//#include "scheduler.h"
#include "main.h"
#include "stm32scheduler.h"


#define __DSB()  __asm__ volatile ("dsb" ::: "memory")
#define __ISB()  __asm__ volatile ("isb" ::: "memory")
//#define __disable_irq() __asm__ volatile ("cpsid i")
//#define __enable_irq()  __asm__ volatile ("cpsie i")

Stm32Scheduler* scheduler;

global_data _global;

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering
 */
volatile uint32_t system_millis;
// volatile?


#define AUTO_POWER_OFF_LIMIT_SEC (60 * 5)


/* to solve linker warning, see https://openinverter.org/forum/viewtopic.php?p=64546#p64546 */
extern "C" void __cxa_pure_virtual()
{
    while (1);
}

ChademoCharger* chademoCharger;
LedBlinker* ledBlinker;

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


void power_off_no_return(const char* reason)
{
    printf("Power off: %s. Bye!\r\n", reason);

    // weird...i dont think this has any effect
    DigIo::switch_d1_out.Clear();

    // stop can?

    DigIo::power_on_out.Clear();

    // and wont these be turned off now anyways? when power goes...
    DigIo::external_led_out_inverted.Set(); // led off
    DigIo::internal_led_out_inverted.Set(); // led off

    while (1)
    {
        __WFI();
    }
}

void power_off_check()
{
    bool buttonPressed30Seconds = _global.stopButtonCounter > 60; // 30 seconds
    if (buttonPressed30Seconds)
    {
        power_off_no_return("Stop pressed for 30sec");
    }

    // fixme: power off after charging?

    bool inactivity = _global.auto_power_off_timer_count_up_ms / 1000 > AUTO_POWER_OFF_LIMIT_SEC;

    bool powerOffOkCcs = Param::GetInt(Param::LockState) == LOCK_OPEN;
    bool powerOffOkCha = chademoCharger->IsPowerOffOk();

//    bool autoOffImmediatelyAfterCharging = _global.ccsDeliveredAmpsEvent && not_anymore;

    if ((_global.stopButtonEvent || inactivity || _global.ccsDeliveredAmpsEvent) && powerOffOkCcs && powerOffOkCha)// || _cyclesWaitingForCcsStart > 3000 /* 5 min*/)
    {
        if (_global.stopButtonEvent)
            printf("power off: stopButtonEvent (and plugs unlocked)\r\n");
        if (inactivity)
            printf("power off: inactivity (and plugs unlocked)\r\n");
        if (_global.ccsDeliveredAmpsEvent)
            printf("power off: amps delivered (but now plugs are unlocked)\r\n");

        power_off_no_return("Plugs unlocked");

        // I Think it only need to check ccs here, i trust it more. If ccs is not delivering amps, it should not matter what chademo says.
        //if (lockedCha || lockedCcs)
        //    //Param::Get(Param::EvseCurrent) > 5 || Param::Get(Param::EVTargetCurrent) > 5)
        //{
        //    //printf("Power off request denied, connector is (logically) locked. pending stop charging. press for 30sec for emergency shutdown.\r\n");
        //    return;
        //}

    }
}

// Buffer to store ADC results (PA8, VREFINT, VBAT)
#define ADC_CHANNEL_COUNT 3
uint16_t adc_results[ADC_CHANNEL_COUNT];
static uint8_t adc_channels[ADC_CHANNEL_COUNT] = { 10, 11, 17 /*vrefint*/ };

void adc_battery_init(void)
{
    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);

    //adc_enable_vrefint();
    //ADC_CCR(ADC1) |= ADC_CCR_VREFEN;
    // Enable VREFINT for calibration by setting bit 22 in ADC_CCR
    ADC_CCR |= (1 << 22);

    // Enable required clocks
    rcc_periph_clock_enable(RCC_ADC1);

    adc_power_off(ADC1);
    // ADC clock: PCLK2/8 = 168/2/8 = 10.5 MHz
    adc_set_clk_prescale(ADC_CCR_ADCPRE_BY8);
    // Set ADC to 12-bit resolution, right-aligned
    adc_set_resolution(ADC1, ADC_CR1_RES_12BIT);
    adc_set_single_conversion_mode(ADC1);
    // Set sample time for all channels (84 cycles, conservative choice)
    adc_set_sample_time_on_all_channels(ADC1, ADC_SMPR_SMP_84CYC);
    adc_power_on(ADC1);
}

static void adc_read_all(void) {
    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adc_set_regular_sequence(ADC1, 1, &adc_channels[i]);
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
        adc_results[i] = adc_read_regular(ADC1);
    }
}

float adc_to_voltage(uint16_t adc, float gain) {
    return adc * (3.3f / 4095.0f) * gain;
}

void print_sysinfo()
{
    // only every 5sec
    static uint32_t nextPrint = 0;
    if (system_millis >= nextPrint)
    {
        adc_read_all();

        float vdd_voltage = (3.3f * ST_VREFINT_CAL) / adc_results[2];

        // after changing for one day... (max:4.0) % fV(max:11.56) vdd:% fV(max : 3.16). But I saw vdd 3.4 earlier.
        printf("[sysinfo] uptime:%dsec %fV (max:4.0) %fV (max:11.56) vdd:%fV (max:3.4) cpu:%d%% pwroff_cnt:%d css_amps_evt:%d 100c:%d 101c:%d 102c:%d 108c:%d 109c:%d ???c:%d 108l:%d 108d:%d 109l:%d 109d:%d\r\n",
            system_millis / 1000,
            FP_FROMFLT(adc_to_voltage(adc_results[1], 2.0f)),
            FP_FROMFLT(adc_to_voltage(adc_results[0], 11.0f)),
            FP_FROMFLT(vdd_voltage),
            //scheduler_get_cpu_usage(),
            scheduler->GetCpuLoad(),
            _global.auto_power_off_timer_count_up_ms / 1000,
            _global.ccsDeliveredAmpsEvent,
            _global.cha100,
            _global.cha101,
            _global.cha102,
            _global.cha108,
            _global.cha109,
            _global.chaOther,

            _global.cha108last,
            _global.cha108dur,
            _global.cha109last,
            _global.cha109dur

        );

        nextPrint = system_millis + 2000; // 1 sec.
    }
    // Min values seen and working: 3.78V 11.56V 3.4V
}


// FIXME: find a good wad to manage auto power off. By some custom wdog etc.
// OR maybe...if chargingAccomplished + currently stopped\unlocked (imply 0 amps delivered))?
//static volatile uint16_t _autoPowerOffCycles100ms = 0; // 100ms. Wait max 5min = 10 * 60 * 5 = 3000

static void Ms100Task(void)
{
//    static int counter = 0;
//    static bool setOnce = false;
//  //  if (counter++ > (30 * 10))// 1,5 min 90 sec, change to 30
//    {
////        if (!setOnce)
//        {
//            Param::SetInt(Param::EvseMaxVoltage, 500);
//            Param::SetInt(Param::EvseMaxCurrent, 200);
//            //out:0V / 0A avail : 464V / 200A
//
//            printf("hack: set available volt and amps\r\n");
//            setOnce = true;
//        }
//    }

    _global.auto_power_off_timer_count_up_ms += 100;

    ledBlinker->tick(100); // 100 ms tick

    iwdg_reset();

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        if (!_global.stopButtonEvent)
        {
            _global.stopButtonEvent = true; // one way street
            printf("Stop button pressed. Stop pending...\r\n");
        }

        _global.stopButtonCounter++;

        ledBlinker->setOnOffDuration(300, 300); // short blinking
    }
    else
    {
        _global.stopButtonCounter = 0;
    }

    // TODO: maybe simply use Locked state? When locked, we assume chargingAccomplished. Then when unlocked, we power off.
    // Becuse of we locked and unlocked, it does not matter if we delivered amps or not (for most cases)
    if (!_global.ccsDeliveredAmpsEvent && Param::GetInt(Param::EvseCurrent) > 0) // or 5 amps? Or > 1? But I guess 1 amp should suffice...
    {
        // dont care about chademo, if amps delivered it has to go somewhere:-)
        _global.ccsDeliveredAmpsEvent = true; // triggered
    }

    // todo: use plug locked trigger instead?
    if (!_global.stopButtonEvent && _global.ccsDeliveredAmpsEvent)
    {
        ledBlinker->setOnOffDuration(1200, 1200); // long blinking
    }

    power_off_check();

    print_sysinfo();

    
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
}

static void Ms100CanSendTask(void)
{
    // no printf here to make it most stable
    chademoCharger->SendCanMessages();
}

static void Ms10Task()
{
    // maybe it can be relaxed later and run less often
    chademoCharger->Run();
}

static void Ms30Task()
{
    if (!_global.ccsKickoff)
    {
        // chademo will set these and then ccs can start
        // TODO: wait until soc has correct value.....need to check some value?
        _global.ccsKickoff = Param::GetInt(Param::BatteryVoltage) > 0
            && Param::GetInt(Param::MaxVoltage) > 0
            && Param::GetInt(Param::TargetVoltage) > 0;

        if (_global.ccsKickoff)
            printf("ccs kickoff, voltages satisfied!\r\n");
        //        else
          //          _cyclesWaitingForCcsStart++; auto power off attempt...
    }

    // hack
//    if (!_global.ccsKickoff)
        return; 

    spiQCA7000checkForReceivedData();
    connMgr_Mainfunction(); /* ConnectionManager */
    modemFinder_Mainfunction();
    runSlacSequencer();
    runSdpStateMachine();
    tcp_Mainfunction();
    pevStateMachine_Mainfunction();

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

static void PrintCcsTrace()
{
    int state = Param::GetInt(Param::opmode);
    const char* label = pevSttLabels[state];

//    if ((Param::GetInt(Param::logging) & MOD_PEV) && ((rtc_get_ms() - lastSttPrint) >= 100 || lastState != state))
        //      lastSttPrint = rtc_get_ms();// counter_val();
    // TODO: log events flags
    printf("In state %s. TcpRetries %u. out:%uV/%uA max:%uV/%uA car: ask:%uA target:%uV batt:%uV max:%uV/%uA\r\n", 
        label,
        tcp_getTotalNumberOfRetries(),
        Param::GetInt(Param::EvseVoltage),
        Param::GetInt(Param::EvseCurrent),
        Param::GetInt(Param::EvseMaxVoltage),
        Param::GetInt(Param::EvseMaxCurrent),
        // car
        Param::GetInt(Param::ChargeCurrent),
        Param::GetInt(Param::TargetVoltage),
        Param::GetInt(Param::BatteryVoltage),
        Param::GetInt(Param::MaxVoltage),
        Param::GetInt(Param::MaxCurrent)
    );
}

// Weird stuff...
//#define FLASH_REFERENCE_ADDR 0x0800C000
//#define MAGIC_MASK 0x20231212
//#define WORD_COUNT 3
//
//alignas(4) volatile uint32_t runtimeBuffer[WORD_COUNT] = { 0 };
//alignas(4) volatile uint32_t referenceBuffer[WORD_COUNT] = { 0 };
//
//// Called once to initialize referenceBuffer
//void initializeReferenceBuffer()
//{
//    const uint32_t* flash = reinterpret_cast<const uint32_t*>(FLASH_REFERENCE_ADDR);
//    for (int i = 0; i < WORD_COUNT; ++i)
//    {
//        uint32_t val = flash[i];
//        val ^= MAGIC_MASK;
//        val |= MAGIC_MASK;
//        referenceBuffer[i] = val;
//    }
//    // magic mask has no effect?
//    //magic 0: 0x2023123f 0x2023123f magic 1 : 0x32335313 0x32335313 magic 2 : 0x36333a37 0x36333a37
// TODO: could scan all of memory hunting for these? we have the offsets they should be separated with.
//}


//extern uint8_t scheduler_get_cpu_usage(void);




/* Called when systick fires */
extern "C" void sys_tick_handler(void)
{
    system_millis += 1;
}

/* sleep for delay milliseconds */
//static void msleep(uint32_t delay)
//{
//    uint32_t wake = system_millis + delay;
//    while (wake > system_millis);
//}


void iwdg_configure(uint16_t period_ms)
{
    // Set watchdog timeout period
    iwdg_set_period_ms(period_ms);

    // Start the watchdog
    iwdg_start();
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

    // spi
    DigIo::spi_cs_out.Set();
    DigIo::spi_clock_out.Set();
    DigIo::spi_mosi_out.Set();

    // ccs
    DigIo::state_c_out_inverted.Set();

    // chademo
    DigIo::switch_d1_out.Clear();
    DigIo::contactor_out.Clear();
    DigIo::switch_d2_out_inverted.Set(); // important!

    // leds
    DigIo::internal_led_out_inverted.Set(); // led off
    DigIo::external_led_out_inverted.Set(); // led off

    usart1_setup();

    printf("ccs32clara-chademo v0.5\r\n");

    printf("rcc_ahb_frequency:%d rcc_apb1_frequency:%d rcc_apb2_frequency:%d\r\n", rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency);
    // rcc_ahb_frequency:168000000, rcc_apb1_frequency:42000000, rcc_apb2_frequency:84000000

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        power_off_no_return("Stop button pressed during startup");
    }

    systick_setup();
    can_setup();
    adc_battery_init();

    //wakecontrol_init(); //Make sure board stays awake

    hardwareInterface_setStateB();
    SetMacAddress();
    qca7000setup();
    demoQCA7000();

    ChademoCharger cc;
    chademoCharger = &cc;
    chademoCharger->_delayCycles = CHA_CYCLES_PER_SEC * 5; // 5 sec delay initial

    LedBlinker lb(600, 600); // medium blinking
    ledBlinker = &lb;

    // scheduler and TIM4 stuff
    rcc_periph_clock_enable(RCC_TIM4);
    Stm32Scheduler s(TIM4);
    scheduler = &s;

    nvic_set_priority(NVIC_TIM4_IRQ, IRQ_PRIORITY_SCHED); //second lowest priority
    nvic_enable_irq(NVIC_TIM4_IRQ); // will now fire tim4_isr


    // temp hack
    Param::SetInt(Param::EvseMaxVoltage, 464);
    // I see that sometimes...this changes after car tell its target voltage etc.
    Param::SetInt(Param::EvseMaxCurrent, 100);
    //Param::SetInt(Param::EvseVoltage, 350); // FAKE IT HARD

    Param::SetInt(Param::LockState, LOCK_OPEN); //Assume lock open
    Param::SetInt(Param::VehicleSideIsoMonAllowed, 1); /* isolation monitoring on vehicle side is allowed per default */
    // TODO: set Param::MaxCurrent to 200?? does it matter? can it hurt? could choose from battery size...

    scheduler->AddTask(Ms30Task, 30);
    scheduler->AddTask(Ms100Task, 100);
    scheduler->AddTask(Ms10Task, CHA_CYCLE_MS); // hack: probably too often
    

    
    while (1)
    {
        __WFI();
    }

    return 0;
}

void AddCanSendTask()
{
    static bool taskAdded = false;
    if (!taskAdded)
    {
        printf("[cha] AddCanSendTask\r\n");
        taskAdded = true;
        scheduler->AddTask(Ms100CanSendTask, 100);
    }
}

extern int usart1_dma_putchar(char c);

extern "C" void putchar(char c)
{
    usart_send_blocking(USART1, c);
    //usart1_dma_putchar(c);
};

//Whichever timer(s) you use for the scheduler, you have to
//implement their ISRs here and call into the respective scheduler
extern "C" void tim4_isr(void)
{
    scheduler->Run();
}

// 
//extern "C" double __aeabi_f2d(float f) {
//    return (double)f;
//}
//extern "C" float __aeabi_d2f(double f) {
//    return (float)f;
//}


/** This function is called when the user changes a parameter */
//void Param::Change(Param::PARAM_NUM paramNum)
//{
////    static bool enableReceived = false;
//    //switch (paramNum)
//    //{
//    //case Param::EVTargetCurrent:
//    //    //Charge current is the single most important item that must be constantly updated
//    //    //by the BMS or VCU. Whenever it is updated we feed the dog
//    //    //When it is no longer updated the dog will bark and stop the charge session
//    //    //if (enableReceived)
//    //    //Param::SetInt(Param::CanWatchdog, 0);
//    //    //enableReceived = false; //this will be set back to true once enable is received again
//    //    break;
//    //// OBS: enableReceived is false by default...so enable must be set somewhere?????????????????????????????????????????????????????????????????????????????????
//    //case Param::enable:
//    //    //by the BMS or VCU. Whenever this AND ChargeCurrent is updated we feed the dog
//    //    //When it is no longer updated the dog will bark and stop the charge session
//    //    //enableReceived = true;
//    //    break;
//}


// minimal memset implementation for embedded
#include <stddef.h>  // for size_t

extern "C" void* memset(void* ptr, int value, size_t num) {
    unsigned char* p = (unsigned char*)ptr;
    while (num--) {
        *p++ = (unsigned char)value;
    }
    return ptr;
}

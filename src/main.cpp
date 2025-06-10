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
#include "main.h"
#include "stm32scheduler.h"


#define __DSB()  __asm__ volatile ("dsb" ::: "memory")
#define __ISB()  __asm__ volatile ("isb" ::: "memory")
//#define __disable_irq() __asm__ volatile ("cpsid i")
//#define __enable_irq()  __asm__ volatile ("cpsie i")

ChademoCharger* chademoCharger;
LedBlinker* ledBlinker;

Stm32Scheduler* scheduler;

global_data _global;

/* monotonically increasing number of milliseconds from reset
 * overflows every 49 days if you're wondering */
volatile uint32_t system_millis;

#define AUTO_POWER_OFF_LIMIT_SEC (60 * 3)

enum MainState
{
    Init,
    WaitForCcsStarted,
    WaitForSlacDone,
    WaitForTcpConnected,
    WaitForPreChargeStart,
    WaitForCurrentDemandLoop,
    WaitForDeliveringAmps,
    Charging,
    Stop
};

MainState _state = MainState::Init;

void RunMainStateMachine()
{
    if (_global.powerOffPending && _state != MainState::Stop)
    {
        ledBlinker->setPattern(blink_stop);
        _state = MainState::Stop;
    }

    if (_state == MainState::Init)
    {
        ledBlinker->setPattern(blink_start);
        _state = MainState::WaitForCcsStarted;
    }
    else if (_state == MainState::WaitForCcsStarted)
    {
        if (_global.ccsKickoff)
        {
            _state = MainState::WaitForSlacDone;
        }
    }
    else if (_state == MainState::WaitForSlacDone)
    {
        if (Param::GetInt(Param::checkpoint) >= 200) // SDP (service discovery, slac is done)
        {
            ledBlinker->setPattern(blink_1);
            _state = MainState::WaitForTcpConnected;
        }
    }
    else if (_state == MainState::WaitForTcpConnected)
    {
        if (Param::GetInt(Param::checkpoint) >= 303) // tcp connected
        {
            ledBlinker->setPattern(blink_2);
            _state = MainState::WaitForPreChargeStart;
        }
    }
    else if (_state == MainState::WaitForPreChargeStart)
    {
        if (_global.ccsPreChargeStartedEvent)
        {
            ledBlinker->setPattern(blink_3);
            _state = MainState::WaitForCurrentDemandLoop;
        }
    }
    else if (_state == MainState::WaitForCurrentDemandLoop)
    {
        if (_global.ccsCurrentDemandStartedEvent)
        {
            ledBlinker->setPattern(blink_4);
            _state = MainState::WaitForDeliveringAmps;
        }
    }
    else if (_state == MainState::WaitForDeliveringAmps)
    {
        if (Param::GetInt(Param::EvseCurrent) > 0)
        {
            ledBlinker->setPattern(blink_working);
            _state = MainState::Charging;
        }
    }
    else if (_state == MainState::Charging)
    {

    }
}

/* to solve linker warning, see https://openinverter.org/forum/viewtopic.php?p=64546#p64546 */
extern "C" void __cxa_pure_virtual()
{
    while (1);
}



void LedBlinker::applyLed(bool set)
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

/* sleep for delay milliseconds */
static void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis);
}

extern bool tcp_isClosed(void);
extern void tcp_reset(void);

void power_off_no_return(const char* reason)
{
    printf("Power off: %s. Bye!\r\n", reason);

    if (!tcp_isClosed())
    {
        tcp_reset();
        msleep(100);
    }

    //if (_global.relayProbablyWeldedEvent)
    //{
    //    printf("!!! WARNING: Contactor may be welded. Use a multimeter to verify. May try the unwelding function (hold stop while power on). !!!\r\n");
    //}

    // weird...i dont think this has any effect (here)
    // commented it
//    DigIo::switch_d1_out.Clear();
    // todo: WAIT

    // stop can?

    // power off adapter contactor here.
    // if we did not, both the car contactors and the adapter contactor would loose power at the same time,
    // and it would be chance who takes the hit.
    // Its better that adapter contactor take the hit than the car, so power it off explicitly first, and wait a bit (20ms should suffice, but do 100 anyways).
    if (DigIo::contactor_out.Get())
    {
        printf("Contactor was closed! This may be bad...\r\n");
        DigIo::contactor_out.Clear();
        msleep(100);
    }

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
    bool buttonPressed30Seconds = _global.stopButtonCounter > 10 * 30; // 30 seconds
    if (buttonPressed30Seconds)
    {
        power_off_no_return("Stop pressed for 30sec -> force off");
    }

    // Power off pending detector
    if (!_global.powerOffPending)
    {
        bool buttonPressedBriefly = _global.stopButtonCounter > 0;
        bool buttonPressed5Seconds = _global.stopButtonCounter > 10 * 5; // 5 seconds
        bool inactivity = _global.auto_power_off_timer_count_up_ms / 1000 > AUTO_POWER_OFF_LIMIT_SEC;

        // allow instant power off, unless Slac is pending (allow cable "fiddle" or late plugin before/during slac)
        if (buttonPressedBriefly && _state != MainState::WaitForSlacDone)
        {
            _global.powerOffPending = true; // one way street
            printf("Stop button pressed briefly and slac not pending. Power off pending...\r\n");
        }
        if (buttonPressed5Seconds)
        {
            _global.powerOffPending = true; // one way street
            printf("Stop button pressed for 5 seconds. Power off pending...\r\n");
        }
        if (inactivity)
        {
            _global.powerOffPending = true; // one way street
            printf("Inactivity. Power off pending...\r\n");
        }

        int ccsStopReason = Param::GetInt(Param::StopReason);
        if (ccsStopReason != STOP_REASON_NONE)
        {
            _global.powerOffPending = true; // one way street
            printf("Ccs stop reason %d. Power off pending...\r\n", ccsStopReason);
        }

        StopReason chaStopReason = chademoCharger->GetStopReason(); // do we need one for success? eg. accu full?
        if (chaStopReason != StopReason::NONE)
        {
            _global.powerOffPending = true; // one way street
            printf("Chademo stop reason %d. Power off pending...\r\n", chaStopReason);
        }
    }

    if (_global.powerOffPending)
    {
        bool powerOffOkCcs = Param::GetInt(Param::LockState) == LOCK_OPEN;
        bool powerOffOkCha = chademoCharger->IsPowerOffOk();

        if (powerOffOkCcs && powerOffOkCha)
        {
            power_off_no_return("powerOffPending and both ccs and chademo says plugs are unlocked");
        }
    }
}



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

float adc_to_voltage(uint16_t adc, float gain) {
    return adc * (3.3f / 4095.0f) * gain;
}

#define ADC_CHANNEL_COUNT 3
void adc_read_all(void)
{
    // Buffer to store ADC results (PA8, VREFINT, VBAT)
    uint16_t adc_results[ADC_CHANNEL_COUNT];
    static uint8_t adc_channels[ADC_CHANNEL_COUNT] = { 10, 11, 17 /*vrefint*/ };

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adc_set_regular_sequence(ADC1, 1, &adc_channels[i]);
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
        adc_results[i] = adc_read_regular(ADC1);
    }

    float vdd_voltage = (3.3f * ST_VREFINT_CAL) / adc_results[2];
    
    _global.adc_3_3_volt = vdd_voltage;
    _global.adc_4_volt = adc_to_voltage(adc_results[1], 2.0f);
    _global.adc_12_volt = adc_to_voltage(adc_results[0], 11.0f);
}



#define SYSINFO_EVERY_MS 2000 // 2 sec

void print_sysinfo()
{
    static uint32_t nextPrint = 0;
    if (system_millis >= nextPrint)
    {
        adc_read_all();

        bool powerOffOkCcs = Param::GetInt(Param::LockState) == LOCK_OPEN;
        bool powerOffOkCha = chademoCharger->IsPowerOffOk();

        // after changing for one day... (max:4.0) % fV(max:11.56) vdd:% fV(max : 3.16). But I saw vdd 3.4 earlier.
        printf("[sysinfo] uptime:%dsec %fV (nom:4.0) %fV (nom:12.0) vdd:%fV (nom:3.3) cpu:%d%% pwroff_cnt:%d pwr_off:%d/%d/%d m_state:%d\r\n",
            system_millis / 1000,
            FP_FROMFLT(_global.adc_4_volt),
            FP_FROMFLT(_global.adc_12_volt),
            FP_FROMFLT(_global.adc_3_3_volt),
            scheduler->GetCpuLoad(),
            _global.auto_power_off_timer_count_up_ms / 1000,
            _global.powerOffPending,
            powerOffOkCcs,
            powerOffOkCha,
			_state
        );

        nextPrint = system_millis + SYSINFO_EVERY_MS;
    }
    // Min values seen and working: 3.78V 11.56V 3.4V
}

#define CCS_TRACE_EVERY_MS 1000 // 1 sec

static void print_ccs_trace()
{
    static uint32_t nextPrint = 0;
    if (system_millis >= nextPrint)
    {
        int state = Param::GetInt(Param::opmode);
        const char* label = pevSttLabels[state];

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

        nextPrint = system_millis + SYSINFO_EVERY_MS;
    }
}

// FIXME: find a good wad to manage auto power off. By some custom wdog etc.
// OR maybe...if chargingAccomplished + currently stopped\unlocked (imply 0 amps delivered))?
//static volatile uint16_t _autoPowerOffCycles100ms = 0; // 100ms. Wait max 5min = 10 * 60 * 5 = 3000

static void Ms100Task(void)
{
    if (_global.relayUnweldingAttempt)
        DigIo::contactor_out.Toggle();
	else
	    chademoCharger->Run();

    _global.auto_power_off_timer_count_up_ms += 100;

    RunMainStateMachine();
    ledBlinker->tick(); // 100 ms tick

    iwdg_reset();

    if (DigIo::stop_button_in_inverted.Get() == false) {
        _global.stopButtonCounter++;
    }
    else {
        _global.stopButtonCounter = 0;
    }

    power_off_check();

    print_sysinfo();
    print_ccs_trace();
}

static void Ms30Task()
{
    if (_global.relayUnweldingAttempt)
		return;

    if (!_global.ccsKickoff)
    {
        // chademo will set these and then ccs can start
        _global.ccsKickoff = chademoCharger->IsAutoDetectCompleted();
        if (!_global.ccsKickoff)
			return;

        printf("ccs kickoff, cha autodetect completed\r\n");
    }

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

/* Called when systick fires */
extern "C" void sys_tick_handler(void)
{
    system_millis += 1;
}

void iwdg_configure(uint16_t period_ms)
{
    // Set watchdog timeout period
    iwdg_set_period_ms(period_ms);

    // Start the watchdog
    iwdg_start();
}

void enable_all_faults(void) 
{
    // HAL call them "_Msk"
    SCB_SHCSR |= SCB_SHCSR_USGFAULTENA | SCB_SHCSR_BUSFAULTENA | SCB_SHCSR_MEMFAULTENA;

    // default is to ignore div/0 (undefined behaviour)
 //   SCB_CCR |= SCB_CCR_DIV_0_TRP_Msk; // Trap divide-by-zero
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

    enable_all_faults();

    printf("ccs32clara-chademo %s\r\n", GITHUB_VERSION);

    printf("rcc_ahb_frequency:%d rcc_apb1_frequency:%d rcc_apb2_frequency:%d\r\n", rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency);
    // rcc_ahb_frequency:168000000, rcc_apb1_frequency:42000000, rcc_apb2_frequency:84000000

    if (DigIo::stop_button_in_inverted.Get() == false)
    {
        _global.relayUnweldingAttempt = true;
        DigIo::contactor_out.Toggle();
        printf("Relay unwelding attempt: hold stop while power on. When you hear contactor close, release stop (before 1sec has passed)\r\n");
        msleep(1000);
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

    LedBlinker lb;
    ledBlinker = &lb;

    // scheduler and TIM4 stuff
    rcc_periph_clock_enable(RCC_TIM4);
    Stm32Scheduler s(TIM4);
    scheduler = &s;

    nvic_set_priority(NVIC_TIM4_IRQ, IRQ_PRIORITY_SCHED); //second lowest priority
    nvic_enable_irq(NVIC_TIM4_IRQ); // will now fire tim4_isr

    Param::SetInt(Param::LockState, LOCK_OPEN); //Assume lock open
    Param::SetInt(Param::VehicleSideIsoMonAllowed, 1); /* isolation monitoring on vehicle side is allowed per default */
    Param::SetInt(Param::MaxCurrent, 200);

    scheduler->AddTask(Ms30Task, 30);
    scheduler->AddTask(Ms100Task, 100);
    
    while (1)
    {
        __WFI();
    }

    return 0;
}

extern "C" void putchar(char c)
{
    usart_send_blocking(USART1, c);
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


// minimal memset implementation for embedded
extern "C" void* memset(void* ptr, int value, size_t num) {
    unsigned char* p = (unsigned char*)ptr;
    while (num--) {
        *p++ = (unsigned char)value;
    }
    return ptr;
}

extern "C" void print_crash_info(const char* type, uint32_t* stack)
{
    printf("\r\n--- ");
    printf(type);
    printf(" ---\r\n");

    printf("R0  : 0x%x\r\n", stack[0]);
    printf("R1  : 0x%x\r\n", stack[1]);
    printf("R2  : 0x%x\r\n", stack[2]);
    printf("R3  : 0x%x\r\n", stack[3]);
    printf("R12 : 0x%x\r\n", stack[4]);
    printf("LR  : 0x%x\r\n", stack[5]);
    printf("PC  : 0x%x\r\n", stack[6]);
    printf("xPSR: 0x%x\r\n", stack[7]);

    printf("CFSR : 0x%x\r\n", SCB_CFSR);
    printf("HFSR : 0x%x\r\n", SCB_HFSR);
    printf("MMFAR: 0x%x\r\n", SCB_MMFAR);
    printf("BFAR : 0x%x\r\n", SCB_BFAR);

    power_off_no_return("fault");
}

// Fault labels in flash
extern "C" const char hardfault_label[] = "HardFault";
extern "C" const char busfault_label[] = "BusFault";
extern "C" const char usagefault_label[] = "UsageFault";
extern "C" const char memfault_label[] = "MemManage";

extern "C" __attribute__((naked)) void hard_fault_handler(void) {
    __asm volatile (
    "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "ldr r1, =hardfault_label \n"
        "b print_crash_info \n"
        );
}

extern "C" __attribute__((naked)) void bus_fault_handler(void) {
    __asm volatile (
    "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "ldr r1, =busfault_label \n"
        "b print_crash_info \n"
        );
}

extern "C" __attribute__((naked)) void usage_fault_handler(void) {
    __asm volatile (
    "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "ldr r1, =usagefault_label \n"
        "b print_crash_info \n"
        );
}

extern "C" __attribute__((naked)) void mem_manage_handler(void) {
    __asm volatile (
    "tst lr, #4        \n"
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "ldr r1, =memfault_label \n"
        "b print_crash_info \n"
        );
}

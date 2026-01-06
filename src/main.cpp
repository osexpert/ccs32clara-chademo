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
#include <libopencm3/stm32/rng.h>
#include <libopencm3/stm32/adc.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/iwdg.h>
#include <libopencm3/cm3/systick.h>
#include <libopencmsis/core_cm3.h>
#include <libopencm3/stm32/gpio.h>

#include "params.h"
#include "digio.h"
#include "hwinit.h"
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
#include "main.h"
#include "stm32scheduler.h"
#include "special_modes.h"

#define __DSB()  __asm__ volatile ("dsb" ::: "memory")
#define __ISB()  __asm__ volatile ("isb" ::: "memory")

ChademoCharger* chademoCharger;
LedBlinker* ledBlinker;

Stm32Scheduler* scheduler;

global_data _global;

volatile uint32_t system_millis;

#define AUTO_POWER_OFF_SEC (60 * 3) // 3 min
#define CCS_TRACE_EVERY_MS 1000 // 1 sec
#define SYSINFO_EVERY_MS 2000 // 2 sec

enum MainProgress
{
    Init,
    WaitForCcsStarted,
    WaitForSlacDone,
    WaitForTcpConnected,
    WaitForPreChargeStart,
    WaitForPreChargeDoneButStalled,
    WaitForCurrentDemandLoop,
    WaitForDeliveringAmps,
    Charging,
    Stop
};

MainProgress _state = MainProgress::Init;

void RunMainProgressStateMachine()
{
    if (_global.powerOffPending && _state != MainProgress::Stop)
    {
        ledBlinker->setPattern(blink_stop);
        _state = MainProgress::Stop;
    }

    if (_state == MainProgress::Init)
    {
        ledBlinker->setPattern(blink_start);
        _state = MainProgress::WaitForCcsStarted;
    }
    else if (_state == MainProgress::WaitForCcsStarted)
    {
        if (_global.ccsKickoff)
        {
            _state = MainProgress::WaitForSlacDone;
        }
    }
    else if (_state == MainProgress::WaitForSlacDone)
    {
        if (Param::GetInt(Param::checkpoint) >= 200) // SDP (service discovery, slac is done)
        {
            ledBlinker->setPattern(blink_1);
            _state = MainProgress::WaitForTcpConnected;
        }
    }
    else if (_state == MainProgress::WaitForTcpConnected)
    {
        if (Param::GetInt(Param::checkpoint) >= 303) // tcp connected
        {
            ledBlinker->setPattern(blink_2);
            _state = MainProgress::WaitForPreChargeStart;
        }
    }
    else if (_state == MainProgress::WaitForPreChargeStart)
    {
        if (Param::GetInt(Param::checkpoint) >= 570)
        {
            ledBlinker->setPattern(blink_3);
            _state = MainProgress::WaitForPreChargeDoneButStalled;
        }
    }
    else if (_state == MainProgress::WaitForPreChargeDoneButStalled)
    {
        if (_global.ccsPreChargeDoneButStalledTrigger)
        {
            ledBlinker->setPattern(blink_4);
            _state = MainProgress::WaitForCurrentDemandLoop;
        }
    }
    else if (_state == MainProgress::WaitForCurrentDemandLoop)
    {
        if (Param::GetInt(Param::checkpoint) >= 700)
        {
            // barely noticable, removed blink
            _state = MainProgress::WaitForDeliveringAmps;
        }
    }
    else if (_state == MainProgress::WaitForDeliveringAmps)
    {
        if (Param::GetInt(Param::EvseCurrent) > 0)
        {
            ledBlinker->setPattern(blink_working);
            _state = MainProgress::Charging;
        }
    }
    else if (_state == MainProgress::Charging)
    {

    }
}

void led_on()
{
    DigIo::internal_led_out_inverted.Clear();
    DigIo::external_led_out_inverted.Clear();
}
void led_off()
{
    DigIo::internal_led_out_inverted.Set();
    DigIo::external_led_out_inverted.Set();
}

void LedBlinker::applyLed(bool set)
{
    if (set) {
        led_on();
    }
    else {
        led_off();
    }
}

/* sleep for delay milliseconds */
static void msleep(uint32_t delay)
{
    uint32_t wake = system_millis + delay;
    while (wake > system_millis);
}

void power_off_no_return(const char* reason)
{
    println("Power off: %s. Bye!", reason);

    // In case of emergency shutdown (stop button for 10 sec, fault, other very bad things) and contactor is still closed, power off adapter contactor here.
    // If we did not, both the car contactors and the adapter contactor would loose power at the same time, and it would be chance who takes the hit.
    // Its better to sacrefice the adapter instead of the car, so power off adapter contactor explicitly first, and wait a bit (20ms should suffice, but do 100 anyways).
    if (DigIo::contactor_out.Get())
    {
        println("Contactor was closed! This may be bad...");
        DigIo::contactor_out.Clear();
        msleep(100);
    }

    DigIo::power_on_out.Clear();

    led_off();

    while (true)
    {
        __WFI();
    }
}

bool ccs_isPowerOffOk()
{
    return not hardwareInterface_isConnectorLocked();
}

void power_off_check()
{
    bool buttonPressed10Seconds = _global.stopButtonCounter > 10 * 10; // 10 seconds
    if (buttonPressed10Seconds)
    {
        power_off_no_return("Stop pressed for 10sec -> force off");
    }

    // Power off pending detector
    if (!_global.powerOffPending)
    {
        bool buttonPressedBriefly = _global.stopButtonCounter > 0;
        bool buttonPressed5Seconds = _global.stopButtonCounter > 10 * 5; // 5 seconds
        bool inactivity = _global.auto_power_off_timer_count_up_ms / 1000 > AUTO_POWER_OFF_SEC;

        // allow instant power off, unless Slac is pending (allow cable "fiddle" or late plugin before/during slac)
        if (buttonPressedBriefly && _state != MainProgress::WaitForSlacDone && not special_modes_selection_pending())
        {
            _global.powerOffPending = true;
            println("Stop button pressed briefly and slac not pending. Power off pending...");
        }
        if (buttonPressed5Seconds)
        {
            _global.powerOffPending = true;
            println("Stop button pressed for 5 seconds. Power off pending...");
        }
        if (inactivity)
        {
            _global.powerOffPending = true;
            println("Inactivity. Power off pending...");
        }

        if (_global.ccsEnded)
        {
            _global.powerOffPending = true;
            println("Ccs ended. Power off pending...");
        }

        int ccsStopReason = Param::GetInt(Param::StopReason);
        if (ccsStopReason != STOP_REASON_NONE)
        {
            _global.powerOffPending = true;
            println("Ccs stop reason 0x%x. Power off pending...", ccsStopReason);
        }

        StopReason chaStopReason = chademoCharger->GetStopReason();
        if (chaStopReason != StopReason::NONE)
        {
            _global.powerOffPending = true;
            println("Chademo stop reason 0x%x. Power off pending...", chaStopReason);
        }
    }

    if (_global.powerOffPending)
    {
        bool powerOffOkCcs = ccs_isPowerOffOk();
        bool powerOffOkCha = chademoCharger->IsPowerOffOk();

        if (powerOffOkCcs && powerOffOkCha)
        {
            power_off_no_return("powerOffPending and both ccs and chademo says power off is ok");
        }
    }
}

void adc_battery_init(void)
{
    adc_enable_temperature_sensor(); // enables vrefint too

    gpio_mode_setup(GPIOC, GPIO_MODE_ANALOG, GPIO_PUPD_NONE, GPIO0 | GPIO1);

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

float adc_to_voltage(uint16_t adc, float vdd_voltage, float gain)
{
    return adc * (vdd_voltage / 4095.0f) * gain;
}

static float adc_3_3_volt = 0.0f;
static float adc_4_volt = 0.0f;
static float adc_12_volt = 0.0f;

#define ADC_CHANNEL_COUNT 3

void adc_read_all(void)
{
    uint16_t adc_results[ADC_CHANNEL_COUNT];
    static uint8_t adc_channels[ADC_CHANNEL_COUNT] = { 10 /* PC0 */, 11 /* PC1 */, ADC_CHANNEL_VREF };

    for (int i = 0; i < ADC_CHANNEL_COUNT; i++) {
        adc_set_regular_sequence(ADC1, 1, &adc_channels[i]);
        adc_start_conversion_regular(ADC1);
        while (!adc_eoc(ADC1));
        adc_results[i] = adc_read_regular(ADC1);
    }

    if (adc_results[2] > 0) // prevent DIV/0
    {
        float vdd = (3.3f * ST_VREFINT_CAL) / adc_results[2];

        adc_3_3_volt = vdd;
        adc_4_volt = adc_to_voltage(adc_results[1], vdd, 2.0f);
        adc_12_volt = adc_to_voltage(adc_results[0], vdd, 11.0f);
    }
}


void print_sysinfo()
{
    static uint32_t nextPrint = 0;
    if (system_millis >= nextPrint)
    {
        adc_read_all();

        bool powerOffOkCcs = ccs_isPowerOffOk();
        bool powerOffOkCha = chademoCharger->IsPowerOffOk();

        // after charging via usb-c for one day... vcc4:4v vcc12:11.56v vdd:3.16v (vdd was low here)
        // during charging car, vcc12 seen between 12.19-12.31V
        // Min values seen and working: vcc4:3.78V vcc12:11.46V
        println("[sysinfo] uptime:%dsec vcc4:%fV vcc12:%fV vdd:%fV cpu:%d%% pwroff_cnt:%d pwr_off:%d/%d/%d m_state:%d",
            system_millis / 1000,
            &adc_4_volt, // bypass float to double promotion by passing as reference
            &adc_12_volt, // bypass float to double promotion by passing as reference
            &adc_3_3_volt, // bypass float to double promotion by passing as reference
            scheduler->GetCpuLoad() / 10,
            _global.auto_power_off_timer_count_up_ms / 1000,
            _global.powerOffPending,
            powerOffOkCcs,
            powerOffOkCha,
            _state
        );

        nextPrint = system_millis + SYSINFO_EVERY_MS;
    }
}

static void print_ccs_trace()
{
    static uint32_t nextPrint = 0;
    if (system_millis >= nextPrint)
    {
        int state = Param::GetInt(Param::opmode);
        const char* label = pevSttLabels[state];

        println("[ccs] In state %s. TcpRetries %u. out:%uV/%uA max:%uV/%uA car: ask:%uA target:%uV batt:%uV max:%uV/%uA",
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

void special_mode_selected(enum SpecialMode mode)
{
    println("Special mode selected:%d", mode);

    if (mode == SpecialMode::Discharge)
        chademoCharger->EnableDischarge();
    else if (mode == SpecialMode::LongerPrecharge)
        chademoCharger->EnableLongerPrecharge();
    else if (mode == SpecialMode::MoreLogging)
        _global.moreLogging = true;
}

static void Ms100Task(void)
{
    bool stopPressed = not DigIo::stop_button_in_inverted.Get();
    special_modes_tick_100ms(stopPressed);
    if (special_modes_selection_pending())
    {
        // waiting...
    }
    else
    {
        RunMainProgressStateMachine();
        ledBlinker->tick(); // 100 ms tick

        if (special_modes_is_selected(SpecialMode::Unwelding))
            DigIo::contactor_out.Toggle();
        else
            chademoCharger->Run();
    }

    _global.auto_power_off_timer_count_up_ms += 100;

    iwdg_reset();

    if (stopPressed) {
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
    if (!_global.ccsKickoff)
    {
        // chademo will set these and then ccs can start
        _global.ccsKickoff = chademoCharger->IsDiscoveryCompleted();
        if (!_global.ccsKickoff)
            return;

        println("[cha] discovery completed => ccs kickoff");
    }

#ifdef CHADEMO_STANDALONE_TESTING

    static int delay = 100;
    if (delay > 0 && --delay == 0)
    {
        chademoInterface_preChargeCompleted();
    }
    return;
#endif

    // run eth even after ccsEnded, so we look "alive" to the charger
    // (some chargers complain about contact lost with car after charging, this may fix it?)
    spiQCA7000checkForReceivedData();

    if (_global.ccsEnded)
        return;

    connMgr_Mainfunction(); /* ConnectionManager */
    modemFinder_Mainfunction();
    runSlacSequencer();
    runSdpStateMachine();
    tcp_Mainfunction();
    pevStateMachine_Mainfunction();

    _global.ccsEnded = chademoInterface_ccsInEndState();
    if (_global.ccsEnded)
        tcp_disconnect(); // kill the last connection, if any
}

static void SetMacAddress()
{
    uint8_t mac[6];

    mac[0] = 2; //locally administered
    mac[1] = DESIG_UNIQUE_ID0 & 0xFF;
    *((uint32_t*)&mac[2]) = DESIG_UNIQUE_ID2;

    println("Our MAC address: %02x:%02x:xx:xx:xx:%02x", mac[0], mac[1], mac[5]);
    //more info: https://community.st.com/t5/stm32-mcus/how-to-obtain-and-use-the-stm32-96-bit-uid/ta-p/621443
    setOurMac(mac);
}

static void SetRandomStartPort()
{
    rng_enable();
    uint32_t rand_num = rng_get_random_blocking(); // Get a random number
    rng_disable(); // Disable RNG when finished to save power

    uint16_t start_port = setStartPort(rand_num);
    println("Random start port:%u", start_port);
}

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

    println("ccs32clara-chademo %s", GITHUB_VERSION);

    //println("experiment: {experiment info here}");

    println("rcc_ahb_frequency:%d rcc_apb1_frequency:%d rcc_apb2_frequency:%d", rcc_ahb_frequency, rcc_apb1_frequency, rcc_apb2_frequency);
    // rcc_ahb_frequency:168000000, rcc_apb1_frequency:42000000, rcc_apb2_frequency:84000000

    bool stopPressed = not DigIo::stop_button_in_inverted.Get();
    special_modes_init(stopPressed);

    systick_setup();
    can_setup();
    adc_battery_init();

    hardwareInterface_setStateB();
    SetMacAddress();
    SetRandomStartPort();
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
    Param::SetInt(Param::MaxCurrent, 200);

    scheduler->AddTask(Ms30Task, 30);
    scheduler->AddTask(Ms100Task, 100);
    
    while (true)
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
    println("--- %s ---", type);

    println("R0  : 0x%x", stack[0]);
    println("R1  : 0x%x", stack[1]);
    println("R2  : 0x%x", stack[2]);
    println("R3  : 0x%x", stack[3]);
    println("R12 : 0x%x", stack[4]);
    println("LR  : 0x%x", stack[5]);
    println("PC  : 0x%x", stack[6]);
    println("xPSR: 0x%x", stack[7]);

    println("CFSR : 0x%x", SCB_CFSR);
    println("HFSR : 0x%x", SCB_HFSR);
    println("MMFAR: 0x%x", SCB_MMFAR);
    println("BFAR : 0x%x", SCB_BFAR);

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

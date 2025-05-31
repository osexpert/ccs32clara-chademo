# ccs32clara-chademo

Alternative firmware for adapter that uses My407ccs2chademo.bin.
Adapter probably uses a stm32f407 cpu. It has a boot loader with support for firmware update from usb fat32, very nice.

It has an internal DC transformator to charge the batteries (pri: 210-1200V DC sec: 12.6V DC 2A).
It has 2 batteries. One short and one long. The long one seems to drive electronics. Guessing the short one drive contactors (one in the adapter itself + 2 in the car).
2 stop buttons, but they are the same GPIO for both.
It has 2 controllable leds, one external and one internal.
It has a QCA7000 powerline modem. It is wired up strangely, so can not use hw spi/dma, must use bitbang.
Some of the voltages are wired up to adc gpios. At least the 12v reading does not seem to match with what i measure on eg. Chademo d1 pin.

Operation:
Plug adapter into car. Ccs logic will not start until chademo communication is established and we have maxVoltage, targetVoltage, soc and estimated battery voltage from the car. After this, chademo is shut down and will restart when ccs reach end of ccs PreCharge.
When ccs reach CurrentDemand, then hopefully chademo is also ready and they will join in their respective charge loops.

Stop button/power off:
When idle, stop button will power off instantly.
When working (ccs logic started), a 5 sec. press will initiate power off. Led flash 300/300ms. Depending on how far it has progressed, it can power off instantly or after charging has stopped.
When noting else works, there is a hard power off mode, where a 30 sec. stop button press will just kill the power. Only do this as last resort, it may hurt the contactors if charging is active.
Auto power off after 5 minutes of not being active inside ccs CurrentDemand loop (WIP/could be smarter).

Led:
Initially, led flash 600/600ms.
When charging/after delivered amps, led flash 1200/1200ms.
When stop/power off pending, led flash 300/300ms.

Other:
There is a 5sec. watchdog that will reset (effectively power off) adapter if there is a hang.

Original firmware:
Original firmware seems to be based on open-plc-utils. I think it uses a rtos of some kind, with a preemtive scheduler.
Original firmware generally works well. It it missing several of the ccs shutdown mechanism (I struggle with both Tesla and Kempower), else I had little problems.
Even so, I found it very interesting to hack on this adapter so I made this. It is possible it will not work at all or as well as the original firmware. Be warned:-)
Happy hacking.

TODO:
Add more led blibk sequences to track progress.

# ccs32clara

![image](doc/clara_logo_colored.jpg) Hi, I'm Clara. I'm a piece of software, which was born in the OpenInverter forum community, https://openinverter.org/forum/viewtopic.php?t=3727, and I'm loving to grow due to the great people there.
Im running on an STM32, and I'm talking with a QCA7005 homeplug modem. All this, and some more components, brings my team-mate Foccci to you. Foccci is the hard(-ware) part of our powerful team. Let's charge!

![image](doc/foccci_and_clara_logo_colored.jpg)

## News / Change History / Functional Status

### 2024-01-24 Mini Light Bulb Demo adaptor works
Clara configured to demo-mode using the openinverter web interface box (https://github.com/jsphuebner/esp32-web-interface). Clara runs on Foccci in a 3D printed housing, which contains a CCS2 inlet, a German SchuKo outlet, a relay to connect/disconnect the Schuko from the CCS, red-green-blus LEDs for status indication, a stop-button and D-Sub CAN connector, a little arduino to show the hardware status of the CP line (https://github.com/uhi22/arduino-controlpilot-observer) and a 18650 Li-Ion accu including step-up from accu to 5V to supply the complete box.
![image](doc/2024-01-27_lightbulbdemo_off.jpg)
![image](doc/2024-01-27_lightbulbdemo_on.jpg)


### 2023-12-13 Supercharging works.
Johu is charging the Touran on the TESLA Supercharger: https://openinverter.org/forum/viewtopic.php?p=64563#p64563 and https://www.youtube.com/watch?v=OKg3VUslol8

### 2023-12-06 Charging works on ABB triple charger and Compleo
The liboi port has been tested inside the CCS-to-CHAdeMO adapter and successfully charged a few kWh on said chargers

### 2023-08-03 Charging works on public Alpitronic charger

With the STM32F103RE on the Foccci board, the light-bulb-demo-charging on Alpitronic hypercharger worked on the first attempt.
Pictures here: https://openinverter.org/forum/viewtopic.php?p=59821#p59821

### 2023-07-18 Charging loop reached

Using the NUCLEO F303RE development board, the STM32 talks via SPI to the QCA7005 on the Ioniq CCM. The ccs32clara reaches the charging
loop, and shows the charging progress on the serial console in the Cube IDE.

## Todos
- [x] Implement TCP retry to compensate for single lost packets
- [ ] Takeover latest state machine updates from pyPLC
- [x] Control the CP line and the contactor outputs
- [x] Add CAN
- [x] Migrate to STM32F103RE, which is planned for the foccci board
- [ ] (much more)

## Build Environment / Compiling

- arm-none-eabi-gcc
- Controller: STM32F103RE
- Installation of tool chain and flashing [Clara User Manual](doc/clara_user_manual.md)

## Cross References

* The Hyundai Ioniq/Kona Charge Control Module (CCM): https://github.com/uhi22/Ioniq28Investigations/tree/main/CCM_ChargeControlModule_PLC_CCS
* The ccs32berta "reference project" (which uses an ESP32, talking via SPI to a QCA7005: https://github.com/uhi22/ccs32berta
* The ccs32 "reference project" (which uses ethernet instead of SPI, hardware is an ESP32 WT32-ETH01): https://github.com/uhi22/ccs32
* Hardware board which integrates an STM32, QCA7005 and more: https://github.com/uhi22/foccci
* pyPLC as test environment: https://github.com/uhi22/pyPLC
* Discussion on openinverter forum: https://openinverter.org/forum/viewtopic.php?t=3727
* Similar project discussed on SmartEVSE github: https://github.com/SmartEVSE/SmartEVSE-3/issues/25#issuecomment-1608227152

# ccs32clara-chademo

![image](doc/ccs2-chademo_adapter.jpg)

Alternative firmware for Dongguan Longood CCS2 to CHAdeMO adapter that uses My407ccs2chademo.bin.
Adapter probably uses a stm32f407 cpu. It has a boot loader with support for firmware update from usb fat32, very nice.

It has an internal DC transformator to charge the batteries (pri: 210-1200V DC sec: 12.6V DC 2A). Transformer is connected to the car-side of the adapter.
It has 2 batteries. One short and one long. The long one seems to drive electronics. Guessing the short one drive contactors (one in the adapter itself + 2 in the car).
2 stop buttons, but they are the same GPIO for both.
It has 2 controllable leds, one external and one internal (invisible).
It has a QCA7000 powerline modem. It is wired up strangely, so can not use hw spi/dma, must use bitbang.
Some of the voltages are wired up to adc GPIOs. At least the 12v reading does not seem to match with what i measure on eg. Chademo d1 pin.

## Operation
Plug adapter into car. Plug cable into adapter. Power on adapter.
Adapter will start chademo to fetch targetVoltage, soc and estimated battery voltage (calculated from targetVoltage and soc).
Chademo will then shut down and ccs logic will start.
When ccs reach start of PreCharge, chademo is restarted. Estimated battery voltage is used as PreCharge target voltage.
When PreCharge reach this voltage, chademo d2 is set. We stall PreCharge and prevent it from completing until car closes contactors.
During this time, 0 volt has been presented to car both via chademo and on the wire (because adapter contactor not closed yet). It is a lie as we are actually on estimated battery voltage at this time.
After we set d2, car hopefully close contactors. After car close contactors, we close the adapter contactor and voila, we apperantly manifested battery voltage on the wire and via chademo instantly.
This is _not_ how chademo is supposed to work thou (I _think_ charger is supposed to quickly increase voltage from 0 volts to target voltage, immediately after car closes the contactors).

The problem is that ccs seems way to slow to emulate this kind of behaviour and also PreCharge is not necesarely supported from 0 volt and up, some jump directly to target.
At least I was unable to make the car close its contactors without using this trick. If we start chademo before PreCharge, close adapter contactor and set d2, when ccs is 0 volt, before PreCharge,
and try to bring chademo with us on the ride thru Precharge voltage rising, chademo would time out, because car close contactors ca. 1 second after we set d2, and start to ask for amps shortly after,
and if it does not get asked amps within ca. 4 seconds, it will fail. Getting from 0 volt before PreCharge and into CurrentDemand delivering amps on this short time, its only possible on some chargers.
Example, Tesla v2 can use well over 10 second in PreCharge alone. So a stable and reproducable solution may not be possible without the hack, but more investigation needed to be 100% sure.

Ccs otoh, works exactly like we simulate it AFAIK: the PreCharge voltage is set to battery voltage and when voltage is reached, car open its contactors (difference between charger and battery voltage is small).
The problem is, chademo does not expose the battery voltage, but we try to estimate is from target and soc...
So even thou chademo is not made for the charger and car to "meet" on battery voltage, and we do not really know the battery voltage, this _seems_ to work fine, but its hard to say if this has any issues (burnt relays etc.)
How I describe it is also how the original firmware works, AFAICT, allthou it seems to use a fixed nominal battery voltage of 350 volt and it uses chademo 0.9 so the timing may be different
(chademo 0.9 seem to be missing the flag that tell when car closes the contactor after d2 is set, so then have to use a fixed delay after setting d2 to close the adapter contactor, I think...).

## Stop button/power off
Shortly pressing stop button will initiate power off (pending).
Between ccs has started and Slac is done, stop button must be pressed for 5 seconds to initiate power off (pending). This to allow "fiddle" with the plug or late plug insertion.
As soon as Slac is done, the logic revert to "shortly pressing".
When power off is pending, adapter will power off as soon as both ccs and chademo logic says that the plug is unlocked (adapter does not have physical locks on the plugs, but logically).
When nothing else works, there is a hard power off mode, where a 30 sec. stop button press will just kill the power. Only do this as last resort, it may hurt the contactors if charging is active.
Auto power off after 3 minutes of not being inside ccs PreCharge or CurrentDemand loop (could be smarter).

Special mode: hold stop button while powering on. You should hear a click from the adapter contactor. Let go of the stop button within 1 second, and you have activated contactor unwelding attempt, where 
the contactor is rapidly closed/opened, until you press the stop button. If the contactor is welded/stuck, this may help, but you should test with a multimeter to make sure it is stuck and also use a multimeter during the 
process, to see if the contactor becomes unstuck again. My relay got stuck for some reason (be warned) and this is why I made this function, and it helped me, as within a few seconds the relay became unstuck.

## Led
<pre>
Initially, slow blinking [***************_______________]
When Slac is done, one blink [***_________]
When tcp connected, two blinks [***___***_________]
When PreCharge started, three blinks [***___***___***_________]
When PreCharge is done, but stalled waiting for chademo, four blinks [***___***___***___***_________]
When delivering amps, medium blinking [*****_____]
When stop/power off pending, fast blinking [*_]
</pre>

## Other
There is a 5sec. watchdog that will reset (effectively power off) adapter if there is a hung.
There is a welding detection logic that check if supply voltage is > 12 volt before adapter contactor is closed, in case, the contactor is probably welded.
You will see it in the log and the charging is aborted if this is detected. This check is not fool proof as it won't always show > 12 volt before, even if welded (depends on if charger supply the little current needed to drive the transformer).
If charging is started with welded contactor, the car will most likely (at least it did for me), display a warning and say the EV need service, and car was put into turtle mode! I used LeafSpy Pro to clear the DTCs, else I would probably need to visit a garage!
So yes, it happened to me. Not sure why, if it was by chance or if this firmware has a problem. So I suggest traveling with a ODB2 BT dongle, LeafSpy Pro and a multimeter, at least I do. Be warned.

## Original firmware
Original firmware seems to be based on open-plc-utils. I think it uses a rtos of some kind, with a preemtive scheduler.
For some reason it seem to emulate a chademo 0.9 charger and not chademo 1.0. Chademo 1.0 is better defined and works better IMO, so not supporting it in this firmware.
Original firmware generally works well. It it missing several of the ccs shutdown mechanism (I struggle with both Tesla and Kempower). Also it struggle with Slac some times, 
specially at Tesla stations, where I may have to unplug and plug the cable (fiddle) to get things started.
These things are improved in this firmware and this was what kicked off this project. But this firmware may have other problems that the original firmware does not have.
So it is possible it will not work at all or as well as the original firmware if you try it. Be warned.
Happy hacking.

## Download
Every commit is built automatically and can be downloaded here, as artifact of a workflow run: https://github.com/osexpert/ccs32clara-chademo/actions

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

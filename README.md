### Description
This is an alternative firmware for wireless 433MHz magnetic door/window reed sensors.  

Boards with STC15W101/104 8051 based processors + SYN115 radio transmitter are supported.  
The '101 model has 1KB flash and the '104 model has 4KB flash.  
Most boards but not all have a tamper detect switch installed.  
 
STC MCUs do not allow read/verify of written firmware.  
Therefore an open source alternative is needed to confirm program behavior.  
Also for this reason original firmware can not be reflashed once overwritten.  

This work is made possible by Vincent Defert's hardware abstraction layer for STC 8051 MCUs:
https://github.com/area-8051/uni-STC

### Programming Interface
Boards contain a header that may be populated with pins labeled:  
G (ground)  
T (transmit)  
R (receive)  
V (3.3 volts)  

Flashing just requires a 3.3V compatible USB to UART module.  
Alternatively the battery terminal may be powered and only G, T, R pins connected for flashing.  
The board needs fewer than 100 milliamps.  

### Receiver Hardware
Receiving radio packets requires a receiver.  

| Hardware | modifications | stock protocol | protocol 1 | reliable packets? |
| ------------- | ------------- | ------------- | ------------- | ------------- |
| Sonoff R1  | original  | untested | untested | todo |
| Sonoff R2 v1.0  | original  | yes | yes | todo |
| Sonoff R2 v1.0  | RF-Bridge-EFM8BB1  | yes | yes | todo |
| Sonoff R2 v1.0 | bypass EFM8BB1  | untested | untested | todo |
| Sonoff R2 v2.0 | original  | untested | untested | todo |
| Sonoff R2 v2.0  | RF-Bridge-EFM8BB1  | untested | untested | todo |
| Sonoff R2 v2.0 | bypass  | untested | untested | todo |
| Sonoff R2 v2.2  | original  | yes | yes | todo |
| Sonoff R2 v2.2 | bypass  | untested | yes | todo |
| SIGNALDuino  | original  | yes | yes | todo |


### Receiver Modifications (optional)

Generic controller with 433 MHz receiver:  
https://github.com/sui77/rc-switch
Fork that appears more recently updated:  
https://github.com/1technophile/rc-switch


For Sonoff RF Bridge R1/R2 v1.0 and v2.0 modifications are available but are not explicitly required if stock timings are used:  
https://github.com/xoseperez/espurna/wiki/Hardware-Itead-Sonoff-RF-Bridge
https://tasmota.github.io/docs/devices/Sonoff-RF-Bridge-433/#rf-firmware-upgrade


RF-Bridge-EFM8BB1 is not supported on Sonoff R2 v2.2 hardware:  
https://github.com/Portisch/RF-Bridge-EFM8BB1/issues/217

However a similar hardware modification for R2 v2.2 which bypasses the radio chip so that Tasmota directly decodes is available:  
https://community.home-assistant.io/t/new-sonoff-rf-bridge-board-need-flashing-help/344326/17

### Home automation support (optional)

[ESPurna](https://github.com/xoseperez/espurna "ESPurna") is nice because it treats wireless sensors as "virtual" sensors.  
Virtual sensors show up as permanent switch entities in Home Assistant.  
Also ESPurna can learn/remember unique sensor codes.  

[Tasmota](https://tasmota.github.io/docs/devices/Sonoff-RF-Bridge-433/ "Tasmota") firmware can also be flashed.  

ESPHome is also an option (link?).

### Features

| Proposed | original or added | status |
| ------------- | ------------- | ------------- |
| Transmit on reed switch open/close (interrupt)  | original  | DONE |
| Transmit on tamper switch open/close (interrupt)  | original  | DONE |
| Manage power modes  | original  | DONE |
| Support inverted protocols  | added  | DONE |
| Ability to specify timings for transmission protocol (e.g. rc-switch)  | added  | DONE |
| "Heart beat" mode for periodic transmission   | added  | DONE |
| Add tamper closed key  | added  | DONE |
| Add tamper "trip" mode   | added  | DONE |
| Interrupt and heart beat mode for low battery   | original  | DONE |
| Add battery OK code  | added  | DONE |
| Support stock transmission protocols  | added  | DONE |
| Add packet count to upper bits of transmitted rf code    | added  | DONE |
| Allow inverting switch state (open sends "close" code etc.)  | added  | todo |
| User configuration/input with tamper switch press(es) | added  | todo |
| Compare power usage to original firmware  | added  | todo |
| Adjustable LED blink behavior   | added  | todo |
| Adjustable heart beat time period  | added  | todo |
| Store settings in EEPROM  | added  | todo |


![alt text](/photos/water_leak_store_hookup_example.jpg "Wireless 433 MHz Door Sensor")

### Installation
```
# install https://sdcc.sourceforge.net/ for your platform

# switch to base of home directory
cd ~/

# flashing tool
git clone https://github.com/area-8051/stcgal-patched.git

# hardware abstraction layer for stc
git clone https://github.com/area-8051/uni-STC.git
cd uni-STC/demos/

# this repository
git clone https://github.com/mightymos/ReedTripRadio.git
cd ReedTripRadio/

# build
make

# flash over usb to uart (serial)
make upload
```

### Flash premade hex (.ihx)
```
# manual flash (WINDOWS under MSYS2)
# (LINUX: substitue port for example with "/dev/ttyUSB0")
# (note: we set very low baud rates here to eliminate that source of flashing problems)
~/stcgal-patched/stcgal.py -p COM3 -l 1200 -b 1200 -t 24000 ReedTripRadio.ihx
```

### Non free flasher
[STC-ISP] (http://www.stcmcudata.com/)  

### Wireless door/window sensor
| Source | Link | Price (USD) |
| ------------- | ------------- | ------------- |
| aliexpress  | https://www.aliexpress.com/item/3256803337417240.html  | $4.09 (12/05/2022) |
| aliexpress  | https://www.aliexpress.com/item/2255800593881608.html  | $4.38 (1/31/2023) |
| amazon(de)  | https://www.amazon.de/-/en/Aiggend-Magnetic-433M-H-Z-Wireless-Detector/dp/B07Z93Q7NL/  | €11.99 (2/2/2023) |
| amazon(de)  | https://www.amazon.de/-/en/Nikou-Magnetic-Sensor-Switch-Wireless/dp/B07PTYD171/  | €11.49 (2/2/2023) |


### Future plans

As of now, adding more additional features is unlikely.
This is because it was decided to keep firmware under 1KB to support model '101 MCUs/boards.  
It would be possible to add features to the 4KB model '104 MCUs of course.  

One additional possibility is to use emulated EEPROM area on either MCU as code space.  
Experiments with booting from other memory spaces have not worked so far:  
https://github.com/mightymos/stc15bootisp/issues/1 

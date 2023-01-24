### Description
This is an alternative firmware for wireless 433MHz magnetic door/window reed sensors.

STC15W101/104 are 8051 based processors + SYN115 radio transmitter.  
'101 model has 1KB flash.  
'104 model has 4KB flash and circuit board also has a tamper detect switch.
 
Instead of supporting software serial or full hardware abstraction layer (HAL),  
it is decided to keep firmware under 1KB so it works on multiple processors/boards.  

One possibility however is to use emulated EEPROM area for code space.  
Experiments with booting from other memory spaces have not worked so far:  
https://github.com/mightymos/stc15bootisp/issues/1  

Finally, STC processors do not allow read/verify of written firmware.  
Therefore an open source alternative is needed to confirm program behavior.  
Also for this reason original firmware can not be reflashed once overwritten.  

Boards contain a header that may be populated with pins labeled with G (ground), T (transmit), R (receive), and V (3.3 volts) for flashing with USB to UART module.
Alternatively the battery terminal maybe powered and only G, T, R pins connected for flashing.
The board needs fewer than 100 milliamps.


### Receiver Hardware
Receiving radio packets requires a receiver.  
We have chosen "protocol 1" from the rc-switch library.  
You can use a generic 433 MHz receiver and controller using [rc-switch](https://github.com/sui77/rc-switch).  
There is also an rc-switch fork that appears more recently updated [rc-switch by 1technophile] (https://github.com/1technophile/rc-switch).


Another option is the Sonoff RF Bridge 433 MHz.  

For Sonoff R2 V1.0 one of the two methods is required so that "protocol 1" is available [see Modifications section in link below].  
https://github.com/xoseperez/espurna/wiki/Hardware-Itead-Sonoff-RF-Bridge

Flashing Portisch is described here:  
https://tasmota.github.io/docs/devices/Sonoff-RF-Bridge-433/#rf-firmware-upgrade



Portisch is not supported on Sonoff R2 V2.2 hardware:  
https://github.com/Portisch/RF-Bridge-EFM8BB1/issues/217

However apparently there is a hardware modification for R2 V2.2 which bypasses rf chip so Tasmota directly decodes:  
https://community.home-assistant.io/t/new-sonoff-rf-bridge-board-need-flashing-help/344326/17

[ESPurna](https://github.com/xoseperez/espurna "ESPurna") is nice because it treats wireless sensors as "virtual" sensors.  
Virtual sensors show up as permanent switch entities in Home Assistant.  
Also ESPurna can learn/remember unique sensor codes.  


[Tasmota](https://tasmota.github.io/docs/devices/Sonoff-RF-Bridge-433/ "Tasmota") firmware can also be flashed.  


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
| Send code if pin goes low indicating low battery   | original  | DONE |
| Add packet count to upper bits of transmitted rf code    | added  | todo |
| Store settings in EEPROM  | added  | todo |
| Send information over radio (e.g., settings?, battery?)  | added  | todo |
| Compare power usage to original firmware  | added  | todo |
| Test other transmission protocols  | added  | todo |
| Adjustable LED blink behavior   | added  | todo |
| Adjustable sleep behavior  | added  | todo |
| User configuration/input with tamper switch press(es) | added  | todo |

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

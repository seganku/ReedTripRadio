/*
    Alternative firmware for STC15W101/4 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

#include <delay.h>

// just do not have much more space for HAL with 1KB flash code size
//#include <eeprom-hal.h>
//#include <gpio-hal.h>
//#include <power-hal.h>
//#include <timer-hal.h>

#ifndef MCU_HAS_WAKE_UP_TIMER
    // Shouldn't happen, unless using an STC12.
    #error "The selected MCU doesn't have a power-down wake-up timer."
#endif // MCU_HAS_WAKE_UP_TIMER

// check for tamper switch pressed at startup, if so enter bootloader mode
#define CHECK_FOR_TAMPER_AT_STARTUP true
//#define CHECK_FOR_TAMPER_AT_STARTUP false


// hardware pin definitions
//   circuit is voltage divider and high side transistor with processor controlling divider
//   pin 3.0 is also connected with receive pin on header 
#define RADIO_VDD      P3_0
// LED is attached to P3.1 for door sensor RF 433 MHz (STC15W101 and STC15W104 processor, SYN115 RF transmitter)
// pin 3.1 is also connected with transmit pin on header
#define LED_PIN        P3_1
#define REED_SWITCH    P3_2
#define TAMPER_SWITCH  P3_3
// ASK modulation to RF chip
#define RADIO_ASK      P3_4
// on my board when variable power supply (stand in for battery)
//   reaches just above one volt from starting point of 1.5 volts
//   this signal line goes from high (3.3V) to low (0V)
#define BATTERY_DETECT P3_5


// radio protocol apparently requires repeated transmissions so it is accepted at receiver
// portisch firmware for EFM8BB1 seems to require only repeating twice
// original firmware repeats at least three times (oscilloscope runs out of memory)
// @bismosa reports stock repeats transmission twenty times!
// stock EFM8BB1 seems to require four retransmissions
// but again rc-switch protocol 1 is not supported with stock apparently
// mmotley999 reported up to eight retransmissions are required
#define REPEAT_TRANSMISSIONS 4
//#define REPEAT_TRANSMISSIONS 8


// milliseconds
#define RADIO_STARTUP_TIME 120

// milliseconds
#define RADIO_GUARD_TIME 10

// tens of microseconds
#define DEBOUNCE_TIME_10US 20

// maximum is 32768 as per sec. 7.8 power down wake-up special timer
// highest bit of register will be set to enable wake up timer
// maximum is 0x7FFF and then highest bit set

// pg. 551, sec. 7.8 power down wake up special timer
// this equates to about 16 seconds according to examples
// note: why does setting to 0xFFFF wake up too quickly, about every two seconds?
#define SLEEP_TIME_0 0xfffe

// milliseconds
#define CONTROLLER_STARTUP_TIME 200

// array size
#define SWITCH_HISTORY_SIZE 16

// unique ID location in flash
//  1K MCU (e.g. STC15W101)
// #define ID_ADDR_ROM 0x03f9
//  4K MCU (eg. STC15W104)
//#define ID_ADDR_ROM 0x0ff9

// unique ID is stored in ram in the same locations on mcu 101/104 parts
// see makefile - we are limiting ram size so that it is not initialized at these addresses
// it is possible to pull id from flash space, but it is at different locations on different sized parts
// note: the values stored in ram at these locations should match the last four characters shown by Target UID when flashing
#define ID0_ADDR_RAM 0x76
#define ID1_ADDR_RAM 0x77


// point at ram locations to derive uniqe id for inclusion in radio messages as original firmware did
// see sec. 1.12 global unique identification number STC15-English.pdf
static const __idata unsigned char *guid0 = (__idata unsigned char*) ID0_ADDR_RAM;
static const __idata unsigned char *guid1 = (__idata unsigned char*) ID1_ADDR_RAM;


// codes used in original firmware
// (we leave upper bits available for possibly adding transmission count in future)
// low voltage code reported by https://community.home-assistant.io/t/budget-priced-wife-friendly-wireless-wall-switch-using-433mhz-and-mqtt/88281
// if sensor is being powered by header pins this will also show low apparently
static const unsigned char battery_low  = 0x06;
static const unsigned char tamper_open  = 0x07;
static const unsigned char reed_open    = 0x0A;
static const unsigned char reed_close   = 0x0E;
// added close code to support tamper closed
// note: if we resend only a generic tamper code (e.g., 0x07) too quickly due to any switch change (e.g., a press and then a release)
//       I think the duplicate code sent may be discarded at receiver
static const unsigned char tamper_close = 0x08;
// added battery okay code if anyone will use it
static const unsigned char battery_ok   = 0x09;


// rc-switch project timings (https://github.com/sui77/rc-switch)
// static const struct Protocol protocols[] = {
  // { 350, {  1, 31 }, {  1,  3 }, {  3,  1 }, false },    // protocol 1
  // { 650, {  1, 10 }, {  1,  2 }, {  2,  1 }, false },    // protocol 2
  // { 100, { 30, 71 }, {  4, 11 }, {  9,  6 }, false },    // protocol 3
  // { 380, {  1,  6 }, {  1,  3 }, {  3,  1 }, false },    // protocol 4
  // { 500, {  6, 14 }, {  1,  2 }, {  2,  1 }, false },    // protocol 5
  // { 450, { 23,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 6 (HT6P20B)
  // { 150, {  2, 62 }, {  1,  6 }, {  6,  1 }, false },    // protocol 7 (HS2303-PT, i. e. used in AUKEY Remote)
  // { 200, {  3, 130}, {  7, 16 }, {  3,  16}, false},     // protocol 8 Conrad RS-200 RX
  // { 200, { 130, 7 }, {  16, 7 }, { 16,  3 }, true},      // protocol 9 Conrad RS-200 TX
  // { 365, { 18,  1 }, {  3,  1 }, {  1,  3 }, true },     // protocol 10 (1ByOne Doorbell)
  // { 270, { 36,  1 }, {  1,  2 }, {  2,  1 }, true },     // protocol 11 (HT12E)
  // { 320, { 36,  1 }, {  1,  2 }, {  2,  1 }, true }      // protocol 12 (SM5212)
// };

// changed pulse lengths from microseconds to 10 microseconds units
// because hardware abstraction layer provides delay10us() 
// I avoid performing multiplies on processor by computing these as constants
// "protocol 1" is the first rc-switch protocol which also supported by Portisch
// (https://github.com/Portisch/RF-Bridge-EFM8BB1/blob/master/inc/RF_Protocols.h)
const uint16_t gPulseHigh =   35;
const uint16_t gPulseLow  = 1085;
const uint16_t gZeroHigh  =   35;
const uint16_t gZeroLow   =  105;
const uint16_t gOneHigh   =  105;
const uint16_t gOneLow    =   35;

// stock sensor timings (see door_sensor_reverse_notes_fv1.txt)
// (tested working on Sonoff Bridge R2 V1.0)
// (does not work on stock Sonoff Bridge R2 V2.2)
//const uint16_t gPulseHigh =   47;
//const uint16_t gPulseLow  = 1363;
//const uint16_t gZeroHigh  =   47;
//const uint16_t gZeroLow   =  136;
//const uint16_t gOneHigh   =  136;
//const uint16_t gOneLow    =   47;

// it saves code space to just specify a single polarity and use define here
// uncomment only one line immediately below
#define PROTOCOL_INVERTED false
//#define PROTOCOL_INVERTED true


// save switch states checked in interrupts for use in main loop
struct Flags {
    volatile bool reedInterrupted;
    volatile bool tamperInterrupted;
    volatile bool lowBatteryInterrupted;
    volatile bool tamperTripped;
    volatile unsigned char eventHistory[SWITCH_HISTORY_SIZE];
    volatile unsigned char eventCount;
};

// arrays are not initialized
// should be alright because we only read when count is incremented
struct Flags flag = {
    .reedInterrupted       = false, 
    .tamperInterrupted     = false,
    .lowBatteryInterrupted = false,
    .tamperTripped         = false,
    .eventCount = 0
};

struct Settings {
    bool reedHeartbeatEnabled;
    bool tamperHeartbeatEnabled;
    bool batteryHeartbeatEnabled;
    bool interruptsEnabled;
    bool tamperTripEnabled;
};

// default is to pick least frequent wake up time and disable radio heartbeats to save power
struct Settings setting = {
    // only enable tamper heartbeat or tamper trip, not both
    .tamperHeartbeatEnabled  = false,
    // if you are worried about missing codes, periodically send by enabling heart beat and (maybe) disable interrupts
    .reedHeartbeatEnabled    = false,
    // note that the interrupt to detect battery low is always available
    .batteryHeartbeatEnabled = false,
    // recommend only enabling interrupts or heart beat modes, but not both (due to potential for missed codes)
    .interruptsEnabled       = true,
    // this is intended to keep transmitting if someone is fooling with housing (tamper is opened), until tamper is again closed
    .tamperTripEnabled       = true
};


// only enable power to radio when we are going to modulate ASK pin (i.e., send data)
// low pin setting enables transistor which supplies power to radio chip
inline void enable_radio_vdd(void)
{
    RADIO_VDD = 0;
}

// pin setting functions are more readable than direct pin setting
// and avoid making errors (e.g., "enabling" something is actually setting pin equal to zero)
inline void disable_radio_vdd(void)
{
    RADIO_VDD = 1;
}

// specify inline to save some flash code space
inline void radio_ask_high()
{
    #if PROTOCOL_INVERTED
        RADIO_ASK = 0;
    #else
        RADIO_ASK = 1;
    #endif
}

inline void radio_ask_low()
{
    #if PROTOCOL_INVERTED
        RADIO_ASK = 1;
    #else
        RADIO_ASK = 0;
    #endif
}

// led is controlled by transistor which essentially inverts pin output
// (so low level enables transistor and then LED is on)
inline void led_on(void)
{
    LED_PIN = 0;
}

inline void led_off(void)
{
    LED_PIN = 1;
}

inline void enable_ext0(void)
{
    // clear so that interrrupt triggers on falling and rising edges (should be default)
    IT0 = 0;
    
    // enable external interrupt 0
    IE1 |= M_EX0;
}

inline void enable_ext1(void)
{
    // set default such that external interrupt is triggered on falling and rising edges
    IT1 = 0;
    
    // enable external interrupt 1
    IE1 |= M_EX1;
}

inline void enable_ext3(void)
{
    INT_CLKO |= M_EX3;
}

inline void enable_global_interrupts(void)
{
    // enable global interrupts
    EA = 1;
}

// allows integer delays with 10 microsecond function at the expense of accuracy
void delay10us_wrapper(unsigned int microseconds)
{
    const unsigned char step = 0xff;
    
    while (microseconds > step)
    {
        delay10us(step);
        microseconds -= step;
    }
    
    delay10us(microseconds);
}

/*! \brief 
 *
 */        
inline bool isTamperOpen(void)
{
    return TAMPER_SWITCH;
}

inline bool isReedOpen(void)
{    
    return REED_SWITCH;
}

// read resistive divider state
inline bool isBatteryLow(void)
{    
    return !BATTERY_DETECT;
}

void rfsyncPulse()
{
    // rf sync pulse
    radio_ask_high();
    delay10us_wrapper(gPulseHigh);
    
    // this should be the only really long delay required
    radio_ask_low();
    delay10us_wrapper(gPulseLow);
}

/*! \brief Description
 *         Tips [http://ww1.microchip.com/downloads/en/AppNotes/Atmel-9164-Manchester-Coding-Basics_Application-Note.pdf]
 *
 */  
void send(const unsigned char byte)
{
    const unsigned char numBits = 8;
    
    // mask out highest bit
    const unsigned char mask = 1 << (numBits - 1);
    
    unsigned char i = 0;

    // byte for shifting
    unsigned char toSend = byte;
    
    // Repeat until all bits sent
    for(i = 0; i < numBits; i++)
    {
        // mask out all but left most bit value, and if byte is not equal to zero (i.e. left most bit must be one) then send one level
        if((toSend & mask) > 0)
        {
            radio_ask_high();
            delay10us_wrapper(gOneHigh);
            
            radio_ask_low();
            delay10us_wrapper(gOneLow);
        }
        else
        {
            radio_ask_high();
            delay10us_wrapper(gZeroHigh);
            
            radio_ask_low();
            delay10us_wrapper(gZeroLow);
        }
        
        toSend = toSend << 1;
    }
}

void sendRadioPacket(const unsigned char rfcode)
{
    unsigned char index;
    
    enable_radio_vdd();
    
    // sec. 9, table datasheet output blanking VDD transition from low to high (500 microseconds)
    // oscillator startup time crystal HC49S (300 microseconds)
    // standby delay time (min, 30), typical(75), max(120) milliseconds
    delay1ms(RADIO_STARTUP_TIME);
    
    // sonoff or tasmota or espurna seems to require sending repeatedly to accept receipt
    for (index = 0; index < REPEAT_TRANSMISSIONS; index++)
    {
        rfsyncPulse();

        // send rf key with unique id and code
        send(*guid0);
        send(*guid1);
        send(rfcode);
    }
    
    disable_radio_vdd();
    
    // FIXME: we need to force ask low just in case correct?
}


//-----------------------------------------
//FIXME: handle reentrancy?
// interrupt and wake up on reed pin change (default is rising and falling edge)
void external_isr0(void) __interrupt 0
{
    flag.reedInterrupted = true;
}

//-----------------------------------------
// interrupt and wake up on tamper switch pin change
void external_isr1(void) __interrupt 2
{
    flag.tamperInterrupted = true;
}

//-----------------------------------------
// interrupt and wake up on battery state change
void external_isr3(void) __interrupt 11
{
    flag.lowBatteryInterrupted = true;
}

// sec. 4.1 All port pins default to quasi-bidirectional after reset. Each one has a Schmitt-triggered input for improved input noise rejection.
//batteryMonitor GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN5, GPIO_HIGH_IMPEDANCE_MODE)
//radioASK       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN4, GPIO_BIDIRECTIONAL_MODE)
//tamperSwitch   GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN3, GPIO_BIDIRECTIONAL_MODE)
//reedSwitch     GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN2, GPIO_BIDIRECTIONAL_MODE)
//ledPin         GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN1, GPIO_OPEN_DRAIN_MODE)
//radioVDD       GPIO_PIN_CONFIG(GPIO_PORT3, GPIO_PIN0, GPIO_OPEN_DRAIN_MODE)
inline void configure_pin_modes(void)
{
    // avoid bit operations to save on flash code space
    // just explicitly set gpio mode
    P3M1 = 0x23;
    P3M0 = 0x03;
}

// sec. 4.9.1 quasi-bidirectional i/o
// turn led off
// disable radio power
// enables weak pullups for reed and tamper switches assuming we are in bidirectional mode (pins high)
// note: do not use radio_ask_low() to disable power, because it might be inverted, so would actually enable power!
inline void startup_pins_state(void)
{
    P3 = 0x0f;
}

void main(void)
{
    // as per HAL instructions
    INIT_EXTENDED_SFR();
    
    // makes the code a little cleaner below
    bool state;
    bool flagCheck;
    
    // makes accessing array element more readable below
    unsigned char index;

    // setup gpio
    configure_pin_modes();
    startup_pins_state();
    
    // pulse LED at startup
    led_on();
    
    // give the microcontroller time to stabilize
    delay1ms(CONTROLLER_STARTUP_TIME);
    
    led_off();


#if CHECK_FOR_TAMPER_AT_STARTUP

    // if we are holding down tamper switch at power up, enter iap mode
    // this is intended to make entering flash mode easier on subsequent flashes
    // if the board has no tamper switch, then this will just never do anything
    // if a regular user is inserting a battery, it is unlikely that they are also pressing tamper switch
    //  so this should just be skipped over in that case
    // FIXME: on mcu 101 boards with or without tamper switch, this seems to get stuck, no idea why
    if (!isTamperOpen())
    {
        // set ISP boot bit and reset processor
        IAP_CONTR = 0x60;
    }
    
#endif // CHECK_FOR_TAMPER_AT_STARTUP
    
    // does exactly what the function name says
    enable_global_interrupts();


    // we always enable tamper and reed interrupts in hardware
    // but might ignore them depending on settings above
    enable_ext0();
    enable_ext1();
    
    // enable low battery detect interrupt
    enable_ext3();

    // Main loop -------------------------------------------------------
    while (true)
    {
        flagCheck  = setting.tamperHeartbeatEnabled;
        flagCheck |= setting.reedHeartbeatEnabled;
        flagCheck |= setting.batteryHeartbeatEnabled;
        flagCheck |= flag.tamperTripped;
        
        // only enable wake up timer if periodic wake up required, otherwise clear/disable
        // note: sec. 3.3.1 special function registers address map shows that default state of WKTCx has default value in timer
        if (flagCheck)
        {
            // set wake up count and enable wake up timer bit
            WKTC = SLEEP_TIME_0;
        } else {
            WKTC = 0x0000;
        }


        // go to sleep
        // program will either resume due to wake up timer (if enabled) or due to interrupt
        PCON |= M_PD;
        
        // sec. 2.3.3.1 demo program in datasheet and example HAL show providing nops() after power down
        NOP();
        NOP();
        
        // these checks are difficult to read, but allow use to avoid duplicate codes if only interrupt mode or only heart beat mode is enabled
        flagCheck  = setting.interruptsEnabled;
        flagCheck &= flag.reedInterrupted;

        // if we do too much in the interrupts themselves it produces too many stack pushes/pops
        // among other problems that are caused, so perform most logic here
        if (flagCheck)
        {
            // slope on reed switch from high to low is about 200 microseconds as measured on oscilloscope
            // FIXME: this may be a terrible debounce
            delay10us(DEBOUNCE_TIME_10US);
            
            // more readable for array indexing
            index = flag.eventCount;
            
            // save any switch (or later battery) changes in order of occurence
            // note: we really do not have code space for checking array overflows
            if (isReedOpen())
            {
                flag.eventHistory[index] = reed_open;
            } else {
                flag.eventHistory[index] = reed_close;
            }
            
            // track count because we might have multiple events in quick succession
            flag.eventCount++;
            
            flag.reedInterrupted = false;
            
        }
        
        
        flagCheck  = setting.interruptsEnabled;
        flagCheck &= flag.tamperInterrupted;
        
        // we do not check if reed occurred prior to tamper or vice versa but should not matter too much
        if (flagCheck)
        {            
            // FIXME: another terrible debounce
            delay10us(DEBOUNCE_TIME_10US);
            
            index = flag.eventCount;
            
            state = isTamperOpen();
            
            if (state)
            {
                flag.eventHistory[index] = tamper_open;
            } else {
                flag.eventHistory[index] = tamper_close;
            }
            
            flag.eventCount++;
            
            flag.tamperInterrupted = false;
            
            
            if (setting.tamperTripEnabled)
            {
                if (state)
                {
                    // indicate we should periodically send out tamper state until tamper closes again
                    flag.tamperTripped = true;
                }
            }
        }
        
        // interrupt will only trigger once as battery discharges, unless voltage bounces around
        if (flag.lowBatteryInterrupted)
        {
            if (isBatteryLow())
            {
                flag.eventHistory[flag.eventCount] = battery_low;
                flag.eventCount++;
            }
            
            flag.lowBatteryInterrupted = false;
                
        }
        
        // only allow tamper heart beat mode, if tamper trip mode is disabled
        flagCheck  =  setting.tamperHeartbeatEnabled;
        flagCheck &= !setting.tamperTripEnabled;
        
        // force sending out periodic messages to indicate tamper state
        if (flagCheck)
        {
            // send different keys depending on reading push button
            if (isTamperOpen())
            {            
                flag.eventHistory[flag.eventCount] = tamper_open;
            } else {
                flag.eventHistory[flag.eventCount] = tamper_close;
            }
            
            flag.eventCount++;
        }
        
        // only allow tamper trip, if tamper heart beat mode is disabled
        // 
        flagCheck  =  setting.tamperTripEnabled;
        flagCheck &= !setting.tamperHeartbeatEnabled;
        flagCheck &= flag.tamperTripped;
        
        // tamper trip is similar to heart beat mode, but does not send close states
        if (flagCheck)
        {
            // send different keys depending on reading push button
            if (isTamperOpen())
            {            
                flag.eventHistory[flag.eventCount] = tamper_open;
                flag.eventCount++;

            } else {
                flag.tamperTripped = false;
            }
            
        }
        
        // similar periodic message but for reed switch
        if (setting.reedHeartbeatEnabled)
        {
            if(isReedOpen())
            {
                flag.eventHistory[flag.eventCount] = reed_open;
            } else {
                flag.eventHistory[flag.eventCount] = reed_close;
            }
            
            flag.eventCount++;
        }
        
        if (setting.batteryHeartbeatEnabled)
        {
            if(isBatteryLow())
            {
                flag.eventHistory[flag.eventCount] = battery_low;
            } else {
                flag.eventHistory[flag.eventCount] = battery_ok;
            }
            
            flag.eventCount++;
        }
        
        
        // send any stored event(s) if count is incremented
        while (flag.eventCount > 0)
        {
            // this is smarter than using pulseLED() since pulsing would use blocking delays
            // (in other words, the delays in sending radio packet serve as our pulse "on" time)
            led_on();
            
            
            flag.eventCount--;
            
            sendRadioPacket(flag.eventHistory[flag.eventCount]);
            
            // delay before (if) sending next packet so as not to overwhelm receiver
            delay1ms(RADIO_GUARD_TIME);
            
            // effectively just pulsed led
            led_off();
        }
    }
}

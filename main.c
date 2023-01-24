/*
    Alternative firmware for STC15W101/4 processor + SYN115 radio transmitter door/window reed sensor(s)
    (see README)
 */
#include "project-defs.h"

#include <delay.h>
//#include <eeprom-hal.h>
//#include <gpio-hal.h>
//#include <power-hal.h>
//#include <timer-hal.h>

#ifndef MCU_HAS_WAKE_UP_TIMER
    // Shouldn't happen, unless using an STC12.
    #error "The selected MCU doesn't have a power-down wake-up timer."
#endif // MCU_HAS_WAKE_UP_TIMER


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
// stock EFM8BB1 seems to require four retransmissions
// but again rc-switch protocol 1 is not supported with stock apparently
// some users have reported up to eight retransmissions are required
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

// FIXME: needs to be calculated based on 24 MHz clock
// about 16 seconds
#define SLEEP_TIME_0 32766


// milliseconds
#define LED_HIGH_TIME 750
#define LED_LOW_TIME  100

// milliseconds
#define CONTROLLER_STARTUP_TIME 200

// array size
#define SWITCH_HISTORY_SIZE 8

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
    volatile bool reedIsOpen[SWITCH_HISTORY_SIZE];
    volatile bool tamperIsOpen[SWITCH_HISTORY_SIZE];
    volatile unsigned char tamperCount;
    volatile unsigned char reedCount;
};

// isOpen arrays are not initialized
// should be alright because we only read when count incremented
struct Flags flag = {
    .reedInterrupted   = false, 
    .tamperInterrupted = false,
    .tamperTripped     = false,
    .tamperCount = 0, 
    .reedCount   = 0
};

struct Settings {
    unsigned char eepromWritten;
    unsigned char protocol;
    uint16_t sleepTime;
    bool reedHeartbeatEnabled;
    bool tamperHeartbeatEnabled;
    bool tamperTripEnabled;
};

// default is to pick least frequent wake up time and disable radio heartbeats to save power
// convention with eeprom is often that uninitialized is 0xff
struct Settings setting = {
    .eepromWritten = 0xff, 
    .protocol = 0, 
    .sleepTime = SLEEP_TIME_0,
    .tamperHeartbeatEnabled  = false, 
    .reedHeartbeatEnabled    = false,
    .tamperTripEnabled       = false
};


// only enable power to radio when we are going to modulate ASK pin (i.e., send data)
// low pin setting enables transistor which supplies power to radio chip
inline void enable_radio_vdd(void)
{
    RADIO_VDD = 0;
}

// pin setting functions are more readable than direct pin setting
// and avoid making errors (e.g., "enabling" something is actually setting pin equal zero)
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

/*! \brief Purpose of pulsing is to avoid leaving LED on because simultaneously powering other radio pins may be exceeding port sink/source capability
 *         Brief description continued.
 *
 */
void pulseLED(void)
{
    led_on();
    delay1ms(LED_HIGH_TIME);
    
    led_off();
    delay1ms(LED_LOW_TIME);
}

/*! \brief 
 *
 */        
inline bool isTamperOpen(void)
{
    volatile bool pinState;
    
    pinState = TAMPER_SWITCH;
    
    return pinState;
}

inline bool isReedOpen(void)
{
    volatile bool pinState;
    
    pinState = REED_SWITCH;
    
    return pinState;
}

// read resistive divider state
inline bool isBatteryLow(void)
{
    volatile bool pinState;
    
    pinState = BATTERY_DETECT;
    
    return !pinState;
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
    unsigned char i = 0;
    const unsigned char numBits = 8;
    
    // mask out highest bit
    const unsigned char mask = 1 << (numBits - 1);
    
    // byte for shifting
    unsigned char toSend = byte;
    
    // Repeat until all bits sent
    for(i = 0; i < numBits; i++)
    {
        // check left most bit value, and if equal to one then send one level
        if((toSend & mask) == mask)
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
    
    // FIXME: the placement of this delay seems poor
    // FIXME: is this even necessary as compared with original firmware behavior?
    // delay before (if) sending next packet so as not to overwhelm receiver
    delay1ms(RADIO_GUARD_TIME);
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
// interrupt and wake up on tamper switch pin change
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

void main(void)
{
    // as per HAL instructions
    INIT_EXTENDED_SFR();
    
    // makes the code a little cleaner below
    bool tamperState;
    
    // allow possibility to flash multiple pulses
    unsigned char ledPulseCount = 0;

    // setup gpio
    configure_pin_modes();
    
    // if we are holding down tamper switch at power up, enter iap mode
    // this is intended to make entering flash mode easier on subsequent flashes
    // if the board has no tamper switch, then this will just never do anything
    // if a regular user is inserting a battery, it is unlikely that they are also pressing tamper switch
    //  so this should just be skipped over in that case
    while (!isTamperOpen())
    {
        // set ISP boot bit and reset processor
        IAP_CONTR = 0x60;
    }
    
    // pulse LED at startup
    pulseLED();
    
    // disable power to radio for power saving and to disallow any transmission
    disable_radio_vdd();
    
    // datasheet warns against applying (high) pulses to ASK pin while power is disabled
    radio_ask_low();
    
    // give the microcontroller time to stabilize
    delay1ms(CONTROLLER_STARTUP_TIME);

    
    // does exactly what the function name says
    enable_global_interrupts();


    // enable tamper and reed interrupts
    enable_ext0();
    enable_ext1();
    
    // enable low battery detect interrupt
    enable_ext3();

    // Main loop -------------------------------------------------------
    while (true)
    {
        // only enable wake up timer if any heartbeat is enabled
        if (setting.tamperHeartbeatEnabled || setting.reedHeartbeatEnabled || setting.tamperTripEnabled)
        {
            // FIXME: why does setting to 0x7FFF wake up every two seconds?
            // set wake up count
            WKTC = setting.sleepTime;
            
            // enable wake up timer
            WKTC |= 0x8000;
        }
        
        // do debouncing and switch state saving here rather than in interrupt
        //   one reason being that calling functions within interrupt produces too many pushes/pops to stack
        if (flag.reedInterrupted)
        {
            // slope on reed switch from high to low is about 200 microseconds as measured on oscilloscope
            // FIXME: this may be a terrible debounce
            delay10us(DEBOUNCE_TIME_10US);
            
            // FIXME: need to revisit if saving history is necessary or even desired
            if (flag.reedCount < SWITCH_HISTORY_SIZE)
            {
                flag.reedIsOpen[flag.reedCount] = isReedOpen();
                
                flag.reedCount++;
            }
            
            flag.reedInterrupted = false;
            
            ledPulseCount = 1;
        }
        
        if (flag.tamperInterrupted)
        {            
            // FIXME: another terrible debounce
            delay10us(DEBOUNCE_TIME_10US);
            
            tamperState = isTamperOpen();
            
            if (flag.tamperCount < SWITCH_HISTORY_SIZE)
            {
                flag.tamperIsOpen[flag.tamperCount] = tamperState;
                
                if (setting.tamperTripEnabled)
                {
                    if (tamperState)
                    {
                        flag.tamperTripped = true;
                    }
                }
            
                flag.tamperCount++;
            }
            
            flag.tamperInterrupted = false;
            
            ledPulseCount = 1;
        }
        
        // skip this (do not go to sleep) if unsent radio packets are available
        if ((flag.reedCount == 0) && (flag.tamperCount == 0))
        {
            // this will either wake up in the future due to timer (if enabled) or due to interrupt
            // datasheet and example HAL show providing nops() after power down
            PCON |= M_PD;
            NOP();
            NOP();
            
            // FIXME: need to disable wake up timer?
            WKTC &= ~0x8000;
        }
        

        // force sending out periodic messages to indicate tamper state
        if (setting.tamperHeartbeatEnabled)
        {
            // send different keys depending on reading push button
            if (isTamperOpen())
            {            
                sendRadioPacket(tamper_open);
            } else {
                sendRadioPacket(tamper_close);
            }
            
            // single pulse LED
            ledPulseCount = 1;
        }
        
        // similar periodic message but for reed switch
        if (setting.reedHeartbeatEnabled)
        {
            if(isReedOpen())
            {
                sendRadioPacket(reed_open);
            } else {
                sendRadioPacket(reed_close);
            }
            
            // notice that we do not increment count
            // so in effect, multiple radio packets could be sent and we just pulse once
            // and that is okay so that we save battery and avoid long unneeded delays
            ledPulseCount = 1;
        }
        
        // if trip setting enabled and one open tamper interrupt has occurred
        // keep sending out tamper radio message until tamper closes
        if (setting.tamperTripEnabled)
        {
            if (flag.tamperTripped)
            {
                sendRadioPacket(tamper_open);
                
                if (!isTamperOpen())
                {
                    flag.tamperTripped = false;
                }
            }
        }
        
        // FIXME: this will only trigger once as is
        if (flag.lowBatteryInterrupted)
        {
            sendRadioPacket(battery_low);
            
            flag.lowBatteryInterrupted = false;
                
            ledPulseCount = 1;
        }

        // send reed switch state if count is incremented by interrupt
        while (flag.reedCount > 0)
        {
            // track count because we might have multiple reed events in quick succession
            flag.reedCount--;
            
            if(flag.reedIsOpen[flag.reedCount])
            {
                sendRadioPacket(reed_open);
            } else {
                sendRadioPacket(reed_close);
            }
            
            // single pulse LED
            ledPulseCount = 1;
        }
 

        // difficult to capture tamper press releases, so need to track count to avoid misses
        while (flag.tamperCount > 0)
        {
            //
            flag.tamperCount--;
            
            if(flag.tamperIsOpen[flag.tamperCount])
            {
                sendRadioPacket(tamper_open);
            } else {
                sendRadioPacket(tamper_close);
            }
            
            // single pulse LED
            ledPulseCount = 1;
            

        }
        
        
        // finally blink LED here after sending out radio packets
        while (ledPulseCount > 0)
        {
            pulseLED();
            
            // decrementing (instead of zeroing) allows the potential
            // to pulse LED multiple times but usually we just pulse once
            ledPulseCount--;
        }
        

    }
}

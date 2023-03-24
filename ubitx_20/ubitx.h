/*************************************************************************
  header file for C++ by KD8CEC
-----------------------------------------------------------------------------
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
**************************************************************************/
#ifndef _UBITX_HEADER__
#define _UBITX_HEADER__

#include <Arduino.h>  //for Linux, On Linux it is case sensitive.

//==============================================================================
// Detailed debugging control
//==============================================================================
//#define COMMANDDEBUG
//#define DEBUGENCODER
//==============================================================================
// End Detailed debugging control
//==============================================================================


//==============================================================================
// Common Options Select for your onfiguration
//==============================================================================
//Ubitx BOARD Version   - Select one by uncommenting only it
//#define UBITX_BOARD_VERSION 3      //v1 ~ v4 : 4, v5: 5, 6
//#define UBITX_BOARD_VERSION 4
//#define UBITX_BOARD_VERSION 5
#define UBITX_BOARD_VERSION 6

//Define which PROCESSOR is used
#define NANO  
//#define NANOEVERY
//#define NANO33IOT
//#define NANOBLE
//#define NANORP2040
//#define TEENSY
//#define RASPBERRYPIPICO

//Depending on the type of LCD mounted on the uBITX, uncomment one of the options below.
//You must select only one.
//#define UBITX_DISPLAY_LCD1602P        //LCD mounted on unmodified uBITX (Parallel)
//#define UBITX_DISPLAY_LCD1602I        //I2C type 16 x 02 LCD
//#define UBITX_DISPLAY_LCD1602I_DUAL   //I2C type 16 x02 LCD Dual
//#define UBITX_DISPLAY_LCD2004P        //24 x 04 LCD (Parallel)
//#define UBITX_DISPLAY_LCD2004I        //I2C type 24 x 04 LCD
#define UBITX_DISPLAY_NEXTION         //NEXTION LCD

//Feature list - Mainly impacts the LCD and LCD Emulation on Nextion
// #define FUNCTIONS_ALL               //All features enabled
#define FUNCTIONS_NEXTION_NANO       //Max features for Nextion users that fit into a Nano V3.0 with old bootloader
// #define FUNCTIONS_NEXTION_BIG         //Features for Nextion users with bigger processors (i.e. non-Nano)
// #define FUNCTIONS_LCD               //Features oriented at LCD Users
//#define FUNCTIONS_TEST              //Not for users.Test configuration for developers

//You Can Select  Analog S-Meter or DSP (I2C) Meter (2nd Nano) Or Leave both Commented out
//#define USE_I2CSMETER         //This is the option to choose if using a second Nano
//#define USE_ANALOG_SMETER       //Select this option if attaching sensor directly to Radiuno


//==============================================================================
// End Common Options
//==============================================================================


//==============================================================================
//Error checking of Common options
//==============================================================================


#ifndef UBITX_BOARD_VERSION
  #error No uBITX Board Version Selected - Set in top of ubitx.h
#endif

#if !defined(NANO) && !defined(NANOEVERY) && !defined(NANO33IOT) && !defined(NANOBLE) && !defined(NANORP2040) \
    && !defined(TEENSY) && !defined(RASPBERRYPIPICO)
  #error No Processor Defined - Set in top of ubitx.h
#endif

#if !defined(UBITX_DISPLAY_LCD1602P) && !defined(UBITX_DISPLAY_LCD1602I) && !defined(UBITX_DISPLAY_LCD1602I_DUAL) \
    && !defined(UBITX_DISPLAY_LCD2004P) && !defined(UBITX_DISPLAY_LCD2004I) && !defined(UBITX_DISPLAY_NEXTION)
  #error No Display Defined  - Set in top of ubitx.h
#endif

#if !defined(FUNCTIONS_ALL) && !defined(FUNCTIONS_NEXTION_NANO) && !defined(FUNCTIONS_NEXTION_BIG) \
    && !defined(FUNCTIONS_LCD) && !defined(FUNCTIONS_TEST)
  #error No Functionality Level Defined - Set in top of ubitx.h
#endif

#if defined(USE_I2CSMETER) && defined(USE_ANALOG_SMETER) 
  #error Multiple S-Meter types defined. Select only one - Set in top of ubitx.h
#endif

//==============================================================================
// End Error checking of common options
//==============================================================================


//==============================================================================
// Processor Specific Options
//==============================================================================

#ifdef NANO
  #define PROCESSOR 1 
  #define ANALOGCHIPDEFAULT DEFAULT
  #define USE_SOFTWARESERIAL            // Use Software Serial library instead of hardware serial
  #define USE_I2C_EEPROM                // Use external EEPROM connected on I2C bus
#elif defined(NANOEVERY)
  #define PROCESSOR 2
  #define ANALOGCHIPDEFAULT DEFAULT
  #define USE_SOFTWARESERIAL            // Use Software Serial library instead of hardware serial
  #define USE_I2C_EEPROM                // Use external EEPROM connected on I2C bus
#elif defined(NANO33IOT)
  #define PROCESSOR 3
  #define ANALOGCHIPDEFAULT AR_DEFAULT
  #define INTEGERS_ARE_32_BIT
  #define USE_I2C_EEPROM                // Use external EEPROM connected on I2C bus
#elif defined(NANOBLE)
  #define PROCESSOR 4
  #define ANALOGCHIPDEFAULT AR_INTERNAL2V4
  #define INTEGERS_ARE_32_BIT
  #define USE_I2C_EEPROM                // Use external EEPROM connected on I2C bus
#elif defined(NANORP2040)
  #define PROCESSOR 5  
  #define INTEGERS_ARE_32_BIT
  #include <WiFiNINA.h>
  #define USE_I2C_EEPROM                // Use external EEPROM connected on I2C bus
#elif defined(TEENSY)
  #define PROCESSOR 6
  #define INTEGERS_ARE_32_BIT
  #define USE_I2C_EEPROM
  //#define USE_DIGITAL_ENCODER 
#elif defined(RASPBERRYPIPICO)
  #define PROCESSOR 7
  #define INTEGERS_ARE_32_BIT
  #define USE_I2C_EEPROM 
  #define USE_DIGITAL_ENCODER
#endif

//==============================================================================
// End Processor Specific Options
//==============================================================================


//==============================================================================
// Display Specific Options
//==============================================================================


#if defined(UBITX_DISPLAY_LCD1602P)         //LCD mounted on unmodified uBITX (Parallel)
  #define UBITXDISPLAY 1
#elif defined(UBITX_DISPLAY_LCD1602I)       //I2C type 16 x 02 LCD
  #define UBITXDISPLAY 2
  #define USE_I2C_LCD
#elif defined(UBITX_DISPLAY_LCD2004P)       //24 x 04 LCD (Parallel)
  #define UBITXDISPLAY 3
#elif defined(UBITX_DISPLAY_LCD1602I_DUAL)  //I2C type 16 x02 LCD Dual
  #define UBITXDISPLAY 4
  #define USE_I2C_LCD
#elif defined(UBITX_DISPLAY_LCD2004I)      //I2C type 24 x 04 LCD
  #define UBITXDISPLAY 5
  #define USE_I2C_LCD
#elif defined(UBITX_DISPLAY_NEXTION)        //NEXTION LCD
  #define UBITXDISPLAY 6
  #define NEXTIONBAUD 9600                  //must match that in tft file
  #undef ENABLE_ADCMONITOR
  #undef FACTORY_RECOVERY_BOOTUP  
  //#define UBITX_DISPLAY_NEXTION_SAFE      //Only EEProm Write 770~775
#endif  

#ifdef USE_I2C_LCD
  #define I2C_LCD_MASTER_ADDRESS_DEFAULT  0x27     //0x27  //DEFAULT, if Set I2C Address by uBITX Manager, read from EEProm
  #define I2C_LCD_SECOND_ADDRESS_DEFAULT  0x3F     //0x27  //only using Dual LCD Mode
#endif

//==============================================================================
// End Display Specific Options
//==============================================================================


//==============================================================================
// User Select feature list
//==============================================================================

#if defined(FUNCTIONS_ALL) 
  //Enable all features
  #define FUNCTIONALITY   1
  #define FN_BAND         1 //592
  #define FN_VFO_TOGGLE   1 //78
  #define FN_MODE         1 //20
  #define FN_RIT          1 //58
  #define FN_SPLIT        1 //62
  #define FN_IFSHIFT      1 //238
  #define FN_ATT          1 //128
  #define FN_CW_SPEED     1 //152
  #define FN_VFOTOMEM     1 //254
  #define FN_MEMTOVFO     1 //188
  #define FN_MEMORYKEYER  1 //156
  #define FN_WSPR         1 //1044
  #define FN_SDRMODE      1 //68
  #define FN_CALIBRATION  1 //666
  #define FN_CARRIER      1 //382
  #define FN_CWCARRIER    1 //346
  #define FN_CWTONE       1 //148
  #define FN_CWDELAY      1 //98
  #define FN_TXCWDELAY    1 //94
  #define FN_KEYTYPE      1 //168
  #define FN_ADCMONITOR   1 //516
  #define FN_TXONOFF      1 //58

#elif defined(FUNCTIONS_TEST)
  #define FUNCTIONALITY   2
  //Test Configuration  (88%)
  #define FN_BAND         0 //592
  #define FN_VFO_TOGGLE   0 //78
  #define FN_MODE         0 //20
  #define FN_RIT          0 //58
  #define FN_SPLIT        0 //62
  #define FN_IFSHIFT      0 //238
  #define FN_ATT          0 //128
  #define FN_CW_SPEED     1 //152
  #define FN_VFOTOMEM     0 //254
  #define FN_MEMTOVFO     0 //188
  #define FN_MEMORYKEYER  1 //156
  #define FN_WSPR         0 //1044
  #define FN_SDRMODE      1 //68
  #define FN_CALIBRATION  1 //666
  #define FN_CARRIER      1 //382 
  #define FN_CWCARRIER    1 //346
  #define FN_CWTONE       1 //148
  #define FN_CWDELAY      1 //98
  #define FN_TXCWDELAY    1 //94
  #define FN_KEYTYPE      1 //168
  #define FN_ADCMONITOR   1 //516 
  #define FN_TXONOFF      1 //58

#elif defined(FUNCTIONS_LCD)
  #define FUNCTIONALITY   3
  //Recommended Character LCD Developer  87%
  #define FN_BAND         1 //592
  #define FN_VFO_TOGGLE   1 //78
  #define FN_MODE         1 //20
  #define FN_RIT          1 //58
  #define FN_SPLIT        1 //62
  #define FN_IFSHIFT      1 //238
  #define FN_ATT          0 //128
  #define FN_CW_SPEED     0 //152 //using MM
  #define FN_VFOTOMEM     1 //254
  #define FN_MEMTOVFO     1 //188
  #define FN_MEMORYKEYER  1 //156
  #define FN_WSPR         1 //1044
  #define FN_SDRMODE      1 //68
  #define FN_CALIBRATION  0 //667 //using MM
  #define FN_CARRIER      0 //382 //using MM
  #define FN_CWCARRIER    0 //346 //using MM
  #define FN_CWTONE       0 //148 //using MM
  #define FN_CWDELAY      0 //98 //using MM
  #define FN_TXCWDELAY    0 //94 //using MM
  #define FN_KEYTYPE      0 //168 //using MM
  #define FN_ADCMONITOR   0 //516 //using MM
  #define FN_TXONOFF      1 //58

#elif defined(FUNCTIONS_NEXTION_NANO)
  #define FUNCTIONALITY   4
  //Recommended for Nextion, TJC LCD 88%
  #define FN_BAND         1 //600
  #define FN_VFO_TOGGLE   1 //90
  #define FN_MODE         1 //318
  #define FN_RIT          1 //62
  #define FN_SPLIT        1 //2
  #define FN_IFSHIFT      1 //358
  #define FN_ATT          1 //250
  #define FN_CW_SPEED     0 //286
  #define FN_VFOTOMEM     0 //276
  #define FN_MEMTOVFO     0 //234
  #define FN_MEMORYKEYER  1 //168
  #define FN_WSPR         0 //1130      //mjh was originally enabled in 1.2. But software has grown
  #define FN_SDRMODE      1 //70
  #define FN_CALIBRATION  0 //790
  #define FN_CARRIER      0 //500
  #define FN_CWCARRIER    0 //464
  #define FN_CWTONE       0 //158
  #define FN_CWDELAY      0 //108
  #define FN_TXCWDELAY    0 //106
  #define FN_KEYTYPE      0 //294
  #define FN_ADCMONITOR   0 //526 //not available with Nextion or Serial UI
  #define FN_TXONOFF      1 //70

#elif defined(FUNCTIONS_NEXTION_BIG)
  #define FUNCTIONALITY   5
 //Recommended for Nextion, TJC LCD 88%
  #define FN_BAND         1 //600
  #define FN_VFO_TOGGLE   1 //90
  #define FN_MODE         1 //318
  #define FN_RIT          1 //62
  #define FN_SPLIT        1 //2
  #define FN_IFSHIFT      1 //358
  #define FN_ATT          1 //250
  #define FN_CW_SPEED     0 //286
  #define FN_VFOTOMEM     0 //276
  #define FN_MEMTOVFO     0 //234
  #define FN_MEMORYKEYER  1 //168
  #define FN_WSPR         1 //1130
  #define FN_SDRMODE      1 //70
  #define FN_CALIBRATION  0 //790
  #define FN_CARRIER      0 //500
  #define FN_CWCARRIER    0 //464
  #define FN_CWTONE       0 //158
  #define FN_CWDELAY      0 //108
  #define FN_TXCWDELAY    0 //106
  #define FN_KEYTYPE      0 //294
  #define FN_ADCMONITOR   0 //526 //not available with Nextion or Serial UI
  #define FN_TXONOFF      1 //70
#endif

//==============================================================================
// End User Select feature list
//==============================================================================

//==============================================================================
// Track Status of S_Meter
//==============================================================================

#if defined(USE_I2CSMETER) 
  #define S_METER 1
#elif defined(USE_ANALOG_SMETER)
  #define S_METER 2
#else
  #define S_METER 3
#endif

//==============================================================================
// End Track Status of S_Meter
//==============================================================================

//==============================================================================
// Misc settings - don't mess with unless you know whet you are doing
//==============================================================================

//Use Internal or External EEPROM

#ifdef USE_I2C_EEPROM
  #define EXT_EEPROM_TYPE 1
  #define EEPROMTYPE  I2C_EEPROM
#else
  #define EXT_EEPROM_TYPE 0
  #define EEPROMTYPE  EEPROM
#endif

#ifdef USE_SOFTWARESERIAL  
  #include <SoftwareSerial.h>
  #define SERIAL_TYPE 0
  #define RX_PIN 8
  #define TX_PIN 9
  #define SERIALPORT sSERIAL
  SoftwareSerial SERIALPORT(RX_PIN, TX_PIN); // RX, TX
#else
  #define SERIAL_TYPE 1
  #define SERIALPORT Serial1                  //Using hardware serial
#endif  

#define MAXEEPROMSIZE 2048              //the Cat routines limit EEPROM addressing to 16bit offset and 16bit size (max 32kb) 
                                        //default binary file size for eeprom backup is 2048 bytes. So stick with this for
                                        //now until a reason to break compatibility.      


#define EXTEND_KEY_GROUP1               //MODE, BAND(-), BAND(+), STEP  //required to activate encoder menu system
//#define EXTEND_KEY_GROUP2             //Not in use


//Custom LPF Filter Mod
// See http://www.hamskey.com/2018/09/ubitx-setting-for-custmizedhacked-or.html
//#define USE_CUSTOM_LPF_FILTER           //LPF FILTER MOD

//#define ENABLE_FACTORYALIGN
#define FACTORY_RECOVERY_BOOTUP         //Whether to enter Factory Recovery mode by pressing FKey and turning on power
//#define ENABLE_ADCMONITOR             //Starting with Version 1.07, you can read ADC values directly from uBITX Manager. So this function is not necessary.


#define SMeterLatency   3               //1 is 0.25 sec

//==============================================================================
// End Misc settings - don't mess with unless you know whet you are doing
//==============================================================================



//==============================================================================
// Hardware, Define PIN Usage
//==============================================================================
/**
 * We need to carefully pick assignment of pin for various purposes.
 * There are two sets of completely programmable pins on the Raduino.
 * First, on the top of the board, in line with the LCD connector is an 8-pin connector
 * that is largely meant for analog inputs and front-panel control. It has a regulated 5v output,
 * ground and six pins. Each of these six pins can be individually programmed 
 * either as an analog input, a digital input or a digital output. 
 * The pins are assigned as follows (left to right, display facing you): 
 *      Pin 1 (Violet), A7, SPARE => Analog S-Meter
 *      Pin 2 (Blue),   A6, KEYER (DATA)
 *      Pin 3 (Green), +5v 
 *      Pin 4 (Yellow), Gnd
 *      Pin 5 (Orange), A3, PTT
 *      Pin 6 (Red),    A2, F BUTTON
 *      Pin 7 (Brown),  A1, ENC B
 *      Pin 8 (Black),  A0, ENC A
 *Note: A5, A4 are wired to the Si5351 as I2C interface 
 *       *     
 * Though, this can be assigned anyway, for this application of the Arduino, we will make the following
 * assignment
 * A2 will connect to the PTT line, which is the usually a part of the mic connector
 * A3 is connected to a push button that can momentarily ground this line. This will be used for RIT/Bandswitching, etc.
 * A6 is to implement a keyer, it is reserved and not yet implemented
 * A7 is connected to a center pin of good quality 100K or 10K linear potentiometer with the two other ends connected to
 * ground and +5v lines available on the connector. This implments the tuning mechanism
 */


#ifdef USE_DIGITAL_ENCODER
  #define ENCODER_TYPE 1
  #ifdef  RASPBERRYPIPICO
    #define ENC_A         17
    #define ENC_B         18
    #define FBUTTON       19
    #define PTT           22
  #else                         //currently only teensy
    #define ENC_A         14
    #define ENC_B         15
    #define FBUTTON       16
    #define PTT           17
  #endif
#else                           //Traditional Analog encoder
  #define ENCODER_TYPE 0  
  #define ENC_A         (A0)
  #define ENC_B         (A1)
  #define FBUTTON       (A2)
  #define PTT           (A3)
#endif

#ifdef RASPBERRYPIPICO
  #define ANALOG_KEYER  A0
  #define ANALOG_SPARE  A1
  #define ANALOG_SMETER A1  //by KD8CEC
  #define SDA_PIN       20
  #define SCL_PIN       21
  #define LCD_PIN_RS 1
  #define LCD_PIN_EN 0
  //
  // Set defaults for CW ADC
  //
  #define CWADCSTFROM_DEFAULT     0       //straight key
  #define CWADCSTTO_DEFAULT      50
  #define CWADCDOTFROM_DEFAULT  450     //dot key (left side, push right) pressed
  #define CWADCDOTTO_DEFAULT    550
  #define CWADCDASHFROM_DEFAULT 750     //dash keys (right side, push left) pressed
  #define CWADCDASHTO_DEFAULT   900
  #define CWADCBOTHFROM_DEFAULT 300      //both keys pressed
  #define CWADCBOTHTO_DEFAULT   425


#elif defined(NANO33IOT)
  #define ANALOG_KEYER  A6
  #define ANALOG_SPARE  A7
  #define ANALOG_SMETER A7  //by KD8CEC
  #define LCD_PIN_RS 8
  #define LCD_PIN_EN 9
  //
  // Set defaults for CW ADC
  //
  #define CWADCSTFROM_DEFAULT     0       //straight key
  #define CWADCSTTO_DEFAULT      50
  #define CWADCDOTFROM_DEFAULT  475     //dot key (left side, push right) pressed
  #define CWADCDOTTO_DEFAULT    600
  #define CWADCDASHFROM_DEFAULT 750     //dash keys (right side, push left) pressed
  #define CWADCDASHTO_DEFAULT   900
  #define CWADCBOTHFROM_DEFAULT 325     //both keys pressed
  #define CWADCBOTHTO_DEFAULT   425


#elif defined(TEENSY)
  #define ANALOG_KEYER  A6
  #define ANALOG_SPARE  A7
  #define ANALOG_SMETER A7  //by KD8CEC
  #define LCD_PIN_RS 0
  #define LCD_PIN_EN 1
  //
  // Set defaults for CW ADC
  //
  #define CWADCSTFROM_DEFAULT     0       //straight key
  #define CWADCSTTO_DEFAULT      50
  #define CWADCDOTFROM_DEFAULT  500     //dot key (left side, push right) pressed
  #define CWADCDOTTO_DEFAULT    600
  #define CWADCDASHFROM_DEFAULT 800     //dash keys (right side, push left) pressed
  #define CWADCDASHTO_DEFAULT   900
  #define CWADCBOTHFROM_DEFAULT 375     //both keys pressed
  #define CWADCBOTHTO_DEFAULT   499

#elif defined(NANOBLE)
  #define ANALOG_KEYER  A6
  #define ANALOG_SPARE  A7
  #define ANALOG_SMETER A7  //by KD8CEC
  #define LCD_PIN_RS 8
  #define LCD_PIN_EN 9
  //
  // Set defaults for CW ADC
  //
  #define CWADCSTFROM_DEFAULT     0       //straight key
  #define CWADCSTTO_DEFAULT      50
  #define CWADCDOTFROM_DEFAULT  500     //dot key (left side, push right) pressed
  #define CWADCDOTTO_DEFAULT    600
  #define CWADCDASHFROM_DEFAULT 800     //dash keys (right side, push left) pressed
  #define CWADCDASHTO_DEFAULT   900
  #define CWADCBOTHFROM_DEFAULT 400      //both keys pressed
  #define CWADCBOTHTO_DEFAULT   499

#else
  #define ANALOG_KEYER  A6
  #define ANALOG_SPARE  A7
  #define ANALOG_SMETER A7  //by KD8CEC
  #define LCD_PIN_RS 8
  #define LCD_PIN_EN 9
  //
  // Set defaults for CW ADC
  //
  #define CWADCSTFROM_DEFAULT   0       //straight key
  #define CWADCSTTO_DEFAULT      50
  #define CWADCDOTFROM_DEFAULT  301     //dot key (left side, push right) pressed
  #define CWADCDOTTO_DEFAULT    600
  #define CWADCDASHFROM_DEFAULT 601     //dash keys (right side, push left) pressed
  #define CWADCDASHTO_DEFAULT   700
  #define CWADCBOTHFROM_DEFAULT 51      //both keys pressed
  #define CWADCBOTHTO_DEFAULT   300

#endif


//  LCD Data pins are the same for all boards
#define LCD_PIN_D4  10
#define LCD_PIN_D5  11
#define LCD_PIN_D6  12
#define LCD_PIN_D7  13



/** 
 *  The second set of 16 pins on the Raduino's bottom connector are have the three clock outputs and the digital lines to control the rig.
 *  This assignment is as follows :
 *    Pin   1   2    3    4    5    6    7    8    9    10   11   12   13   14   15   16
 *         GND +5V CLK2  GND  GND  CLK1 GND  GND  CLK0  GND  D2   D3   D4   D5   D6   D7  
 *  These too are flexible with what you may do with them, for the Raduino, we use them to :
 *  - TX_RX line : Switches between Transmit and Receive after sensing the PTT or the morse keyer
 *  - CW_KEY line : turns on the carrier for CW
 */
#define TX_RX         (7)   //Relay
#define CW_TONE       (6)
#define TX_LPF_A      (5)   //Relay
#define TX_LPF_B      (4)   //Relay
#define TX_LPF_C      (3)   //Relay
#define CW_KEY        (2)

//******************************************************
//DSP (I2C) Meter 
//******************************************************
//S-Meter Address
#define I2CMETER_ADDR     0x58
//VALUE TYPE============================================
//Signal
#define I2CMETER_CALCS    0x59 //Calculated Signal Meter
#define I2CMETER_UNCALCS  0x58 //Uncalculated Signal Meter

//Power
#define I2CMETER_CALCP    0x57 //Calculated Power Meter
#define I2CMETER_UNCALCP  0x56 //UnCalculated Power Meter

//SWR
#define I2CMETER_CALCR    0x55 //Calculated SWR Meter
#define I2CMETER_UNCALCR  0x54 //Uncalculated SWR Meter

//==============================================================================
// for public, Variable, functions
//==============================================================================
#define WSPR_BAND_COUNT 3
#define TX_SSB          0
#define TX_CW           1
#define printLineF1(x) (printLineF(1, x))
#define printLineF2(x) (printLineF(0, x))

//0x00 : None, 0x01 : MODE, 0x02:BAND+, 0x03:BAND-, 0x04:TUNE_STEP, 0x05:VFO Toggle, 0x06:SplitOn/Off, 0x07:TX/ON-OFF,  0x08:SDR Mode On / Off, 0x09:Rit Toggle
#define FUNCTION_KEY_ADC  80  //MODE, BAND(-), BAND(+), STEP
#define FKEY_PRESS      0x78
#define FKEY_MODE       0x01
#define FKEY_BANDUP     0x02
#define FKEY_BANDDOWN   0x03
#define FKEY_STEP       0x04
#define FKEY_VFOCHANGE  0x05
#define FKEY_SPLIT      0x06
#define FKEY_TXOFF      0x07
#define FKEY_SDRMODE    0x08
#define FKEY_RIT        0x09

#define FKEY_ENTER      0x0A
#define FKEY_POINT      0x0B
#define FKEY_DELETE     0x0C
#define FKEY_CANCEL     0x0D

#define FKEY_NUM0       0x10
#define FKEY_NUM1       0x11
#define FKEY_NUM2       0x12
#define FKEY_NUM3       0x13
#define FKEY_NUM4       0x14
#define FKEY_NUM5       0x15
#define FKEY_NUM6       0x16
#define FKEY_NUM7       0x17
#define FKEY_NUM8       0x18
#define FKEY_NUM9       0x19

#define FKEY_TYPE_MAX   0x1F
//
//==============================================================================
// Useful Macros
//==============================================================================
//
#define GETDEFINEDVALUE(s) STRINGIFY(s)     //Used to get the value of a #define 
                                            //e.g. GETDEFINEDVALUE(ENC_A) allows you to see AO
#define STRINGIFY(s) #s                     //Used to get the string 
                                            //e.g., STRINGIFY(ENC_A) will return the string ENC_A
                                            
#define conv2BytesToInt32(lsb,msb) (int)((int16_t)((msb<<8) + lsb));
#define conv4BytesToLong(lsb,lsb1,lsb2,msb) (unsigned long)(((int)(msb<<24)) + ((int)(lsb2<<16)) + ((int)(lsb1<<8))+lsb);



//
//==============================================================================
// Extern definitions
//==============================================================================
//

extern byte I2C_LCD_MASTER_ADDRESS;     //0x27  //if Set I2C Address by uBITX Manager, read from EEProm
extern byte I2C_LCD_SECOND_ADDRESS;     //only using Dual LCD Mode

extern uint8_t SI5351BX_ADDR;     //change typical -> variable at Version 1.097, address read from eeprom, default value is 0x60
                                  //EEProm Address : 63
extern unsigned long frequency;
extern byte WsprMSGCount;
extern byte sMeterLevels[9];
extern int currentSMeter;         //ADC Value for S.Meter
extern byte scaledSMeter;         //Calculated S.Meter Level

extern byte KeyValues[16][3];     //Set : Start Value, End Value, Key Type, 16 Set (3 * 16 = 48)
extern byte TriggerBySW;   //Action Start from Nextion LCD, Other MCU

extern void printLine1(const char *c);
extern void printLine2(const char *c);
extern void printLineF(char linenmbr, const __FlashStringHelper *c);
extern void printLineFromEEPRom(char linenmbr, char lcdColumn, byte eepromStartIndex, byte eepromEndIndex, char offsetType);
extern byte delay_background(unsigned delayTime, byte fromType);
extern int btnDown(void);
extern char c[30];
extern char b[30];
extern int enc_read(void);
extern void si5351bx_init(void);
extern void si5351bx_setfreq(uint8_t clknum, uint32_t fout);
extern void si5351_set_calibration(int32_t cal);
extern void initOscillators(void);
extern void Set_WSPR_Param(void);
extern void TXSubFreq(unsigned long P2);

extern void startTx(byte txMode, byte isDisplayUpdate);
extern void stopTx(void);
extern void setTXFilters(unsigned long freq);

extern void SendWSPRManage(void);
extern char byteToChar(byte srcByte);
extern void DisplayCallsign(byte callSignLength);
extern void DisplayVersionInfo(const char* fwVersionInfo);

//I2C Signal Meter, Version 1.097
extern int GetI2CSmeterValue(int valueType);  //ubitx_ui.ino

#endif    //end of if header define

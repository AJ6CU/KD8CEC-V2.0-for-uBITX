/*************************************************************************
  header file for EEProm Address Map by KD8CEC
  It must be protected to protect the factory calibrated calibration.
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
#ifndef _UBITX_EEPOM_HEADER__
#define _UBITX_EEPOM_HEADER__

//==============================================================================
// Factory-shipped EEProm address
// (factory Firmware)
// Address : 0 ~ 31
//==============================================================================
#define MASTER_CAL            0
#define LSB_CAL               4
#define USB_CAL               8
#define SIDE_TONE             12
//these are ids of the vfos as well as their offset into the eeprom storage, don't change these 'magic' values
#define VFO_A                 16
#define VFO_B                 20
#define CW_SIDETONE           24
#define CW_SPEED              28

//==============================================================================
// The spare space available in the original firmware #1
// Address : 32 ~ 62
//==============================================================================
#define RESERVE_FOR_FACTORY1  32

//==============================================================================
// custom LPF Filter
// See http://www.hamskey.com/2018/09/ubitx-setting-for-custmizedhacked-or.html
// 48 : Using Custom LPF Filter  (48 = 0x57 or 0x58 => Using Custom LPF Filter, 0x58 = using A7 IO
// 49, 50 : LPF1  (49 : MHz  (~ Mhz), 50 : Enabled PIN
// 51, 52 : LPF2
// 53, 54 : LPF3
// 55, 56 : LPF4
// 57, 58 : LPF5
// 59, 60 : LPF6
// 61, 62 : LPF7
//==============================================================================
#define CUST_LPF_ENABLED 48
#define CUST_LPF_START   49

//SI5351 I2C Address (Version 1.097)
#define I2C_ADDR_SI5351       63

//==============================================================================
// The spare space available in the original firmware #2
// (Enabled if the EEProm address is insufficient)
// Address : 64 ~ 100
//==============================================================================
#define RESERVE_FOR_FACTORY2  64  //use Factory backup from Version 1.075
#define FACTORY_BACKUP_YN     64  //Check Backup //Magic : 0x13
#define FACTORY_VALUES        65  //65 ~ 65 + 32

//MJH Added so we can see specific Factory Values
#define FACTORY_VALUES_MASTER_CAL            65
#define FACTORY_VALUES_LSB_CAL               69
#define FACTORY_VALUES_USB_CAL               73
#define FACTORY_VALUES_SIDE_TONE             77       //not used in CEC
#define FACTORY_VALUES_VFO_A                 81
#define FACTORY_VALUES_VFO_B                 85
#define FACTORY_VALUES_CW_SIDETONE           89
#define FACTORY_VALUES_CW_SPEED              93



//==============================================================================
// KD8CEC EEPROM MAP
// Address : 101 ~ 1023
// 256 is the base address
// 256 ~ 1023 (EEProm Section #1)
// 255 ~ 101  (EEProm Section #2)
//==============================================================================

//0x00 : None, 0x01 : MODE, 0x02:BAND+, 0x03:BAND-, 0x04:TUNE_STEP, 0x05:VFO Toggle, 0x06:SplitOn/Off, 0x07:TX/ON-OFF,  0x08:SDR Mode On / Off, 0x09:Rit Toggle
#define EXTENDED_KEY_RANGE    140 //Extended Key => Set : Start Value, End Value, Key Type, 16 Set (3 * 16 = 48)

#define I2C_LCD_MASTER        190
#define I2C_LCD_SECOND        191

#define S_METER_LEVELS        230 //LEVEL0 ~ LEVEL7

#define ADVANCED_FREQ_OPTION1 240 //Bit0: use IFTune_Value, Bit1 : use Stored enabled SDR Mode, Bit2 : dynamic sdr frequency
#define IF1_CAL               241
#define ENABLE_SDR            242
#define SDR_FREQUNCY          243
#define CW_CAL                252

#define VFO_A_MODE            256
#define VFO_B_MODE            257
#define CW_DELAY              258
#define CW_START              259
#define HAM_BAND_COUNT        260    //
#define TX_TUNE_TYPE          261      //
#define HAM_BAND_RANGE        262    //FROM (2BYTE) TO (2BYTE) * 10 = 40byte
#define HAM_BAND_FREQS        302    //40, 1 BAND = 4Byte most bit is mode
#define TUNING_STEP           342   //TUNING STEP * 6 (index 1 + STEPS 5)  //1STEP : 

//for reduce cw key error, eeprom address
#define CW_ADC_MOST_BIT1      348   //most 2bits of  DOT_TO , DOT_FROM, ST_TO, ST_FROM
#define CW_ADC_ST_FROM        349   //CW ADC Range STRAIGHT KEY from (Lower 8 bit)
#define CW_ADC_ST_TO          350   //CW ADC Range STRAIGHT KEY to   (Lower 8 bit)
#define CW_ADC_DOT_FROM       351   //CW ADC Range DOT  from         (Lower 8 bit)
#define CW_ADC_DOT_TO         352   //CW ADC Range DOT  to           (Lower 8 bit)

#define CW_ADC_MOST_BIT2      353   //most 2bits of BOTH_TO, BOTH_FROM, DASH_TO, DASH_FROM
#define CW_ADC_DASH_FROM      354   //CW ADC Range DASH from         (Lower 8 bit)
#define CW_ADC_DASH_TO        355   //CW ADC Range DASH to           (Lower 8 bit)
#define CW_ADC_BOTH_FROM      356   //CW ADC Range BOTH from         (Lower 8 bit)
#define CW_ADC_BOTH_TO        357   //CW ADC Range BOTH to           (Lower 8 bit)
#define CW_KEY_TYPE           358
#define CW_DISPLAY_SHIFT      359   //Transmits on CWL, CWU Mode, LCD Frequency shifts Sidetone Frequency. 
                                    //(7:Enable / Disable //0: enable, 1:disable, (default is applied shift)
                                    //6 : 0 : Adjust Pulus, 1 : Adjust Minus
                                    //0~5: Adjust Value : * 10 = Adjust Value (0~300)
#define COMMON_OPTION0        360   //0: Confirm : CW Frequency Shift
                                    //1 : IF Shift Save
#define IF_SHIFTVALUE         363

#define DISPLAY_OPTION1       361   //Display Option1
#define DISPLAY_OPTION2       362   //Display Option2

// Following defines were active on 10/9/2022 to document where some of the wspr
// variables are stored. These defines are not actually used at this point
// because ubitx.wspr.ino has hardcoded knowledge of these locations and didn't share ;-)

#define WSPR_BAND1_TXFREQ     401   // location for frequency of band 1, 4 byte number
#define WSPR_BAND1_MULTICHAN  405   // Seems to be always 500. offset freq for second TX? 2 bytes
#define WSPR_BAND1_REG1       407   // REG1 - 5 bytes
#define WSPR_BAND1_REG2       412   // REG1 - 3 bytes

#define WSPR_BAND2_TXFREQ     415   // location for frequency of band 2, 4 byte number
#define WSPR_BAND2_MULTICHAN  419   // Seems to be always 500. offset freq for second TX? 2 bytes
#define WSPR_BAND2_REG1       421   // REG1 - 5 bytes
#define WSPR_BAND2_REG2       426   // REG1 - 3 bytes

#define WSPR_BAND3_TXFREQ     429   // location for frequnecy of band 3, 4 byte number
#define WSPR_BAND3_MULTICHAN  433   // Seems to be always 500. offset freq for second TX? 2 bytes
#define WSPR_BAND3_REG1       435   // REG1 - 5 bytes
#define WSPR_BAND3_REG2       440   // REG1 - 3 bytes

#define WSPR_COUNT            443   //WSPR_MESSAGE_COUNT
#define WSPR_MESSAGE1         444   //
#define WSPR_MESSAGE2         490   //
#define WSPR_MESSAGE3         536   //
#define WSPR_MESSAGE4         582   //

#define CHANNEL_FREQ          630   //Channel 1 ~ 20, 1 Channel = 4 bytes
#define CHANNEL_DESC          710   //Channel 1 ~ 20, 1 Channel = 4 bytes
#define EXTERNAL_DEVICE_OPT1  770   //for External Device 4byte
#define EXTERNAL_DEVICE_OPT2  774   //for External Device 2byte

//Check Firmware type and version
#define FIRMWAR_ID_ADDR       776 //776 : 0x59, 777 :0x58, 778 : 0x68 : Id Number, if not found id, erase eeprom(32~1023) for prevent system error.
#define FIRMWARE_ID_ADDR1       776 //Magic better be: 0x59
#define FIRMWARE_ID_ADDR2       777 //Magic better be: 0x58
#define FIRMWARE_ID_ADDR3       778 //Magic better be: 0x68  if these 3 numbers don't match eeprom has not been initialized



#define VERSION_ADDRESS       779   //check Firmware version
//USER INFORMATION
#define USER_CALLSIGN_KEY     780   //0x59 decimal 89
#define USER_CALLSIGN_LEN     781   //1BYTE (OPTION + LENGTH) + CALLSIGN (MAXIMUM 18)
#define USER_CALLSIGN_DAT     782   //CALL SIGN DATA  //direct EEPROM to LCD basic offset

//AUTO KEY STRUCTURE
//AUTO KEY USE 800 ~ 1023
#define CW_AUTO_MAGIC_KEY     800   //0x73 decimal 115
#define CW_AUTO_COUNT         801   //0 ~ 25  This is the count of number of CW messages in EEPROM
                                    //Max messages is 25 iwth Labels 0-9 then A-O
#define CW_AUTO_DATA          803   //Data structure is a pair of start/end up to CW_AUTO_COUNT (max 25)
                                    //CW_AUTO_DATA + BEG of the pair is the first position of the Mesg key Text
                                    //
                                    //For example, if there are 2 messages:
                                    //801 (CW_AUTO_COUNT) contains 2
                                    //803,804 Beg/End Msg 0  "CQ" 803=4, 804=5
                                    //805,806 Beg/End Msg 1  "TU" 805=6, 806=7
                                    //807 Start of messge for Msg 0 803 + Contents of 803 (4)
                                    //809 Start of text for Msg 1 803 + Contenst of 805)
                                    //All upper case text
    

#define CW_DATA_OFSTADJ       CW_AUTO_DATA - USER_CALLSIGN_DAT   //offset adjust for ditect eeprom to lcd (basic offset is USER_CALLSIGN_DAT
#define CW_STATION_LEN        1023  //value range : 4 ~ 30

// This is the start of the EEPROM locations for EEPROMS > 1024
#define EXT_FIRMWARE_ID_ADDR  1024  //1024 : 0x59, 1025 :0x58, 1026 : 0x68 : Id Number, if not found id, erase eeprom(1024-end of eeprom) for prevent system error.
#define EXT_FIRMWARE_ID_ADDR1 1024  //Magic better be: 0x59
#define EXT_FIRMWARE_ID_ADDR2 1025  //Magic better be: 0x58
#define EXT_FIRMWARE_ID_ADDR3 1026  //Magic better be: 0x68

#define EXT_FIRMWARE_VERSION_INFO 1027         //max 10 characters, space filed on right
#define EXT_RELEASE_NAME          1037         //max 15 characters, space filled on right
#define EXT_DATE_TIME_STAMP       1052        // 12 byes. This is the software build date in format mmddyyyyhhmm


#define EXT_UBITX_BOARD_VERSION 1064  //Board version that software was built for (3,4,5,6)
#define EXT_PROCESSOR_TYPE    1065  // 1 byte. 1=Nano,2=Nano Every, 3=NanoIOT, 4=Nano BLE, 5=Nano RP2040, 6=Teensy 4, 7=RaspberryPiPico
#define EXT_DISPLAY_TYPE      1066  // 1 byte. 1=16x2 Parallel, 2=16x2 I2C, 3= 20x4 Parallel, 4=Dual 16x2 I2C, 5=20x4 I2C, 6= Nextion
#define EXT_FUNCTIONALITY_SET 1067  // 1 byte. 1=All functionality, 2=Test functionality, 3=Recommended for LCD's, 
                                    // 4=Nano/Nextion with new bootloader and expanded flash, 5=None - If you
                                    // Are using a older Nano, this is your only choice, 6=Recommended for Nano Every/Nextion and above
#define EXT_SMETER_SELECTION  1068  // 1 byte. 1=I2C S-meter on second Nano. 2=Analog S-meter connected to primary processor, 3=None

#define EXT_SERIAL_TYPE       1069  //1 byte. 0=Software Serial, 1=Hardware Serial
#define EXT_EEPROM_TYPE       1070  //1 byte. 0 = Internal, 1 = External I2C EEPROM
#define EXT_ENCODER_TYPE      1071  //1 byte. 0 = Analog, 1 = Digital

#define EXT_NEXTIONBAUD       1072  // 6 bytes.

#define TWOSPAREBYTES         1078  // use them wisely...           

#define EXT_ENC_A             1080  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_ENC_B             1085  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_FBUTTON           1090  // 5 byte. Pin name using processor pin def. space filled on right

#define EXT_PTT               1095  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_ANALOG_KEYER      1100  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_ANALOG_SMETER     1105  // 5 byte. Pin name using processor pin def. space filled on right 



#define EXT_LCD_PIN_RS        1110  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_LCD_PIN_EN        1115  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_LCD_PIN_D4        1120  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_LCD_PIN_D5        1125  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_LCD_PIN_D6        1130  // 5 byte. Pin name using processor pin def. space filled on right
#define EXT_LCD_PIN_D7        1135  // 5 byte. Pin name using processor pin def. space filled on right

#define EXT_SOFTWARESERIAL_RX_PIN 1140 //5 byte Pin name using processor pin def. space filled on right
#define EXT_SOFTWARESERIAL_TX_PIN 1145 //5 byte Pin name using processor pin def. space filled on right



#endif    //end of if header define


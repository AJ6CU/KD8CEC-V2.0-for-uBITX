
/*************************************************************************
  KD8CEC's uBITX Display Routine for Nextion LCD
  
  Uses the default protocol of Nextion LCD.
  Do not assign a 2 byte address to Nextion LCD.
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
#include "ubitx.h"
#include "ubitx_lcd.h"

//======================================================================== 
//Begin of Nextion LCD Library by KD8CEC
//========================================================================
#ifdef UBITX_DISPLAY_NEXTION
/*************************************************************************
  Nextion Library for uBItX
  KD8CEC
**************************************************************************/

#ifdef USE_SOFTWARESERIAL  
  #include <SoftwareSerial.h>

  #define RX_PIN 8
  #define TX_PIN 9
  #define SERIALPORT sSERIAL
  SoftwareSerial SERIALPORT(RX_PIN, TX_PIN); // RX, TX
#else
  #ifdef USE_HARDWARESERIAL
     #define SERIALPORT Serial1
  #endif
#endif  

#define SS_MAX_RX_BUFF 50 // RX buffer size  was 35
#define PRINT_MAX_LENGTH 30
static uint8_t swr_receive_buffer[SS_MAX_RX_BUFF];




#define TEXT_LINE_LENGTH 20
char softBuffLines[2][TEXT_LINE_LENGTH + 1];
char softBuffSended[2][TEXT_LINE_LENGTH + 1];
char softBuffTemp[TEXT_LINE_LENGTH + 1];    //for STR Command

char c[30], b[30];
char softBuff[20];
char softTemp[20];

void LCDNextion_Init()
{
  delay(3000);
  SERIALPORT.begin(NEXTIONBAUD);
  memset(softBuffLines[0], ' ', TEXT_LINE_LENGTH); 
  softBuffLines[0][TEXT_LINE_LENGTH + 1] = 0x00;
  memset(softBuffLines[1], ' ', TEXT_LINE_LENGTH);
  softBuffLines[1][TEXT_LINE_LENGTH + 1] = 0x00;
}

void LCD_Init(void)
{
  LCDNextion_Init();  
}

//===================================================================
//Begin of Nextion LCD Protocol
//
// v0~v9, va~vz : Numeric (Transceiver -> Nextion LCD)
// s0~s9  : String (Text) (Transceiver -> Nextion LCD)
// vlSendxxx, vloxxx: Reserve for Nextion (Nextion LCD -> Transceiver)
//
//===================================================================
#define CMD_NOW_DISP      '0' //c0
char L_nowdisp = -1;          //Sended nowdisp

#define CMD_VFO_TYPE      'v' //cv
char L_vfoActive;             //vfoActive

#define CMD_CURR_FREQ     'c' //vc
unsigned long L_vfoCurr;      //vfoA
#define CMD_CURR_MODE     'c' //cc
byte L_vfoCurr_mode;          //vfoA_mode

#define CMD_VFOA_FREQ     'a' //va
unsigned long L_vfoA;         //vfoA
#define CMD_VFOA_MODE     'a' //ca
byte L_vfoA_mode;             //vfoA_mode

#define CMD_VFOB_FREQ     'b' //vb
unsigned long L_vfoB;         //vfoB
#define CMD_VFOB_MODE     'b' //cb
byte L_vfoB_mode;             //vfoB_mode

#define CMD_IS_RIT        'r' //cr
char L_ritOn;
#define CMD_RIT_FREQ      'r' //vr
unsigned long L_ritTxFrequency; //ritTxFrequency

#define CMD_IS_TX         't' //ct
char L_inTx;

#define CMD_IS_DIALLOCK   'l' //cl
byte L_isDialLock;            //byte isDialLock

#define CMD_IS_SPLIT      's' //cs
byte  L_Split;            //isTxType
#define CMD_IS_TXSTOP     'x' //cx
byte  L_TXStop;           //isTxType

#define CMD_TUNEINDEX     'n' //cn
byte L_tuneStepIndex;     //byte tuneStepIndex

#define CMD_SMETER        'p' //cs
byte L_scaledSMeter;      //scaledSMeter

#define CMD_SIDE_TONE     't' //vt
unsigned long L_sideTone; //sideTone
#define CMD_KEY_TYPE      'k' //ck
byte L_cwKeyType = -1;          //L_cwKeyType 0: straight, 1 : iambica, 2: iambicb

#define CMD_CW_SPEED      's' //vs
unsigned int L_cwSpeed;   //cwSpeed

#define CMD_CW_DELAY      'y' //vy
byte L_cwDelayTime=-1;       //cwDelayTime

#define CMD_CW_STARTDELAY 'e' //ve
byte L_delayBeforeCWStartTime=-1;  //byte delayBeforeCWStartTime

#define CMD_ATT_LEVEL     'f' //vf
byte L_attLevel;

byte L_isIFShift;             //1 = ifShift, 2 extend
#define CMD_IS_IFSHIFT    'i' //ci

int L_ifShiftValue;
#define CMD_IFSHIFT_VALUE 'i' //vi

byte L_sdrModeOn;
#define CMD_SDR_MODE      'j' //cj

#define CMD_UBITX_INFO     'm' //cm  Complete Send uBITX Information

//Once Send Data, When boot
//arTuneStep, When boot, once send
//long arTuneStep[5];
#define CMD_AR_TUNE1      '1' //v1
#define CMD_AR_TUNE2      '2' //v2
#define CMD_AR_TUNE3      '3' //v3
#define CMD_AR_TUNE4      '4' //v4
#define CMD_AR_TUNE5      '5' //v5


#define CMD_IS_CW_SHIFT_DISPLAY 'h' //ch
byte L_isShiftDisplayCWFreq;  //byte isShiftDisplayCWFreq

#define CMD_CW_SHIFT_ADJUST     'h' //vh
int L_shiftDisplayAdjustVal;        //int shiftDisplayAdjustVal

//0:CW Display Shift Confirm, 1 : IFshift save
#define CMD_COMM_OPTION     'o'     //vo
byte L_commonOption0;         //byte commonOption0

//0:Line Toggle, 1 : Always display Callsign, 2 : scroll display, 3 : s.meter
#define CMD_DISP_OPTION1    'p'   //vp
byte L_displayOption1;            //byte displayOption1
#define CMD_DISP_OPTION2    'q'   //vq
byte L_displayOption2;            //byte displayOption2 (Reserve)

#define CMD_TEXT_LINE0      '0'   //s0
#define CMD_TEXT_LINE1      '1'   //s1

#define CMD_CW_TEXT         'a'   //sa
#define CMD_CALLSIGN        'c'   //sc
#define CMD_VERSION         'v'   //sv

#define TS_CMD_MODE           1
#define TS_CMD_FREQ           2
#define TS_CMD_BAND           3
#define TS_CMD_VFO            4
#define TS_CMD_SPLIT          5
#define TS_CMD_RIT            6
#define TS_CMD_TXSTOP         7
#define TS_CMD_SDR            8
#define TS_CMD_LOCK           9 //Dial Lock
#define TS_CMD_ATT           10 //ATT
#define TS_CMD_IFS           11 //IFS Enabled
#define TS_CMD_IFSVALUE      12 //IFS VALUE
#define TS_CMD_STARTADC      13
#define TS_CMD_STOPADC       14
#define TS_CMD_SPECTRUMOPT   15 //Option for Spectrum
#define TS_CMD_SPECTRUM      16 //Get Spectrum Value
#define TS_CMD_TUNESTEP      17 //Get Spectrum Value
#define TS_CMD_WPM           18 //Set WPM
#define TS_CMD_KEYTYPE       19 //Set KeyType

#define TS_CMD_SWTRIG        21 //SW Action Trigger for WSPR and more
#define TS_CMD_READMEM       31 //Read EEProm
#define TS_CMD_WRITEMEM      32 //Write EEProm
#define TS_CMD_LOOPBACK0     74 //Loopback1 (Response to Loopback Channgel)
#define TS_CMD_LOOPBACK1     75 //Loopback2 (Response to Loopback Channgel)
#define TS_CMD_LOOPBACK2     76 //Loopback3 (Response to Loopback Channgel)
#define TS_CMD_LOOPBACK3     77 //Loopback4 (Response to Loopback Channgel)
#define TS_CMD_LOOPBACK4     78 //Loopback5 (Response to Loopback Channgel)
#define TS_CMD_LOOPBACK5     79 //Loopback6 (Response to Loopback Channgel)
#define TS_CMD_FACTORYRESET  85 //Factory Reset
#define TS_CMD_UBITX_REBOOT  95 //Reboot

char nowdisp = 0;

#define SWS_HEADER_CHAR_TYPE 'c'  //1Byte Protocol Prefix
#define SWS_HEADER_INT_TYPE  'v'  //Numeric Protocol Prefex
#define SWS_HEADER_STR_TYPE  's'  //for TEXT Line compatiable Character LCD Control


//Control must have prefix 'v' or 's'
char softSTRHeader[11] = {'p', 'm', '.', 's', '0', '.', 't', 'x', 't', '=', '\"'};
char softINTHeader[10] = {'p', 'm', '.', 'v', '0', '.', 'v', 'a', 'l', '='};

//send data for Nextion LCD
void SendHeader(char varType, char varIndex)
{
  if (varType == SWS_HEADER_STR_TYPE)
  {
    softSTRHeader[4] = varIndex;
    for (int i = 0; i < 11; i++)
      SERIALPORT.write(softSTRHeader[i]);
  }
  else
  {
    softINTHeader[4] = varIndex;
    for (int i = 0; i < 10; i++)
      SERIALPORT.write(softINTHeader[i]);
  }
}

#define INT_ETX 0
#define STR_ETX 1
#define TMP_ETX 2
//Send 0xFF, 0xFF, 0xFF
//etxType : INT_ETX = 0xFF, 0xFF, 0xFF
//          STR_ETX = ", 0xFF, 0xFF, 0xFF
//          TEMP_ETX = softTemp, 0xFF, 0xFF, 0xff

void SendCommandETX(char etxType)
{
  if (etxType == 2)
  {
    SERIALPORT.print(softTemp);
  }
  else if (etxType == 1)
  {
    SERIALPORT.print("\"");
  }
  
  SERIALPORT.write(0xff);
  SERIALPORT.write(0xff);
  SERIALPORT.write(0xff);
}

void SendCommandUL(char varIndex, unsigned long sendValue)
{
  SendHeader(SWS_HEADER_INT_TYPE, varIndex);

  memset(softTemp, 0, 20);

#ifdef INTEGERS_ARE_32_BIT          //mjh  auto cast of olong to int
  utoa(sendValue, softTemp, DEC);
#else
  ultoa(sendValue, softTemp, DEC);
#endif

  SendCommandETX(TMP_ETX);
}

void SendCommandL(char varIndex, long sendValue)
{
  SendHeader(SWS_HEADER_INT_TYPE, varIndex);

  memset(softTemp, 0, 20);

#ifdef INTEGERS_ARE_32_BIT
//  utoa(sendValue, softTemp, DEC);  //mjh appears to auto cast long to int
  itoa(sendValue, softTemp, DEC);
#else
  ltoa(sendValue, softTemp, DEC);
#endif
  
 
  SendCommandETX(TMP_ETX);
}

void SendCommandStr(char varIndex, char* sendValue)
{
  SendHeader(SWS_HEADER_STR_TYPE, varIndex);
  
  SERIALPORT.print(sendValue);
  SendCommandETX(STR_ETX);
}

//Send String data with duplicate check
void SendTextLineBuff(char lineNumber)
{
  //Check Duplicated data
  if (strcmp(softBuffLines[lineNumber], softBuffSended[lineNumber]))
  {
    SendHeader(SWS_HEADER_STR_TYPE, lineNumber + 0x30);  //s0.txt, s1.txt
  
    SERIALPORT.print(softBuffLines[lineNumber]);
    SendCommandETX(STR_ETX);
    
    strcpy(softBuffSended[lineNumber], softBuffLines[lineNumber]);
  }
}

void SendTextLineStr(char lineNumber, const char* sendValue)
{
  int i = 0;
  for (i = 0; i < 16; i++)
  {
    if (sendValue[i] == 0x00)
      break;
    else
      softBuffLines[lineNumber][i] = sendValue[i];
  }
  
  for (;i < 20; i++)
  {
    softBuffLines[lineNumber][i] = ' ';
  }

  softBuffLines[lineNumber][TEXT_LINE_LENGTH + 1] = 0x00;
  SendTextLineBuff(lineNumber);
}

void SendEEPromData(char varIndex, int16_t eepromStartIndex, int16_t eepromEndIndex, char offsetTtype) 
{
  SendHeader(SWS_HEADER_STR_TYPE, varIndex);
  
  for (int i = eepromStartIndex; i <= eepromEndIndex; i++)
  {
      SERIALPORT.write(EEPROMTYPE.read((offsetTtype == 0 ? USER_CALLSIGN_DAT : WSPR_MESSAGE1) + i));
  }

  SendCommandETX(STR_ETX);
}

uint8_t softBuff1Num[14] = {'p', 'm', '.', 'c', '0', '.', 'v', 'a', 'l', '=', 0, 0xFF, 0xFF, 0xFF};
void SendCommand1Num(char varType, char sendValue) //0~9 : Mode, nowDisp, ActiveVFO, IsDialLock, IsTxtType, IsSplitType
{
  softBuff1Num[4] = varType;
  softBuff1Num[10] = sendValue + 0x30;

  for (int i = 0; i < 14; i++)
    SERIALPORT.write(softBuff1Num[i]);
}

void SetSWActivePage(char newPageIndex)
{
    if (L_nowdisp != newPageIndex)
    {
      L_nowdisp = newPageIndex;
      SendCommand1Num(CMD_NOW_DISP, L_nowdisp);
    }
}
//===================================================================
//End of Nextion LCD Protocol
//===================================================================

// The generic routine to display one line on the LCD 
void printLine(unsigned char linenmbr, const char *c) {
  
  SendTextLineStr(linenmbr, c);
}

void printLineF(char linenmbr, const __FlashStringHelper *c)
{
  int i;
  char tmpBuff[21];
  PGM_P p = reinterpret_cast<PGM_P>(c);  

  for (i = 0; i < 21; i++){
    unsigned char fChar = pgm_read_byte(p++);
    tmpBuff[i] = fChar;
    if (fChar == 0)
      break;
  }

  printLine(linenmbr, tmpBuff);
}

#define LCD_MAX_COLUMN 20
void printLineFromEEPRom(char linenmbr, char lcdColumn, byte eepromStartIndex, byte eepromEndIndex, char offsetTtype) 
{
  int colIndex = lcdColumn;
  for (byte i = eepromStartIndex; i <= eepromEndIndex; i++)
  {
    if (++lcdColumn <= LCD_MAX_COLUMN)
      softBuffLines[linenmbr][colIndex++] = EEPROMTYPE.read((offsetTtype == 0 ? USER_CALLSIGN_DAT : WSPR_MESSAGE1) + i);
    else
      break;
  }

  SendTextLineBuff(linenmbr);
}

//  short cut to print to the first line
void printLine1(const char *c)
{
  printLine(1,c);
}
//  short cut to print to the first line
void printLine2(const char *c)
{
  printLine(0,c);
}

void clearLine2()
{
  printLine2("");
  line2DisplayStatus = 0;
}

//  short cut to print to the first line
void printLine1Clear(){
  printLine(1,"");
}
//  short cut to print to the first line
void printLine2Clear(){
  printLine(0, "");
}

void printLine2ClearAndUpdate(){
  printLine(0, "");
  line2DisplayStatus = 0;  
  updateDisplay();
}

//==================================================================================
//End of Display Base Routines
//==================================================================================

//==================================================================================
//Begin of User Interface Routines
//==================================================================================
//Main Display for Nextion LCD
//unsigned long 
byte nowPageIndex = 0;

//sendType == 1 not check different 
void sendUIData(int sendType)
{
  char nowActiveVFO = vfoActive == VFO_A ? 0 : 1;

  //#define CMD_VFO_TYPE      'v' //cv
  if (L_vfoActive != nowActiveVFO)
  {
    L_vfoActive = nowActiveVFO;
    SendCommand1Num(CMD_VFO_TYPE, L_vfoActive);
  }

  //#define CMD_CURR_FREQ     'c' //vc
  if (L_vfoCurr != frequency)
  {
    L_vfoCurr = frequency;
    SendCommandUL(CMD_CURR_FREQ, frequency);
  }

  //#define CMD_CURR_MODE     'c' //cc
  byte vfoCurr_mode = modeToByte();
  if (L_vfoCurr_mode != vfoCurr_mode)
  {
    L_vfoCurr_mode = vfoCurr_mode;
    SendCommand1Num(CMD_CURR_MODE, L_vfoCurr_mode);
  }

  //if auto cw key mode, exit
  //if (isCWAutoMode != 0 || menuOn != 0)
  if (isCWAutoMode != 0)
    return;

  //nowPageIndex = 0;
  if (menuOn==0)
  {
    if (sendType == 0)
    {
      SetSWActivePage(0);
    }
    else
    {
      SetSWActivePage(0);
    }
  }
  else
  {
    //Text Line Mode
      SetSWActivePage(1);
  }

  //#define CMD_VFOA_FREQ     'a' //va
  //VFOA
  if (L_vfoA != vfoA)
  {
    L_vfoA = vfoA;
    SendCommandUL(CMD_VFOA_FREQ, L_vfoA);
  }

  //#define CMD_VFOA_MODE     'a' //ca
  if (L_vfoA_mode != vfoA_mode)
  {
    L_vfoA_mode = vfoA_mode;
    SendCommand1Num(CMD_VFOA_MODE, L_vfoA_mode);
  }

  //#define CMD_VFOB_FREQ     'b' //vb
  //VFOB
  if (L_vfoB != vfoB)
  {
    L_vfoB = vfoB;
    SendCommandUL(CMD_VFOB_FREQ, L_vfoB);
  }

  //#define CMD_VFOB_MODE     'b' //cb
  if (L_vfoB_mode != vfoB_mode)
  {
    L_vfoB_mode = vfoB_mode;
    SendCommand1Num(CMD_VFOB_MODE, L_vfoB_mode);  
  }

  //byte isDialLock = ((isTxType & 0x01) == 0x01) ? 1 : 0;
  if (L_isDialLock != isDialLock)
  {
    L_isDialLock = isDialLock;
    SendCommand1Num(CMD_IS_DIALLOCK, L_isDialLock);  
  }

  //#define CMD_IS_RIT        'r' //cr
  if (L_ritOn != ritOn)
  {
    L_ritOn = ritOn;
    SendCommand1Num(CMD_IS_RIT, L_ritOn);  
  }
  
  //#define CMD_RIT_FREQ      'r' //vr
  //unsigned long L_ritTxFrequency; //ritTxFrequency
  if (L_ritTxFrequency != ritTxFrequency)
  {
    L_ritTxFrequency = ritTxFrequency;
    SendCommandUL(CMD_RIT_FREQ, L_ritTxFrequency);  
  }

  //#define CMD_IS_TX         't' //ct
  //char L_inTx;
  if (L_inTx != inTx)
  {
    L_inTx = inTx;
    SendCommand1Num(CMD_IS_TX, L_inTx);  
  }

  //#define CMD_IS_DIALLOCK   'l' //cl
  //byte L_isDialLock;            //byte isDialLock
  if (L_isDialLock != isDialLock)
  {
    L_isDialLock = isDialLock;
    SendCommand1Num(CMD_IS_DIALLOCK, L_isDialLock);  
  }

  //#define CMD_IS_SPLIT      's' //cs
  //byte  L_Split;            //isTxType
  if (L_Split != splitOn)
  {
    L_Split = splitOn;
    SendCommand1Num(CMD_IS_SPLIT, L_Split);  
  }
  

  //#define CMD_IS_TXSTOP     'x' //cx
  byte isTXStop = ((isTxType & 0x01) == 0x01);
  if (L_TXStop != isTXStop)
  {
    L_TXStop = isTXStop;
    SendCommand1Num(CMD_IS_TXSTOP, L_TXStop);
  }

  //#define CMD_TUNEINDEX     'n' //cn
  if (L_tuneStepIndex != tuneStepIndex)
  {
    L_tuneStepIndex = tuneStepIndex;
    SendCommand1Num(CMD_TUNEINDEX, L_tuneStepIndex);
  }

  //#define CMD_SMETER        'p' //cp
  if (L_scaledSMeter != scaledSMeter)
  {
    L_scaledSMeter = scaledSMeter;
    SendCommand1Num(CMD_SMETER, L_scaledSMeter);  
  }

  //#define CMD_SIDE_TONE     't' //vt
  if (L_sideTone != sideTone)
  {
    L_sideTone = sideTone;
    SendCommandL(CMD_SIDE_TONE, L_sideTone);
  }

  //#define CMD_KEY_TYPE      'k' //ck
  if (L_cwKeyType != cwKeyType)
  {
    L_cwKeyType = cwKeyType;
    SendCommand1Num(CMD_KEY_TYPE, L_cwKeyType);  
  }

  //#define CMD_CW_SPEED      's' //vs
  if (L_cwSpeed != cwSpeed)
  {
    L_cwSpeed = cwSpeed;
    SendCommandL(CMD_CW_SPEED, L_cwSpeed);  
  }

  //#define CMD_CW_DELAY      'y' //vy
  if (L_cwDelayTime != cwDelayTime)
  {
    L_cwDelayTime = cwDelayTime;
    SendCommandL(CMD_CW_DELAY, L_cwDelayTime);  
  }

  //#define CMD_CW_STARTDELAY 'e' //ve
  if (L_delayBeforeCWStartTime != delayBeforeCWStartTime)
  {
    L_delayBeforeCWStartTime = delayBeforeCWStartTime;
    SendCommandL(CMD_CW_STARTDELAY, L_delayBeforeCWStartTime);
  }

  //#define CMD_ATT_LEVEL     'f' //vf
  if (L_attLevel != attLevel)
  {
    L_attLevel = attLevel;
    SendCommandL(CMD_ATT_LEVEL, L_attLevel);
  }

  //#define CMD_IS_IFSHIFT    'i'
  if (L_isIFShift != isIFShift)
  {
    L_isIFShift = isIFShift;
    SendCommand1Num(CMD_IS_IFSHIFT, L_isIFShift);
  }

  //#define CMD_IFSHIFT_VALUE 'i'
  if (L_ifShiftValue != ifShiftValue)
  {
    //Serial.println("Updating Nextion with new Shiftvalue");  //mjh
    //Serial.println(ifShiftValue);    //mjh
    L_ifShiftValue = ifShiftValue;
    SendCommandL(CMD_IFSHIFT_VALUE, L_ifShiftValue);
  }

  //#define CMD_SDR_MODE      'j' //cj
  if (L_sdrModeOn != sdrModeOn)
  {
    L_sdrModeOn = sdrModeOn;
    SendCommand1Num(CMD_SDR_MODE, L_sdrModeOn);
  }
}

void updateDisplay() {
  sendUIData(0);  //UI 
}

//****************************************************************
// Spectrum for Range scan and Band Scan
//****************************************************************
#define RESPONSE_SPECTRUM     0
#define RESPONSE_EEPROM       1
#define RESPONSE_EEPROM_HEX_F 89  //C Language order
#define RESPONSE_EEPROM_HEX_R 72  //Nextion order (Reverse)
#define RESPONSE_EEPROM_STR   87  //String

const uint8_t ResponseHeader[11]={'p', 'm', '.', 's', 'h', '.', 't', 'x', 't', '=', '"'};
const char HexCodes[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', };

//void sendSpectrumData(unsigned long startFreq, unsigned long incStep, int scanCount, int delayTime, int sendCount)
//sendResponseData(RESPONSE_EEPROM, 0, eepromIndex, eepromReadLength, eepromDataType, 1);
//protocol Type : 0 - Spectrum, 1 : EEProm
//startFreq   : Spectrum - Frequency, EEProm - 0
//sendOption1 : Spectrum - 1 Step Frequency, EEProm - EEProm Start Address
//scanCount   : Spectrum - 1 Set Length, EEProm - Read Length
//sendOption2 : Spectrum - Value offset (because support various S-Meter), EEProm - EEProm Response DataType (0:HEX, 1:String)
//sendCount : Spectrum - All scan set count, EEProm - always 1
void sendResponseData(int protocolType, unsigned long startFreq, unsigned int sendOption1, int readCount, int sendOption2, int sendCount)  //Spectrum and EEProm Data
{
  unsigned long beforFreq = frequency;
  unsigned long k;
  uint8_t adcBytes[200];    //Maximum 200 Step
  
  //Voltage drop
  //scanResult[0] = analogRead(ANALOG_SMETER);
  //adcBytes[0] = analogRead(ANALOG_SMETER);
  //delay(10);
  int readedValue = 0;

  for (int si = 0; si < sendCount; si++)
  {
    for (int i = 0; i < 11; i++)
      SERIALPORT.write(ResponseHeader[i]);
      
    for (k = 0; k < readCount; k ++)
    {
      if (protocolType == RESPONSE_SPECTRUM)
      {
        //Spectrum Data
        //Sampling Range
        setFrequency(startFreq + (k * sendOption1));
        //Wait time for charging
        //delay(10);

#ifdef USE_I2CSMETER 
        readedValue = GetI2CSmeterValue(I2CMETER_UNCALCS);
#else
  
        //ADC
        readedValue = analogRead(ANALOG_SMETER);
        readedValue -= (sendOption2 * 3); //0 ~ 765
        //Down Scale
        readedValue /= 2;
        if (readedValue < 0)
        {
          readedValue = 0;
        }
        else if (readedValue>255)
        {
          readedValue=255;
        }
#endif        
      }
      else
      {
        readedValue = EEPROMTYPE.read(((sendOption2 == RESPONSE_EEPROM_HEX_R) ? (readCount - k - 1) : k) + sendOption1);
      }

      if (protocolType == RESPONSE_EEPROM && sendOption2 == RESPONSE_EEPROM_STR) //None HEX
      {
        SERIALPORT.write(readedValue);
      }
      else
      {
        SERIALPORT.write(HexCodes[readedValue >> 4]);
        SERIALPORT.write(HexCodes[readedValue & 0xf]);
      }
    }
    
    SendCommandETX(STR_ETX);
  } //end of for
}

//****************************************************************
//Receive command and processing from External device (LCD or MCU)
//****************************************************************
int spectrumSendCount = 10;   //count of full scan and Send
int spectrumOffset = 0;    //offset position
int spectrumScanCount = 100;  //Maximum 200
unsigned int spectrumIncStep = 1000;   //Increaase Step

uint8_t receivedCommandLength;
int8_t receiveIndex = 0;
int8_t ffCount = 0;

uint8_t d = 0;

//SoftwareSerial_Process
void SWS_Process(void)
 
{
   unsigned long tempFreq;            //MJH Temp variable for freq and conversions.

  if(SERIALPORT.available())
  {
        d=SERIALPORT.read();
        
        //Store Received Data
        swr_receive_buffer[receiveIndex++] = d;

        //Did we find a command?
        if (d == 0x73 && ffCount > 1 && receiveIndex > 6)
        {
          receivedCommandLength = receiveIndex;
          receiveIndex = 0;
          ffCount = 0;
        }
        else if (receiveIndex > SS_MAX_RX_BUFF)
        {
          //Buffer Overflow
          receiveIndex = 0;
          ffCount = 0;
        }
        else if (d == 0xFF)
        {
          ffCount++;
        }
        else
        {
          ffCount = 0;
        }
  
  }
  
  
  //Received Command from touch screen, now process it
  if (receivedCommandLength > 0)
  {

#ifdef COMMANDDEBUG
    for(int i=0;i<receivedCommandLength;i++)
        Serial.print(swr_receive_buffer[i],HEX);
    Serial.println("****");
#endif
    int8_t comandLength = receivedCommandLength;

#ifdef COMMANDDEBUG
    Serial.print("receivedCommandLength=");
    Serial.println(receivedCommandLength);
#endif
    
    int8_t commandStartIndex = -1;
    receivedCommandLength = 0;

    //Data Process
    //comandLength //Find start Length
    for (int i = 0; i < comandLength - 3; i++)
    {
      if (swr_receive_buffer[i] == 0x59 && swr_receive_buffer[i+ 1] == 0x58 && swr_receive_buffer[i + 2] == 0x68)
      {
        commandStartIndex = i;
        break;
      }
    } //end of for

#ifdef COMMANDDEBUG
    Serial.print(commandStartIndex);
    Serial.print(",");
    Serial.println(comandLength);
#endif    

    if (commandStartIndex != -1)
    {
      //Complete received command from touch screen
      uint8_t commandType = swr_receive_buffer[commandStartIndex + 3];

      if (commandType == TS_CMD_MODE)
      {
        byteToMode(swr_receive_buffer[commandStartIndex + 4], 1);
      }
      else if (commandType == TS_CMD_FREQ)
      {
#ifdef COMMANDDEBUG
    Serial.println("changing frequency");
#endif

        tempFreq = conv4BytesToLong(swr_receive_buffer[commandStartIndex + 4],swr_receive_buffer[commandStartIndex + 5],swr_receive_buffer[commandStartIndex + 6],swr_receive_buffer[commandStartIndex + 7]);

#ifdef COMMANDDEBUG
    for(int jj=0; jj<4; jj++)
      Serial.print(swr_receive_buffer[commandStartIndex+4+jj],HEX);
    Serial.println("**");
#endif        
        //if (tempFreq > 3000)  //for loss protcol
        //{
          frequency = tempFreq;
#ifdef COMMANDDEBUG
          Serial.print("new frequency="); Serial.print(frequency); Serial.println("***");
#endif              
        //}
      }
      else if (commandType == TS_CMD_BAND)
      {
        char currentBandIndex = -1;
        if (tuneTXType == 2 || tuneTXType == 3 || tuneTXType == 102 || tuneTXType == 103) 
        {  //only ham band move
          currentBandIndex = getIndexHambanBbyFreq(frequency);
          
          if (currentBandIndex >= 0) 
          {
            saveBandFreqByIndex(frequency, modeToByte(), currentBandIndex);
          }
        }
        setNextHamBandFreq(frequency, swr_receive_buffer[commandStartIndex + 4] == 1 ? -1 : 1);  //Prior Band      
      }
      else if (commandType == TS_CMD_VFO)
      {
        menuVfoToggle(1); //Vfo Toggle        
      }
      else if (commandType == TS_CMD_SPLIT)
      {
        menuSplitOnOff(10);
      }
      else if (commandType == TS_CMD_RIT)
      {
        menuRitToggle(1);
      }
      else if (commandType == TS_CMD_TXSTOP)
      {
        menuTxOnOff(1, 0x01);
      }
      else if (commandType == TS_CMD_SDR)
      {
        menuSDROnOff(1);
      }
      else if (commandType == TS_CMD_LOCK)
      {
        if (vfoActive == VFO_A)
          setDialLock((isDialLock & 0x01) == 0x01 ? 0 : 1, 0); //Reverse Dial lock
        else
          setDialLock((isDialLock & 0x02) == 0x02 ? 0 : 1, 0); //Reverse Dial lock
      }
      else if (commandType == TS_CMD_ATT)
      {
        attLevel = swr_receive_buffer[commandStartIndex + 4];
      }
      else if (commandType == TS_CMD_IFS)
      {
       // Serial.println("IFShift toggled");  //mjh
        isIFShift = isIFShift ? 0 : 1;  //Toggle
      }
      else if (commandType == TS_CMD_IFSVALUE)
      {
        ifShiftValue = conv2BytesToInt32(swr_receive_buffer[commandStartIndex + 4], swr_receive_buffer[commandStartIndex + 5]);
      }
      else if (commandType == TS_CMD_STARTADC)
      {
        int startIndex = swr_receive_buffer[commandStartIndex + 4];
        int endIndex = swr_receive_buffer[commandStartIndex + 5];
        int adcCheckInterval = swr_receive_buffer[commandStartIndex + 6] * 10;
        int nowCheckIndex = startIndex;
        
        while(1 == 1)      
        {
          if (receivedCommandLength > 0)
          {
            break;
          }
          
          SendCommandL('n', nowCheckIndex);    //Index Input
//
//MJH     The following was written much more elegantly originally using an array of pins and a simple loop to get the data.
//        However, because R6 and R7 are NinaPins (i.e. assigned to the Nina co-processor) on RP Connect, there was no way
//        I could figure out how to put a NinaPin in this array. Kept throwing a compile error...  Although, not elegant, an perhaps
//        a maintenance issue that someone will face in the future, this at least works on all processors.
//        Also note the pinMode(pin, INPUT_PULLUPS) that are required because some processors turn off the pullups after an analog read.
//
          switch(nowCheckIndex){    // MJH needs to be adjusted for RaspberryPI Pico
          case 0:
              #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders
                SendCommandL('x', analogRead(ENC_A));
                pinMode(ENC_A, INPUT_PULLUP);
              #endif
                break;
          case 1:
              #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders
                SendCommandL('x', analogRead(ENC_B));
                pinMode(ENC_B, INPUT_PULLUP);
              #endif
                break;
          case 2:
              #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders
                SendCommandL('x', analogRead(FBUTTON));
                pinMode(ENC_B, INPUT_PULLUP);
              #endif
                break;
          case 3:
              #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders   
                SendCommandL('x', analogRead(PTT));
                pinMode(PTT,INPUT_PULLUP);
              #endif 
                break;
          case 4:
                SendCommandL('x', analogRead(ANALOG_KEYER));
                pinMode(ANALOG_KEYER,INPUT_PULLUP);
                break;
          case 5:
                SendCommandL('x', analogRead(ANALOG_SMETER));
                break;
          }
          nowCheckIndex++;
          
          if (nowCheckIndex > endIndex)
            nowCheckIndex = startIndex;
            
          delay(adcCheckInterval);
        } //end of while 
      }  
      else if (commandType == TS_CMD_STOPADC)
      {
          //None Action
          return;
      }
      else if (commandType == TS_CMD_SPECTRUM)
      {
        //sendSpectrumData(unsigned long startFreq, unsigned int incStep, int scanCount, int delayTime, int sendCount)
        //sendSpectrumData(frequency - (1000L * 50), 1000, 100, 0, 10);
        //sendSpectrumData(*(long *)(&swr_receive_buffer[commandStartIndex + 4]), spectrumIncStep, spectrumScanCount, spectrumDelayTime, spectrumSendCount);
        unsigned long beforeFreq = frequency;
        
        tempFreq = conv4BytesToLong(swr_receive_buffer[commandStartIndex + 4],swr_receive_buffer[commandStartIndex + 5],swr_receive_buffer[commandStartIndex + 6],swr_receive_buffer[commandStartIndex + 7]);
 
        sendResponseData(RESPONSE_SPECTRUM, tempFreq, spectrumIncStep, spectrumScanCount, spectrumOffset, spectrumSendCount);
        frequency = beforeFreq;
      }
      else if (commandType == TS_CMD_SPECTRUMOPT)
      {
        //sendSpectrumData(unsigned long startFreq, unsigned int incStep, int scanCount, int delayTime, int sendCount)
        //sendSpectrumData(frequency - (1000L * 50), 1000, 100, 0, 10);
        spectrumSendCount = swr_receive_buffer[commandStartIndex + 4];          //count of full scan and Send
        spectrumOffset = swr_receive_buffer[commandStartIndex + 5];             //Scan interval time
        spectrumScanCount = swr_receive_buffer[commandStartIndex + 6];          //Maximum 120
        spectrumIncStep = swr_receive_buffer[commandStartIndex + 7] * 20;       //Increaase Step
      }
      else if (commandType == TS_CMD_TUNESTEP)      //Set Tune Step
      {
        tuneStepIndex = swr_receive_buffer[commandStartIndex + 4];    //Tune Step Index
      }
      else if (commandType == TS_CMD_WPM)      //Set WPM
      {
        cwSpeed = swr_receive_buffer[commandStartIndex + 4];    //
      }
      else if (commandType == TS_CMD_KEYTYPE)                 //Set Key Type
      {
        cwKeyType = swr_receive_buffer[commandStartIndex + 4];

        //for reduce program memory
        Iambic_Key = cwKeyType != 0;
        //if (cwKeyType == 0)
        //  Iambic_Key = false;
        //else
          //Iambic_Key = true;
          if (cwKeyType == 1)
            keyerControl &= ~IAMBICB;
          else
            keyerControl |= IAMBICB;
        //}
      }
      else if (commandType == TS_CMD_SWTRIG)
      {
        TriggerBySW = 1;    //Action Trigger by Software
      }
      else if (commandType == TS_CMD_READMEM ) //Read Mem
      //mjhfix
      {
        uint16_t eepromIndex    = conv2BytesToInt32(swr_receive_buffer[commandStartIndex + 4], swr_receive_buffer[commandStartIndex + 5]);
        byte eepromReadLength   = swr_receive_buffer[commandStartIndex + 6];
        byte eepromDataType     = swr_receive_buffer[commandStartIndex + 7];  //0 : Hex, 1 : String
        
        sendResponseData(RESPONSE_EEPROM, 0, eepromIndex, eepromReadLength, eepromDataType, 1);
      }
      else if (commandType == TS_CMD_WRITEMEM)    //Write Mem
      {
        /*
          Address : 2 byte int
          Length   : Data Length
          Checksum : (Addr0+Addr1+Len) %256
          Data      : Variable (Max 23)
         */
         //mjhfix
        uint16_t eepromIndex = conv2BytesToInt32(swr_receive_buffer[commandStartIndex + 4], swr_receive_buffer[commandStartIndex + 5]);
        byte writeLength     = swr_receive_buffer[commandStartIndex + 6];
        byte writeCheckSum   = swr_receive_buffer[commandStartIndex + 7];

        //Check Checksum
        if (writeCheckSum == (swr_receive_buffer[commandStartIndex + 4] + swr_receive_buffer[commandStartIndex + 5] + swr_receive_buffer[commandStartIndex + 6]))
        //if (writeCheckSum == (swr_receive_buffer[commandStartIndex + 4] + swr_receive_buffer[commandStartIndex + 5] + writeLength))
        {
            //if (eepromIndex > 64) //Safe #1
#ifdef UBITX_DISPLAY_NEXTION_SAFE
            //Safe #2
            if (eepromIndex < 770 || eepromIndex > 775 )
            {
              eepromIndex = -2;              
            }
            else
#else
            if (1 == 1)            
#endif
            {
              for (int i = 0; i < writeLength; i++)
                EEPROMTYPE.write(eepromIndex + i , swr_receive_buffer[commandStartIndex + 8 + i]);
            }
        }
        else
        {
          eepromIndex = -2;
        }
        SendCommandL('n', eepromIndex);             //Index Input
      }
      //else if (TS_CMD_LOOPBACK0 <= commandType && commandType <= TS_CMD_LOOPBACK5)  //Loop back Channel 0 ~ 5 Loop back Channel 1~5 : Reserve
      else if (TS_CMD_LOOPBACK0 == commandType)    //Loop back Channel 0 ~ 5
      {
        unsigned long tempCommand;
        tempCommand= conv4BytesToLong(swr_receive_buffer[commandStartIndex + 4],swr_receive_buffer[commandStartIndex + 5],
                        swr_receive_buffer[commandStartIndex + 6],swr_receive_buffer[commandStartIndex + 7]);
        SendCommandUL('v',tempCommand);     //Return data
        SendCommandUL('g', commandType);                                               //Index Input
        //return;
      }
      else if (commandType == TS_CMD_FACTORYRESET || commandType == TS_CMD_UBITX_REBOOT)
      {
        unsigned long passKey;
        passKey = conv4BytesToLong(swr_receive_buffer[commandStartIndex + 4],swr_receive_buffer[commandStartIndex + 5],swr_receive_buffer[commandStartIndex + 6],swr_receive_buffer[commandStartIndex + 7]);
        if ( passKey== 1497712748)
        {
          if (commandType == TS_CMD_UBITX_REBOOT)
          {
            FrequencyToVFO(1);  //Save current Frequency and Mode to eeprom
            #if defined(NANO33IOT)  || defined(NANOBLE) || defined(NANORP2040) || defined(RASPBERRYPIPICO)
              NVIC_SystemReset();
            #else
              #if defined(TEENSY)
                SCB_AIRCR = 0x05FA0004;
              #else
                asm volatile ("  jmp 0");
              #endif
            #endif
          }
          else
          {
            for (unsigned int i = 0; i < 32; i++) //factory setting range
              EEPROMTYPE.write(i, EEPROMTYPE.read(FACTORY_VALUES + i)); //65~96 => 0~31
          }
        }
      }

      setFrequency(frequency);
      SetCarrierFreq();
      updateDisplay(); 
    }
  }

}

char checkCount = 0;
char checkCountSMeter = 0;

//execute interval : 0.25sec
void idle_process()
{
  //S-Meter Display
  if (((displayOption1 & 0x08) == 0x08 && (sdrModeOn == 0)) && (++checkCountSMeter > SMeterLatency))
  {
#ifdef USE_I2CSMETER 
    scaledSMeter = GetI2CSmeterValue(I2CMETER_CALCS);
#else
    int newSMeter;
    
    newSMeter = analogRead(ANALOG_SMETER) / 4;
    currentSMeter = newSMeter;
  
    scaledSMeter = 0;
    for (byte s = 8; s >= 1; s--) {
      if (currentSMeter > sMeterLevels[s]) {
        scaledSMeter = s;
        break;
      }
    }

#endif  
    checkCountSMeter = 0; //Reset Latency time
  } //end of S-Meter

  sendUIData(1);
}

//When boot time, send data
void SendUbitxData(void)
{
  //Wait for ready other device (LCD, DSP and more)
  //delay(500);
  delay_background(500, 2);
  
  SendCommandL(CMD_AR_TUNE1, arTuneStep[0]);
  SendCommandL(CMD_AR_TUNE2, arTuneStep[1]);
  SendCommandL(CMD_AR_TUNE3, arTuneStep[2]);
  SendCommandL(CMD_AR_TUNE4, arTuneStep[3]);
  SendCommandL(CMD_AR_TUNE5, arTuneStep[4]);
  
  SendCommand1Num(CMD_IS_CW_SHIFT_DISPLAY, isShiftDisplayCWFreq);
  SendCommandL(CMD_CW_SHIFT_ADJUST, shiftDisplayAdjustVal);
  SendCommandL(CMD_COMM_OPTION, commonOption0);
  SendCommandL(CMD_DISP_OPTION1, displayOption1);  

  unsigned long nextionDisplayOption;
  EEPROMTYPE.get(EXTERNAL_DEVICE_OPT1, nextionDisplayOption); 
  SendCommandUL(CMD_DISP_OPTION2, nextionDisplayOption);


  SendCommandStr(CMD_VERSION, (char *)(FIRMWARE_VERSION_INFO)); //Version  mjh - set to right constant
  SendEEPromData(CMD_CALLSIGN, 0, userCallsignLength -1, 0);

  /*
  //Frequency of Bands
  for (int i = 0; i < 11; i++)
    SERIALPORT.write(SpectrumHeader[i]);

  byte *tmpByte;
  tmpByte = (byte *)hamBandRange;
  for (byte i = 0; i < (useHamBandCount -1) * 4; i++) 
  {
    SERIALPORT.write(HexCodes[*tmpByte >> 4]);
    SERIALPORT.write(HexCodes[*tmpByte & 0xf]);
    tmpByte++;
  }
      
  for (int i = 0; i < 4; i++)
    SERIALPORT.write(SpectrumFooter[i]);
  */    
    
  //Complte Send Info
  SendCommand1Num(CMD_UBITX_INFO, 1);

  //Page Init
  L_nowdisp = 0;
  SendCommand1Num(CMD_NOW_DISP, L_nowdisp);
}


//AutoKey LCD Display Routine
void Display_AutoKeyTextIndex(byte textIndex)
{
  byte diplayAutoCWLine = 0;
  
  if ((displayOption1 & 0x01) == 0x01)
    diplayAutoCWLine = 1;
  //LCD_SetCursor(0, diplayAutoCWLine);

  softBuffLines[diplayAutoCWLine][0] = byteToChar(textIndex);
  softBuffLines[diplayAutoCWLine][1] = ':';

  SendTextLineBuff(diplayAutoCWLine);
}

void LCD_CreateChar(uint8_t location, uint8_t charmap[]) 
{
}

void updateLine2Buffer(char displayType)
{
}

//not use with Nextion LCD
void DisplayCallsign(byte callSignLength)
{
}

//Not use with Nextion LCD
void DisplayVersionInfo(const __FlashStringHelper * fwVersionInfo)
{
}

#endif

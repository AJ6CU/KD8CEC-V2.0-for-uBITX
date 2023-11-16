
/*************************************************************************
  uBITX Display Routine for TFT LCD
  
  This uses the following libraries:
  TFT_eSPI : Add via IDE Library tool or from:
            https://github.com/Bodmer/TFT_eSPI/tree/master
  LVGL: Also installable via IDE Library or from:
            https://github.com/lvgl

        Actual UI structure is creased by SquareLine Studio:
            https://squareline.io/
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
// #include "ubitx_lcd.h"

//======================================================================== 
//Begin of TfT LCD Library by AJ6CU
//========================================================================
#ifdef UBITX_DISPLAY_TXT_240x320



/*Change to your screen resolution*/

// static const uint16_t screenWidth  = 480;
// static const uint16_t screenHeight = 320;
static const uint16_t screenWidth  = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[ screenWidth * screenHeight / 10 ];

TFT_eSPI tft = TFT_eSPI(screenHeight,screenWidth ); /* TFT instance */
// TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */


/* Display flushing */
void my_disp_flush( lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p )
{
    uint32_t w = ( area->x2 - area->x1 + 1 );
    uint32_t h = ( area->y2 - area->y1 + 1 );

    tft.startWrite();
    tft.setAddrWindow( area->x1, area->y1, w, h );
    tft.pushColors( ( uint16_t * )&color_p->full, w * h, true );
    tft.endWrite();

    lv_disp_flush_ready( disp );
}

/*Read the touchpad*/
void my_touchpad_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    uint16_t touchX = 0, touchY = 0;

    bool touched = tft.getTouch( &touchX, &touchY, 600 );


 
    if( !touched )
    {
        data->state = LV_INDEV_STATE_REL;
    }
    else
    {
        data->state = LV_INDEV_STATE_PR;

        /*Set the coordinates*/
        data->point.x = touchX;
        data->point.y = touchY;

        // Serial.print( "Data x " );
        // Serial.println( touchX );

        // Serial.print( "Data y " );
        // Serial.println( touchY );

    }
}


void LCDTFT240x320_Init()

{
    //Serial.begin(38400);
    // Serial.println("in LCDTFT240x320_Init");
    lv_init();

    tft.begin();          /* TFT init */
    tft.setRotation( 1 ); /* Landscape orientation, flipped */


    lv_disp_draw_buf_init( &draw_buf, buf, NULL, screenWidth * screenHeight / 10 );

    /*Initialize the display*/
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init( &disp_drv );
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register( &disp_drv );

    /*Initialize the (dummy) input device driver*/
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register( &indev_drv );

    uint16_t calData[5] = { 380, 3518, 291, 3465, 7 };
    tft.setTouch(calData);


    ui_init();

}



void LCD_Init(void)
{
  LCDTFT240x320_Init();  
}


/*************************************************************************
  MJH: Being a little tricky here.  Serial I/O to the Nextion can be one of:
  1. softserial_tiny - The original KD8CEC cut down version of SoftwareSerial
  2. SoftwareSerial - The "standard" SoftwareSerial provied with the Arduino IDE
  3. Hardware Serial

  The softserial_tiny can only be used with the Nano. Given it is smaller, a 
good choice. Also had problems with the standard software serial on Nano. Some
versions worked, others didn't...

  The standard softwareserial has to be used on the Nano and optionally can be
  used on other processors if you can connect the right pins. The softserial_tiny
  throws a lot of errors if you try to compiled with the Nano Every.

  Hardware Serial makes sense for most processors. The only reason it is not used
  on the Nano/Nano Every is that there is only one UART and it is assigned the the USB
  connections.

  With these 3 choices, the following defines just makes sure that the right set of
  routines are called when the chosen type of serial is used. 
    
**************************************************************************/
// #ifdef USE_SOFTWARESERIAL_TINY                       // Need to reference software_serial_tiny cpp functions 
// // The actual definitions are in the softserial_tiny library
//   extern void SWSerial_Begin(long speedBaud);
//   extern void SWSerial_Write(uint8_t b);
//   extern int SWSerial_Available(void);
//   extern int SWSerial_Read(void);
//   extern void SWSerial_Print(uint8_t *b);

//   #define SERIALPORTBEGIN SWSerial_Begin
//   #define SERIALPORTWRITE SWSerial_Write
//   #define SERIALPORTAVAILABLE SWSerial_Available
//   #define SERIALPORTREAD SWSerial_Read
//   #define SERIALPORTPRINT SWSerial_Print

// #elif defined(USE_SOFTWARESERIAL_STD)             //This is for the "standard" softwareserial used by the Nano
//   #include <SoftwareSerial.h>  
//   SoftwareSerial sSERIAL(SOFTWARESERIAL_RX_PIN, SOFTWARESERIAL_TX_PIN); // RX, TX
//   #define SERIALPORTBEGIN sSERIAL.begin
//   #define SERIALPORTWRITE sSERIAL.write
//   #define SERIALPORTAVAILABLE sSERIAL.available
//   #define SERIALPORTREAD sSERIAL.read
//   #define SERIALPORTPRINT sSERIAL.print 
// #else
//   //#define SERIALPORT Serial1                    //These routines are for the Hardware Serial I/O                 
//   #define SERIALPORTBEGIN Serial1.begin
//   #define SERIALPORTWRITE Serial1.write
//   #define SERIALPORTAVAILABLE Serial1.available
//   #define SERIALPORTREAD Serial1.read
//   #define SERIALPORTPRINT Serial1.print
// #endif  




// #define TEXT_LINE_LENGTH 20
// char softBuffLines[2][TEXT_LINE_LENGTH + 1];
// char softBuffSended[2][TEXT_LINE_LENGTH + 1];
// char softBuffTemp[TEXT_LINE_LENGTH + 1];    //for STR Command

char c[30], b[30];
// char softBuff[TEXT_LINE_LENGTH];        //mjh switch to using constant instead of hardwired to 20
// char softTemp[TEXT_LINE_LENGTH];        //mjh switch to using constant instead of hardwired to 20


// // ***************
// // Compatibility routines for old software serial tiny
// // ***************
// #ifndef USE_SOFTWARESERIAL_TINY            //i.e. we are using hardware serial and need to replace some functions
// #define _SS_MAX_RX_BUFF 35            //from old tiny serial to make our job easier.

// static uint8_t swr_receive_buffer[_SS_MAX_RX_BUFF];
// int8_t receiveIndex = 0;
// uint8_t receivedCommandLength = 0;
// int8_t ffCount = 0;

// void softSerail_Recv()
// {

//   uint8_t d = 0;
//   while (SERIALPORTAVAILABLE()) 
//   {
//     d=SERIALPORTREAD ();

//     if (receivedCommandLength == 0) //check Already Command
//     {
//         //Set Received Data
//         swr_receive_buffer[receiveIndex++] = d;

//         //Finded Command
//         if (d == 0x73 && ffCount > 1 && receiveIndex > 6)
//         {
//           receivedCommandLength = receiveIndex;
//           receiveIndex = 0;
//           ffCount = 0;
//         }
//         else if (receiveIndex > _SS_MAX_RX_BUFF)
//         {
//           //Buffer Overflow
//           receiveIndex = 0;
//           ffCount = 0;
//         }
//         else if (d == 0xFF)
//         {
//           ffCount++;
//         }
//         else
//         {
//           ffCount = 0;
//         }
//     }

//   }

// }

// void SWSerial_Read(uint8_t * receive_cmdBuffer)
// {
//   for (int i = 0; i < receivedCommandLength; i++)
//     receive_cmdBuffer[i] = swr_receive_buffer[i];
// }


// #endif
// // ***************
// // Endof Compatiblitiy routines for software serial tiny
// // ***************




//===================================================================
//
// Local variables to track what is on the Display right now
//
//===================================================================
// #define CMD_NOW_DISP      '0' //c0
// char L_nowdisp = -1;          //Sended nowdisp

// #define CMD_VFO_TYPE      'v' //cv
char L_vfoActive;             //vfoActive

// #define CMD_CURR_FREQ     'c' //vc
unsigned long L_vfoCurr;      //vfoA
// #define CMD_CURR_MODE     'c' //cc
// byte L_vfoCurr_mode;          //vfoA_mode

// #define CMD_VFOA_FREQ     'a' //va
unsigned long L_vfoA;         //vfoA
// #define CMD_VFOA_MODE     'a' //ca
byte L_vfoA_mode;             //vfoA_mode

// #define CMD_VFOB_FREQ     'b' //vb
unsigned long L_vfoB;         //vfoB
// #define CMD_VFOB_MODE     'b' //cb
byte L_vfoB_mode;             //vfoB_mode

// #define CMD_IS_RIT        'r' //cr
char L_ritOn;
// #define CMD_RIT_FREQ      'r' //vr
unsigned long L_ritTxFrequency; //ritTxFrequency

// #define CMD_IS_TX         't' //ct
char L_inTx;

// #define CMD_IS_DIALLOCK   'l' //cl
byte L_isDialLock;            //byte isDialLock

// #define CMD_IS_SPLIT      's' //cs
byte  L_Split;            //isTxType
// #define CMD_IS_TXSTOP     'x' //cx
byte  L_TXStop;           //isTxType  //mjh This was assumed to be zero by default??

// #define CMD_TUNEINDEX     'n' //cn
byte L_tuneStepIndex;     //byte tuneStepIndex

// #define CMD_SMETER        'p' //cs
byte L_scaledSMeter;      //scaledSMeter

// #define CMD_SIDE_TONE     't' //vt
unsigned long L_sideTone; //sideTone
// #define CMD_KEY_TYPE      'k' //ck
byte L_cwKeyType = -1;          //L_cwKeyType 0: straight, 1 : iambica, 2: iambicb

// #define CMD_CW_SPEED      's' //vs
unsigned int L_cwSpeed;   //cwSpeed

// #define CMD_CW_DELAY      'y' //vy
byte L_cwDelayTime=-1;       //cwDelayTime

// #define CMD_CW_STARTDELAY 'e' //ve
byte L_delayBeforeCWStartTime=-1;  //byte delayBeforeCWStartTime

// #define CMD_ATT_LEVEL     'f' //vf
byte L_attLevel;

// #define CMD_IS_IFSHIFT    'i' //ci
byte L_isIFShift;             //1 = ifShift, 2 extend

// #define CMD_IFSHIFT_VALUE 'i' //vi
int L_ifShiftValue;

// #define CMD_SDR_MODE      'j' //cj
byte L_sdrModeOn;

// #define CMD_UBITX_INFO     'm' //cm  Complete Send uBITX Information

// //Once Send Data, When boot
// //arTuneStep, When boot, once send
// //long arTuneStep[5];
// #define CMD_AR_TUNE1      '1' //v1
// #define CMD_AR_TUNE2      '2' //v2
// #define CMD_AR_TUNE3      '3' //v3
// #define CMD_AR_TUNE4      '4' //v4
// #define CMD_AR_TUNE5      '5' //v5


// #define CMD_IS_CW_SHIFT_DISPLAY 'h' //ch
byte L_isShiftDisplayCWFreq;  //byte isShiftDisplayCWFreq

// #define CMD_CW_SHIFT_ADJUST     'h' //vh
int L_shiftDisplayAdjustVal;        //int shiftDisplayAdjustVal

// //0:CW Display Shift Confirm, 1 : IFshift save
// #define CMD_COMM_OPTION     'o'     //vo
byte L_commonOption0;         //byte commonOption0

// //0:Line Toggle, 1 : Always display Callsign, 2 : scroll display, 3 : s.meter
// #define CMD_DISP_OPTION1    'p'   //vp
byte L_displayOption1;            //byte displayOption1
// #define CMD_DISP_OPTION2    'q'   //vq
byte L_displayOption2;            //byte displayOption2 (Reserve)

// #define CMD_TEXT_LINE0      '0'   //s0
// #define CMD_TEXT_LINE1      '1'   //s1

// #define CMD_CW_TEXT         'a'   //sa
// #define CMD_CALLSIGN        'c'   //sc
// #define CMD_VERSION         'v'   //sv

// #define TS_CMD_MODE           1
// #define TS_CMD_FREQ           2
// #define TS_CMD_BAND           3
// #define TS_CMD_VFO            4
// #define TS_CMD_SPLIT          5
// #define TS_CMD_RIT            6
// #define TS_CMD_TXSTOP         7
// #define TS_CMD_SDR            8
// #define TS_CMD_LOCK           9 //Dial Lock
// #define TS_CMD_ATT           10 //ATT
// #define TS_CMD_IFS           11 //IFS Enabled
// #define TS_CMD_IFSVALUE      12 //IFS VALUE
// #define TS_CMD_STARTADC      13
// #define TS_CMD_STOPADC       14
// #define TS_CMD_SPECTRUMOPT   15 //Option for Spectrum
// #define TS_CMD_SPECTRUM      16 //Get Spectrum Value
// #define TS_CMD_TUNESTEP      17 //Get Spectrum Value
// #define TS_CMD_WPM           18 //Set WPM
// #define TS_CMD_KEYTYPE       19 //Set KeyType

// #define TS_CMD_SWTRIG        21 //SW Action Trigger for WSPR and more
// #define TS_CMD_READMEM       31 //Read EEProm
// #define TS_CMD_WRITEMEM      32 //Write EEProm
// #define TS_CMD_LOOPBACK0     74 //Loopback1 (Response to Loopback Channgel)
// #define TS_CMD_LOOPBACK1     75 //Loopback2 (Response to Loopback Channgel)
// #define TS_CMD_LOOPBACK2     76 //Loopback3 (Response to Loopback Channgel)
// #define TS_CMD_LOOPBACK3     77 //Loopback4 (Response to Loopback Channgel)
// #define TS_CMD_LOOPBACK4     78 //Loopback5 (Response to Loopback Channgel)
// #define TS_CMD_LOOPBACK5     79 //Loopback6 (Response to Loopback Channgel)
// #define TS_CMD_FACTORYRESET  85 //Factory Reset
// #define TS_CMD_UBITX_REBOOT  95 //Reboot

// char nowdisp = 0;

// #define SWS_HEADER_CHAR_TYPE 'c'  //1Byte Protocol Prefix
// #define SWS_HEADER_INT_TYPE  'v'  //Numeric Protocol Prefex
// #define SWS_HEADER_STR_TYPE  's'  //for TEXT Line compatiable Character LCD Control


// //Control must have prefix 'v' or 's'
// char softSTRHeader[11] = {'p', 'm', '.', 's', '0', '.', 't', 'x', 't', '=', '\"'};
// char softINTHeader[10] = {'p', 'm', '.', 'v', '0', '.', 'v', 'a', 'l', '='};

// //send data for Nextion LCD
// void SendHeader(char varType, char varIndex)
// {
//   if (varType == SWS_HEADER_STR_TYPE)
//   {
//     softSTRHeader[4] = varIndex;
//     for (int i = 0; i < 11; i++)
//       SERIALPORTWRITE(softSTRHeader[i]);
//   }
//   else
//   {
//     softINTHeader[4] = varIndex;
//     for (int i = 0; i < 10; i++)
//       SERIALPORTWRITE(softINTHeader[i]);
//   }
// }

// #define INT_ETX 0
// #define STR_ETX 1
// #define TMP_ETX 2
// //Send 0xFF, 0xFF, 0xFF
// //etxType : INT_ETX = 0xFF, 0xFF, 0xFF
// //          STR_ETX = ", 0xFF, 0xFF, 0xFF
// //          TEMP_ETX = softTemp, 0xFF, 0xFF, 0xff

// void SendCommandETX(char etxType)
// {
//   if (etxType == 2)
//   {
//     SERIALPORTPRINT(softTemp);
//   }
//   else if (etxType == 1)
//   {
//     SERIALPORTPRINT("\"");
//   }
  
//   SERIALPORTWRITE(0xff);
//   SERIALPORTWRITE(0xff);
//   SERIALPORTWRITE(0xff);
// }

// void SendCommandUL(char varIndex, unsigned long sendValue)
// {
//   SendHeader(SWS_HEADER_INT_TYPE, varIndex);

//   memset(softTemp, 0, TEXT_LINE_LENGTH);        //mjh switch to using #define constant instead of hardwired

// #ifdef INTEGERS_ARE_32_BIT          //mjh  auto cast of olong to int
//   utoa(sendValue, softTemp, DEC);
// #else
//   ultoa(sendValue, softTemp, DEC);
// #endif

//   SendCommandETX(TMP_ETX);
// }

// void SendCommandL(char varIndex, long sendValue)
// {
//   SendHeader(SWS_HEADER_INT_TYPE, varIndex);

//   memset(softTemp, 0, TEXT_LINE_LENGTH);          //mjh switched to using #define constant instead of hardwired to 20

// #ifdef INTEGERS_ARE_32_BIT
// //  utoa(sendValue, softTemp, DEC);  //mjh appears to auto cast long to int
//   itoa(sendValue, softTemp, DEC);
// #else
//   ltoa(sendValue, softTemp, DEC);
// #endif
  
 
//   SendCommandETX(TMP_ETX);
// }

// void SendCommandStr(char varIndex, char* sendValue)
// {
//   SendHeader(SWS_HEADER_STR_TYPE, varIndex);
  
//   SERIALPORTPRINT(sendValue);
//   SendCommandETX(STR_ETX);
// }

// //Send String data with duplicate check
void SendTextLineBuff(uint8_t lineNumber)
{
//   //Check Duplicated data
//   if (strcmp(softBuffLines[lineNumber], softBuffSended[lineNumber]))
//   {
//     SendHeader(SWS_HEADER_STR_TYPE, lineNumber + 0x30);  //s0.txt, s1.txt
  
//     SERIALPORTPRINT(softBuffLines[lineNumber]);
//     SendCommandETX(STR_ETX);
    
//     strcpy(softBuffSended[lineNumber], softBuffLines[lineNumber]);
//   }
}

void SendTextLineStr(uint8_t lineNumber, const char* sendValue)
{
//   int i = 0;
//   for (i = 0; i < 16; i++)
//   {
//     if (sendValue[i] == 0x00)
//       break;
//     else
//       softBuffLines[lineNumber][i] = sendValue[i];
//   }
  
//   for (;i < TEXT_LINE_LENGTH; i++)
//   {
//     softBuffLines[lineNumber][i] = ' ';
}

//   // softBuffLines[lineNumber][TEXT_LINE_LENGTH + 1] = 0x00;    //mjh Original. But this means the 0 is out of bounds
//   softBuffLines[lineNumber][TEXT_LINE_LENGTH] = 0x00;
//   SendTextLineBuff(lineNumber);
// }

// Not clear if needed or useful for tft
void SendEEPromData(char varIndex, int16_t eepromStartIndex, int16_t eepromEndIndex, char offsetTtype) 
{
//   SendHeader(SWS_HEADER_STR_TYPE, varIndex);
  
//   for (int i = eepromStartIndex; i <= eepromEndIndex; i++)
//   {
//       SERIALPORTWRITE(EEPROMTYPE.read((offsetTtype == 0 ? USER_CALLSIGN_DAT : WSPR_MESSAGE1) + i));
//   }

//   SendCommandETX(STR_ETX);
}

// not clear if this is necessary in tft
// uint8_t softBuff1Num[14] = {'p', 'm', '.', 'c', '0', '.', 'v', 'a', 'l', '=', 0, 0xFF, 0xFF, 0xFF};
void SendCommand1Num(char varType, char sendValue) //0~9 : Mode, nowDisp, ActiveVFO, IsDialLock, IsTxtType, IsSplitType
{
//   softBuff1Num[4] = varType;
//   softBuff1Num[10] = sendValue + 0x30;

//   for (int i = 0; i < 14; i++)
//     SERIALPORTWRITE(softBuff1Num[i]);
}

void SetSWActivePage(char newPageIndex)   //sets active page in Nextion, equivalent for tft???
{
//     if (L_nowdisp != newPageIndex)
//     {
//       L_nowdisp = newPageIndex;
//       SendCommand1Num(CMD_NOW_DISP, L_nowdisp);
//     }
}
// //===================================================================
// //End of Nextion LCD Protocol
// //===================================================================

// // The generic routine to display one line on the LCD 
void printLine(unsigned char linenmbr, const char *c) {
  
  // SendTextLineStr(linenmbr, c);
}

void printLineF(uint8_t linenmbr, const __FlashStringHelper *c)
{
  // int i;
  // char tmpBuff[21];
  // PGM_P p = reinterpret_cast<PGM_P>(c);  

  // for (i = 0; i < 21; i++){
  //   unsigned char fChar = pgm_read_byte(p++);
  //   tmpBuff[i] = fChar;
  //   if (fChar == 0)
  //     break;
  // }

  // printLine(linenmbr, tmpBuff);
}

// #define LCD_MAX_COLUMN 20 
void printLineFromEEPRom(uint8_t linenmbr, uint8_t lcdColumn, byte eepromStartIndex, byte eepromEndIndex, uint8_t offsetTtype) 
{
//   int colIndex = lcdColumn;
//   for (byte i = eepromStartIndex; i <= eepromEndIndex; i++)
//   {
//     if (++lcdColumn <= LCD_MAX_COLUMN)
//       softBuffLines[linenmbr][colIndex++] = EEPROMTYPE.read((offsetTtype == 0 ? USER_CALLSIGN_DAT : WSPR_MESSAGE1) + i);
//     else
//       break;
//   }

  // SendTextLineBuff(linenmbr);
}

// //  short cut to print to the first line
void printLine1(const char *c)
{
  // printLine(1,c);
}
// //  short cut to print to the first line
void printLine2(const char *c)
{
  // printLine(0,c);
}

void clearLine2()
{
  // printLine2("");
  // line2DisplayStatus = 0;
}

//  short cut to print to the first line
void printLine1Clear(){
  // printLine(1,"");
}
//  short cut to print to the first line
void printLine2Clear(){
  // printLine(0, "");
}

void printLine2ClearAndUpdate(){
  // printLine(0, "");
  // line2DisplayStatus = 0;  
  // updateDisplay();
}

// //==================================================================================
// //End of Display Base Routines
// //==================================================================================

//==================================================================================
//Begin of User Interface Callbacks
//==================================================================================
//originally designed for Nextion, adopted for TFT



void toggleStopButton(lv_event_t * e)
{

  menuTxOnOff(1, 0x01);     // Toggle stop state

  if (isTxType & 0x01)    // a 1 in indicates TX STOP in place
    lv_obj_add_state( ui_txStopButton, LV_STATE_CHECKED);  // This puts button in pressed state

  else
    lv_obj_clear_state( ui_txStopButton, LV_STATE_CHECKED);
}



void toggleVFOClicked(lv_event_t * e)
/*
* Called by the UX when the VFO toggle button is clicked
*/
{
  char tmpBuffer[15];       //used for converting the frequency to xx.xxx.xxx from a number
	//
	// First update the UX labels. 
  // If we were in VFO A, the new label on the button will be VFO B
  // If we were in VFO B, the new label on the button will be VFO A
  //
  if (vfoActive == VFO_A )
		lv_label_set_text(ui_currentVFO, "VFO B");
	else
		lv_label_set_text(ui_currentVFO, "VFO A");

  //
  // The current frequency and mode will become the inactive one.
  // Update the inactive labels with the current values
  //


  formatNumber(frequency, tmpBuffer);
  lv_label_set_text(ui_inactiveFreq,tmpBuffer);   
  lv_label_set_text(ui_inactiveMode,modeToString()); 

  menuVfoToggle(1); //the is the original routine that does the toggling

  //
  // Now update the displayed frequency and mode in the UX after the swap
  //

  formatNumber(frequency, tmpBuffer);
  lv_label_set_text(ui_activeFreq,tmpBuffer); 

  lv_label_set_text(ui_modeSelectLabel,modeToString());   // update displayed mode  

}



void bandDownClicked(lv_event_t * e)
{
	uint8_t currentBandIndex;
  char tmpBuffer[15];
  if (tuneTXType == 2 || tuneTXType == 3 || tuneTXType == 102 || tuneTXType == 103) 
  {  //only ham band move save frequency
    currentBandIndex = getIndexHambanBbyFreq(frequency);
    
    if (currentBandIndex >= 0) 
    {
      saveBandFreqByIndex(frequency, modeToByte(), currentBandIndex);
    }
  }
  
  setNextHamBandFreq(frequency, -1);  // go down one band

  formatNumber(frequency, tmpBuffer);
  lv_label_set_text(ui_activeFreq,tmpBuffer); 
  lv_label_set_text(ui_modeSelectLabel,modeToString());
}


void bandUpClicked(lv_event_t * e)
{
  uint8_t currentBandIndex;
  char tmpBuffer[15];
  if (tuneTXType == 2 || tuneTXType == 3 || tuneTXType == 102 || tuneTXType == 103) 
  {  //only ham band move save frequency
    currentBandIndex = getIndexHambanBbyFreq(frequency);
    
    if (currentBandIndex >= 0) 
    {
      saveBandFreqByIndex(frequency, modeToByte(), currentBandIndex);
    }
  }
  setNextHamBandFreq(frequency, +1);
  
  formatNumber(frequency, tmpBuffer);
  lv_label_set_text(ui_activeFreq,tmpBuffer); 

  lv_label_set_text(ui_modeSelectLabel,modeToString());
  
}

//
// Convenience routine to set new mode on radio and update the label
//
void updateMode(uint8_t newmode){
  byteToMode(newmode,0);
  lv_label_set_text(ui_modeSelectLabel,modeToString());
}

void setModeLSB(lv_event_t * e)
{              
  updateMode(2);                  // 2 is mode for LSB, 0 says dont set by frequency
}

void setModeUSB(lv_event_t * e)
{
  updateMode(3);                  // 3 is mode for USB, 0 says dont set by frequency
}

void setModeCWL(lv_event_t * e)
{
  updateMode(4);                  // 4 is mode for CWL, 0 says dont set by frequency
}

void setModeCWU(lv_event_t * e)
{
  updateMode(5);                  // 5 is mode for CWU, 0 says dont set by frequency
}


void toggleLockButton(lv_event_t * e)
{
  if (vfoActive == VFO_A) {
    setDialLock((isDialLock & 0x01) == 0x01 ? 0 : 1, 0); //Bit 0==1 indicates VFO_A is locked
    //
    // If now locked, change the state to show its locked
    //
    if(isDialLock & 0x01)
      lv_obj_add_state( ui_lockDisplayButton, LV_STATE_CHECKED);  // This puts button in pressed state (locked)
    else
      lv_obj_clear_state( ui_lockDisplayButton, LV_STATE_CHECKED);

  }
  else {          //In VFO B
    setDialLock((isDialLock & 0x02) == 0x02 ? 0 : 1, 0); //Bit 2 ==1 indicates VFO_B is locked
    //
    // If now locked, change the state to show its locked
    //
    if(isDialLock & 0x02)
      lv_obj_add_state( ui_lockDisplayButton, LV_STATE_CHECKED);  // This puts button in pressed state
    else
      lv_obj_clear_state( ui_lockDisplayButton, LV_STATE_CHECKED);
  }

}

void toggleSDRButton(lv_event_t * e)
{
	menuSDROnOff(1);
  // Now update the UX. Highight the button if SDR is on, also make sure Label is correct
  if (sdrModeOn) {   // a 1 in indicates we just went into SDR mode
    lv_obj_add_state( ui_spkToggleButton, LV_STATE_CHECKED);  // This puts button in pressed state
    lv_label_set_text(ui_spkToggleLabel,"SDR");
  }
  else {
    lv_obj_clear_state( ui_spkToggleButton, LV_STATE_CHECKED);
    lv_label_set_text(ui_spkToggleLabel,"SPK");
  }
}


void formatNumber(long f, char *buf) {
// add period separators. Used generally for displaying frequencies

  utoa(f, buf,  DEC);

  int f_Length = strlen(buf);
  if (f < 1000)    //no separators required, just return the buffer
    return;
  else {
    for ( int i = f_Length+1; i > f_Length-4; i--)
      buf[i+1] = buf[i];
    buf[f_Length-3] = '.';

    // Now check on whether there is another separator to insert
    if (f > 999999)     { // two need to be inserted
      f_Length = strlen(buf);    // need to reset because we already inserted one separator
      for ( int i = f_Length+1; i > f_Length-8; i--)
        buf[i+1] = buf[i];
      buf[f_Length-7] = '.';
    }
  }    
}

//unsigned long 
byte nowPageIndex = 0;

// sendType == 1 not check different 
void sendUIData(int sendType)            // Probably should be merged into updatedisplay
{
  //Serial.begin(38400);
  // char nowActiveVFO = vfoActive == VFO_A ? 0 : 1;
  char tmpBuffer[15]; 

//   //#define CMD_VFO_TYPE      'v' //cv
  // if (L_vfoActive != nowActiveVFO)
  // {
    
    // L_vfoActive = nowActiveVFO;

    // formatNumber(L_vfoCurr, tmpBuffer);

    // if(L_vfoActive == 0 ) {            // a zero means VFO A is active
    //   Serial.println("switching active vfo to A");
    //   Serial.print("current frequency="); Serial.println(frequency);
    //   Serial.print("vfoA="); Serial.println(vfoA);
    //   Serial.print("vfoB="); Serial.println(vfoB);

    //   vfoB = frequency;
    //   frequency = vfoA;
    //   L_vfoCurr = frequency;
    //   Serial.print("new frequency="); Serial.println(frequency);

    // } else {         //Switch to VFO B
    //   Serial.println("switching active vfo to B");
    //   Serial.print("vfoB="); Serial.println(vfoB);
    //   Serial.print("frequency="); Serial.println(frequency);
    //   vfoA = frequency;
    //   frequency = vfoB;
    //   L_vfoCurr = frequency;

    // }

    // formatNumber(frequency, tmpBuffer);
    // lv_label_set_text(ui_activeFreq,tmpBuffer);
    


    // SendCommand1Num(CMD_VFO_TYPE, L_vfoActive);
  // }

//   //#define CMD_CURR_FREQ     'c' //vc

  if (L_vfoCurr != frequency) 
  {
    // Serial.println("local and freq are not equal");
    // Serial.print("local frq="); Serial.println(L_vfoCurr);
    // Serial.print("frequency="); Serial.println(frequency);
    L_vfoCurr = frequency;

    formatNumber(frequency, tmpBuffer);

    lv_label_set_text(ui_activeFreq,tmpBuffer);
  } // else
        // {Serial.println("local and freq are equal");
        // Serial.print("local frq="); Serial.println(L_vfoCurr);
        // Serial.print("frequency="); Serial.println(frequency);}

//   //#define CMD_CURR_MODE     'c' //cc
//   byte vfoCurr_mode = modeToByte();
//   if (L_vfoCurr_mode != vfoCurr_mode)
//   {
//     L_vfoCurr_mode = vfoCurr_mode;
//     SendCommand1Num(CMD_CURR_MODE, L_vfoCurr_mode);
//   }

//   //if auto cw key mode, exit
//   //if (isCWAutoMode != 0 || menuOn != 0)
//   if (isCWAutoMode != 0)
//     return;

//   //nowPageIndex = 0;
//   if (menuOn==0)
//   {
//     if (sendType == 0)
//     {
//       SetSWActivePage(0);
//     }
//     else
//     {
//       SetSWActivePage(0);
//     }
//   }
//   else
//   {
//     //Text Line Mode
//       SetSWActivePage(1);
//   }

//   //#define CMD_VFOA_FREQ     'a' //va
//   //VFOA
  // if (L_vfoA != vfoA)
  // {
  //   L_vfoA = vfoA;
  //   // SendCommandUL(CMD_VFOA_FREQ, L_vfoA);
  // }

//   //#define CMD_VFOA_MODE     'a' //ca
//   if (L_vfoA_mode != vfoA_mode)
//   {
//     L_vfoA_mode = vfoA_mode;
//     SendCommand1Num(CMD_VFOA_MODE, L_vfoA_mode);
//   }

//   //#define CMD_VFOB_FREQ     'b' //vb
//   //VFOB
  // if (L_vfoB != vfoB)
  // {
  //   L_vfoB = vfoB;
  //   // SendCommandUL(CMD_VFOB_FREQ, L_vfoB);
  // }

//   //#define CMD_VFOB_MODE     'b' //cb
//   if (L_vfoB_mode != vfoB_mode)
//   {
//     L_vfoB_mode = vfoB_mode;
//     SendCommand1Num(CMD_VFOB_MODE, L_vfoB_mode);  
//   }

//   //byte isDialLock = ((isTxType & 0x01) == 0x01) ? 1 : 0;
//   if (L_isDialLock != isDialLock)
//   {
//     L_isDialLock = isDialLock;
//     SendCommand1Num(CMD_IS_DIALLOCK, L_isDialLock);  
//   }

//   //#define CMD_IS_RIT        'r' //cr
//   if (L_ritOn != ritOn)
//   {
//     L_ritOn = ritOn;
//     SendCommand1Num(CMD_IS_RIT, L_ritOn);  
//   }
  
//   //#define CMD_RIT_FREQ      'r' //vr
//   //unsigned long L_ritTxFrequency; //ritTxFrequency
//   if (L_ritTxFrequency != ritTxFrequency)
//   {
//     L_ritTxFrequency = ritTxFrequency;
//     SendCommandUL(CMD_RIT_FREQ, L_ritTxFrequency);  
//   }

//   //#define CMD_IS_TX         't' //ct
//   //char L_inTx;
//   if (L_inTx != inTx)
//   {
//     L_inTx = inTx;
//     SendCommand1Num(CMD_IS_TX, L_inTx);  
//   }

//   //#define CMD_IS_DIALLOCK   'l' //cl
//   //byte L_isDialLock;            //byte isDialLock
//   if (L_isDialLock != isDialLock)
//   {
//     L_isDialLock = isDialLock;
//     SendCommand1Num(CMD_IS_DIALLOCK, L_isDialLock);  
//   }

//   //#define CMD_IS_SPLIT      's' //cs
//   //byte  L_Split;            //isTxType
//   if (L_Split != splitOn)
//   {
//     L_Split = splitOn;
//     SendCommand1Num(CMD_IS_SPLIT, L_Split);  
//   }
  

//   //#define CMD_IS_TXSTOP     'x' //cx
//   byte isTXStop = ((isTxType & 0x01) == 0x01);
//   if (L_TXStop != isTXStop)
//   {
//     L_TXStop = isTXStop;
//     SendCommand1Num(CMD_IS_TXSTOP, L_TXStop);
//   }

//   //#define CMD_TUNEINDEX     'n' //cn
//   if (L_tuneStepIndex != tuneStepIndex)
//   {
//     L_tuneStepIndex = tuneStepIndex;
//     SendCommand1Num(CMD_TUNEINDEX, L_tuneStepIndex);
//   }

//   //#define CMD_SMETER        'p' //cp
//   if (L_scaledSMeter != scaledSMeter)
//   {
//     L_scaledSMeter = scaledSMeter;
//     SendCommand1Num(CMD_SMETER, L_scaledSMeter);  
//   }

//   //#define CMD_SIDE_TONE     't' //vt
//   if (L_sideTone != sideTone)
//   {
//     L_sideTone = sideTone;
//     SendCommandL(CMD_SIDE_TONE, L_sideTone);
//   }

//   //#define CMD_KEY_TYPE      'k' //ck
//   if (L_cwKeyType != cwKeyType)
//   {
//     L_cwKeyType = cwKeyType;
//     SendCommand1Num(CMD_KEY_TYPE, L_cwKeyType);  
//   }

//   //#define CMD_CW_SPEED      's' //vs
//   if (L_cwSpeed != cwSpeed)
//   {
//     L_cwSpeed = cwSpeed;
//     SendCommandL(CMD_CW_SPEED, L_cwSpeed);  
//   }

//   //#define CMD_CW_DELAY      'y' //vy
//   if (L_cwDelayTime != cwDelayTime)
//   {
//     L_cwDelayTime = cwDelayTime;
//     SendCommandL(CMD_CW_DELAY, L_cwDelayTime);  
//   }

//   //#define CMD_CW_STARTDELAY 'e' //ve
//   if (L_delayBeforeCWStartTime != delayBeforeCWStartTime)
//   {
//     L_delayBeforeCWStartTime = delayBeforeCWStartTime;
//     SendCommandL(CMD_CW_STARTDELAY, L_delayBeforeCWStartTime);
//   }

//   //#define CMD_ATT_LEVEL     'f' //vf
//   if (L_attLevel != attLevel)
//   {
//     L_attLevel = attLevel;
//     SendCommandL(CMD_ATT_LEVEL, L_attLevel);
//   }

//   //#define CMD_IS_IFSHIFT    'i'
//   if (L_isIFShift != isIFShift)
//   {
//     L_isIFShift = isIFShift;
//     SendCommand1Num(CMD_IS_IFSHIFT, L_isIFShift);
//   }

//   //#define CMD_IFSHIFT_VALUE 'i'
//   if (L_ifShiftValue != ifShiftValue)
//   {
//     L_ifShiftValue = ifShiftValue;
//     SendCommandL(CMD_IFSHIFT_VALUE, L_ifShiftValue);
//   }

//   //#define CMD_SDR_MODE      'j' //cj
//   if (L_sdrModeOn != sdrModeOn)
//   {
//     L_sdrModeOn = sdrModeOn;
//     SendCommand1Num(CMD_SDR_MODE, L_sdrModeOn);
//   }
}

void updateDisplay() {
  sendUIData(0);  //UI 
}

// //****************************************************************
// // Spectrum for Range scan and Band Scan
// //****************************************************************
// #define RESPONSE_SPECTRUM     0
// #define RESPONSE_EEPROM       1
// #define RESPONSE_EEPROM_HEX_F 89  //C Language order
// #define RESPONSE_EEPROM_HEX_R 72  //Nextion order (Reverse)
// #define RESPONSE_EEPROM_STR   87  //String

// const uint8_t ResponseHeader[11]={'p', 'm', '.', 's', 'h', '.', 't', 'x', 't', '=', '"'};
// const char HexCodes[16] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f', };

// //void sendSpectrumData(unsigned long startFreq, unsigned long incStep, int scanCount, int delayTime, int sendCount)
// //sendResponseData(RESPONSE_EEPROM, 0, eepromIndex, eepromReadLength, eepromDataType, 1);
// //protocol Type : 0 - Spectrum, 1 : EEProm
// //startFreq   : Spectrum - Frequency, EEProm - 0
// //sendOption1 : Spectrum - 1 Step Frequency, EEProm - EEProm Start Address
// //scanCount   : Spectrum - 1 Set Length, EEProm - Read Length
// //sendOption2 : Spectrum - Value offset (because support various S-Meter), EEProm - EEProm Response DataType (0:HEX, 1:String)
// //sendCount : Spectrum - All scan set count, EEProm - always 1
// void sendResponseData(int protocolType, unsigned long startFreq, unsigned int sendOption1, int readCount, int sendOption2, int sendCount)  //Spectrum and EEProm Data
// {
//   // unsigned long k;   // mjh originally this was a long to handle the full frequency. but 32 bit int is fine
//   int32_t k;
  
//   int readedValue = 0;

//   for (int si = 0; si < sendCount; si++)
//   {
//     for (int i = 0; i < 11; i++)
//       SERIALPORTWRITE(ResponseHeader[i]);
      
//     for (k = 0; k < readCount; k ++)
//     {
//       if (protocolType == RESPONSE_SPECTRUM)
//       {
//         //Spectrum Data
//         //Sampling Range
//         setFrequency(startFreq + (k * sendOption1));
//         //Wait time for charging
//         //delay(10);

// #ifdef USE_I2CSMETER 
//         readedValue = GetI2CSmeterValue(I2CMETER_UNCALCS);
// #else
  
//         //ADC
//         readedValue = analogRead(ANALOG_SMETER);
//         readedValue -= (sendOption2 * 3); //0 ~ 765
//         //Down Scale
//         readedValue /= 2;
//         if (readedValue < 0)
//         {
//           readedValue = 0;
//         }
//         else if (readedValue>255)
//         {
//           readedValue=255;
//         }
// #endif        
//       }
//       else
//       {
//         readedValue = EEPROMTYPE.read(((sendOption2 == RESPONSE_EEPROM_HEX_R) ? (readCount - k - 1) : k) + sendOption1);
//       }

//       if (protocolType == RESPONSE_EEPROM && sendOption2 == RESPONSE_EEPROM_STR) //None HEX
//       {
//         SERIALPORTWRITE(readedValue);
//       }
//       else
//       {
//         SERIALPORTWRITE(HexCodes[readedValue >> 4]);
//         SERIALPORTWRITE(HexCodes[readedValue & 0xf]);
//       }
//     }
    
//     SendCommandETX(STR_ETX);
//   } //end of for
// }

// //****************************************************************
// //Receive command and processing from External device (LCD or MCU)
// //****************************************************************
// int spectrumSendCount = 10;   //count of full scan and Send
// int spectrumOffset = 0;    //offset position
// int spectrumScanCount = 100;  //Maximum 200
// unsigned int spectrumIncStep = 1000;   //Increaase Step

// #ifdef USE_SOFTWARESERIAL_TINY           // these are in softwareserial_tiny, but defined locally for HW Serial
// extern uint8_t receivedCommandLength;
// extern void SWSerial_Read(uint8_t * receive_cmdBuffer);
// #endif

// uint8_t swr_buffer[20];

// //SoftwareSerial_Process
void SWS_Process(void)
 
{
    lv_timer_handler(); /* let the GUI do its work */
    // setFrequency(frequency);
    // SetCarrierFreq();
    // updateDisplay(); 

}

//  old stuff that was done
//   unsigned long tempFreq;            //MJH Temp variable for freq and conversions
  
//   #ifndef USE_SOFTWARESERIAL_TINY       // Need to try prefetching command to initialize variables if HW Serial
//   softSerail_Recv();               //try to fetch a command
//   #endif

//   if (receivedCommandLength > 0)  //If this global is set, that means a command has been found
//   {
      
//     SWSerial_Read(swr_buffer);    //Bring a copy of he command in, reset input buffer

//     int8_t comandLength = receivedCommandLength;
//     int8_t commandStartIndex = -1;
//     receivedCommandLength = 0;

//     //Data Process
//     //comandLength //Find start Length
//     for (int i = 0; i < comandLength - 3; i++)
//     {
//       if (swr_buffer[i] == 0x59 && swr_buffer[i+ 1] == 0x58 && swr_buffer[i + 2] == 0x68)
//       {
//         commandStartIndex = i;
//         break;
//       }
//     } //end of for

// #ifdef COMMANDDEBUG
//     Serial.print(commandStartIndex);
//     Serial.print(",");
//     Serial.print(comandLength);
//     Serial.print(":");
//     for(int jj=0; jj<comandLength; jj++) {
//       Serial.print(jj);
//       Serial.print("-");
//       Serial.print(swr_buffer[jj],HEX);
//       Serial.print(":");
//     }
//     Serial.println("**");
// #endif    

//     if (commandStartIndex != -1)
//     {
//       //Complete received command from touch screen
//       uint8_t commandType = swr_buffer[commandStartIndex + 3];

//       if (commandType == TS_CMD_MODE)
//       {
//         byteToMode(swr_buffer[commandStartIndex + 4], 1);
//       }
//       else if (commandType == TS_CMD_FREQ)
//       {
// #ifdef COMMANDDEBUG
//     Serial.println("changing frequency");
// #endif

//         tempFreq = conv4BytesToLong(swr_buffer[commandStartIndex + 4],swr_buffer[commandStartIndex + 5],swr_buffer[commandStartIndex + 6],swr_buffer[commandStartIndex + 7]);

// #ifdef COMMANDDEBUG
//     for(int jj=0; jj<4; jj++)
//       Serial.print(swr_buffer[commandStartIndex+4+jj],HEX);
//     Serial.println("**");
// #endif        
//         //if (tempFreq > 3000)  //for loss protcol
//         //{
//           frequency = tempFreq;
// #ifdef COMMANDDEBUG
//           Serial.print("new frequency="); Serial.print(frequency); Serial.println("***");
// #endif              
//         //}
//       }
//       else if (commandType == TS_CMD_BAND)
//       {
//         char currentBandIndex = -1;
//         if (tuneTXType == 2 || tuneTXType == 3 || tuneTXType == 102 || tuneTXType == 103) 
//         {  //only ham band move
//           currentBandIndex = getIndexHambanBbyFreq(frequency);
          
//           if (currentBandIndex >= 0) 
//           {
//             saveBandFreqByIndex(frequency, modeToByte(), currentBandIndex);
//           }
//         }
//         setNextHamBandFreq(frequency, swr_buffer[commandStartIndex + 4] == 1 ? -1 : 1);  //Prior Band      
//       }
//       else if (commandType == TS_CMD_VFO)
//       {
//         menuVfoToggle(1); //Vfo Toggle        
//       }
//       else if (commandType == TS_CMD_SPLIT)
//       {
//         menuSplitOnOff(10);
//       }
//       else if (commandType == TS_CMD_RIT)
//       {
//         menuRitToggle(1);
//       }
//       else if (commandType == TS_CMD_TXSTOP)
//       {
//         menuTxOnOff(1, 0x01);
//       }
//       else if (commandType == TS_CMD_SDR)
//       {
//         menuSDROnOff(1);
//       }
//       else if (commandType == TS_CMD_LOCK)
//       {
//         if (vfoActive == VFO_A)
//           setDialLock((isDialLock & 0x01) == 0x01 ? 0 : 1, 0); //Reverse Dial lock
//         else
//           setDialLock((isDialLock & 0x02) == 0x02 ? 0 : 1, 0); //Reverse Dial lock
//       }
//       else if (commandType == TS_CMD_ATT)
//       {
//         attLevel = swr_buffer[commandStartIndex + 4];
//       }
//       else if (commandType == TS_CMD_IFS)
//       {
//         isIFShift = isIFShift ? 0 : 1;  //Toggle
//       }
//       else if (commandType == TS_CMD_IFSVALUE)
//       {
//         ifShiftValue = conv2BytesToInt32(swr_buffer[commandStartIndex + 4], swr_buffer[commandStartIndex + 5]);
//       }
//       else if (commandType == TS_CMD_STARTADC)
//       {
//         int startIndex = swr_buffer[commandStartIndex + 4];
//         int endIndex = swr_buffer[commandStartIndex + 5];
//         int adcCheckInterval = swr_buffer[commandStartIndex + 6] * 10;
//         int nowCheckIndex = startIndex;
        
//         while(1 == 1)      
//         {
//           if (receivedCommandLength > 0)
//           {
//             break;
//           }
          
//           SendCommandL('n', nowCheckIndex);    //Index Input
// //
// //MJH     The following was written much more elegantly originally using an array of pins and a simple loop to get the data.
// //        However, because R6 and R7 are NinaPins (i.e. assigned to the Nina co-processor) on RP Connect, there was no way
// //        I could figure out how to put a NinaPin in this array. Kept throwing a compile error...  Although, not elegant, an perhaps
// //        a maintenance issue that someone will face in the future, this at least works on all processors.
// //        Also note the pinMode(pin, INPUT_PULLUPS) that are required because some processors turn off the pullups after an analog read.
// //
//           switch(nowCheckIndex){    // MJH needs to be adjusted for RaspberryPI Pico
//           case 0:
//               #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders
//                 SendCommandL('x', analogRead(ENC_A));b,
//                 pinMode(ENC_A, INPUT_PULLUP);
//               #endif
//                 break;
//           case 1:
//               #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders
//                 SendCommandL('x', analogRead(ENC_B));
//                 pinMode(ENC_B, INPUT_PULLUP);
//               #endif
//                 break;
//           case 2:
//               #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders
//                 SendCommandL('x', analogRead(FBUTTON));
//                 pinMode(ENC_B, INPUT_PULLUP);
//               #endif
//                 break;
//           case 3:
//               #ifndef USE_DIGITAL_ENCODER                   //MJH Analog pin not used with digital encoders   
//                 SendCommandL('x', analogRead(PTT));
//                 pinMode(PTT,INPUT_PULLUP);
//               #endif 
//                 break;
//           case 4:
//                 SendCommandL('x', analogRead(ANALOG_KEYER));
//                 pinMode(ANALOG_KEYER,INPUT_PULLUP);
//                 break;
//           case 5:
//                 SendCommandL('x', analogRead(ANALOG_SMETER));
//                 pinMode(ANALOG_SMETER,INPUT_PULLUP);        //mjh
//                 break;
//           }
//           nowCheckIndex++;
          
//           if (nowCheckIndex > endIndex)
//             nowCheckIndex = startIndex;
            
//           delay(adcCheckInterval);
//         } //end of while 
//       }  
//       else if (commandType == TS_CMD_STOPADC)
//       {
//           //None Action
//           return;
//       }
//       else if (commandType == TS_CMD_SPECTRUM)
//       {
//         //sendSpectrumData(unsigned long startFreq, unsigned int incStep, int scanCount, int delayTime, int sendCount)
//         //sendSpectrumData(frequency - (1000L * 50), 1000, 100, 0, 10);
//         //sendSpectrumData(*(long *)(&swr_buffer[commandStartIndex + 4]), spectrumIncStep, spectrumScanCount, spectrumDelayTime, spectrumSendCount);
//         unsigned long beforeFreq = frequency;
        
//         tempFreq = conv4BytesToLong(swr_buffer[commandStartIndex + 4],swr_buffer[commandStartIndex + 5],swr_buffer[commandStartIndex + 6],swr_buffer[commandStartIndex + 7]);
 
//         sendResponseData(RESPONSE_SPECTRUM, tempFreq, spectrumIncStep, spectrumScanCount, spectrumOffset, spectrumSendCount);
//         frequency = beforeFreq;
//       }
//       else if (commandType == TS_CMD_SPECTRUMOPT)
//       {
//         //sendSpectrumData(unsigned long startFreq, unsigned int incStep, int scanCount, int delayTime, int sendCount)
//         //sendSpectrumData(frequency - (1000L * 50), 1000, 100, 0, 10);
//         spectrumSendCount = swr_buffer[commandStartIndex + 4];          //count of full scan and Send
//         spectrumOffset = swr_buffer[commandStartIndex + 5];             //Scan interval time
//         spectrumScanCount = swr_buffer[commandStartIndex + 6];          //Maximum 120
//         spectrumIncStep = swr_buffer[commandStartIndex + 7] * 20;       //Increaase Step
//       }
//       else if (commandType == TS_CMD_TUNESTEP)      //Set Tune Step
//       {
//         tuneStepIndex = swr_buffer[commandStartIndex + 4];    //Tune Step Index
//       }
//       else if (commandType == TS_CMD_WPM)      //Set WPM
//       {
//         cwSpeed = swr_buffer[commandStartIndex + 4];    //
//       }
//       else if (commandType == TS_CMD_KEYTYPE)                 //Set Key Type
//       {
//         cwKeyType = swr_buffer[commandStartIndex + 4];

//         //for reduce program memory
//         Iambic_Key = cwKeyType != 0;
//         //if (cwKeyType == 0)
//         //  Iambic_Key = false;
//         //else
//           //Iambic_Key = true;
//           if (cwKeyType == 1)
//             keyerControl &= ~IAMBICB;
//           else
//             keyerControl |= IAMBICB;
//         //}
//       }
//       else if (commandType == TS_CMD_SWTRIG)
//       {
//         TriggerBySW = 1;    //Action Trigger by Software
//       }
//       else if (commandType == TS_CMD_READMEM ) //Read Mem
    
//       {
//         uint16_t eepromIndex    = conv2BytesToInt32(swr_buffer[commandStartIndex + 4], swr_buffer[commandStartIndex + 5]);
//         byte eepromReadLength   = swr_buffer[commandStartIndex + 6];
//         byte eepromDataType     = swr_buffer[commandStartIndex + 7];  //0 : Hex, 1 : String
        
//         sendResponseData(RESPONSE_EEPROM, 0, eepromIndex, eepromReadLength, eepromDataType, 1);
//       }
//       else if (commandType == TS_CMD_WRITEMEM)    //Write Mem
//       {
//         /*
//           Address : 2 byte int
//           Length   : Data Length
//           Checksum : (Addr0+Addr1+Len) %256
//           Data      : Variable (Max 23)
//          */

//         uint16_t eepromIndex = conv2BytesToInt32(swr_buffer[commandStartIndex + 4], swr_buffer[commandStartIndex + 5]);
//         byte writeLength     = swr_buffer[commandStartIndex + 6];
//         byte writeCheckSum   = swr_buffer[commandStartIndex + 7];

//         //Check Checksum
//         if (writeCheckSum == (swr_buffer[commandStartIndex + 4] + swr_buffer[commandStartIndex + 5] + swr_buffer[commandStartIndex + 6]))
//         //if (writeCheckSum == (swr_buffer[commandStartIndex + 4] + swr_buffer[commandStartIndex + 5] + writeLength))
//         {
//             //if (eepromIndex > 64) //Safe #1
// #ifdef UBITX_DISPLAY_NEXTION_SAFE
//             //Safe #2
//             if (eepromIndex < 770 || eepromIndex > 775 )
//             {
//               eepromIndex = -2;              
//             }
//             else
// #else
//             if (1 == 1)            
// #endif
//             {
//               for (int i = 0; i < writeLength; i++)
//                 EEPROMTYPE.write(eepromIndex + i , swr_buffer[commandStartIndex + 8 + i]);
//             }
//         }
//         else
//         {
//           eepromIndex = -2;
//         }
//         SendCommandL('n', eepromIndex);             //Index Input
//       }
//       //else if (TS_CMD_LOOPBACK0 <= commandType && commandType <= TS_CMD_LOOPBACK5)  //Loop back Channel 0 ~ 5 Loop back Channel 1~5 : Reserve
//       else if (TS_CMD_LOOPBACK0 == commandType)    //Loop back Channel 0 ~ 5
//       {
//         unsigned long tempCommand;
//         tempCommand= conv4BytesToLong(swr_buffer[commandStartIndex + 4],swr_buffer[commandStartIndex + 5],
//                         swr_buffer[commandStartIndex + 6],swr_buffer[commandStartIndex + 7]);
//         SendCommandUL('v',tempCommand);     //Return data
//         SendCommandUL('g', commandType);                                               //Index Input
//         //return;
//       }
//       else if (commandType == TS_CMD_FACTORYRESET || commandType == TS_CMD_UBITX_REBOOT)
//       {
//         unsigned long passKey;
//         passKey = conv4BytesToLong(swr_buffer[commandStartIndex + 4],swr_buffer[commandStartIndex + 5],swr_buffer[commandStartIndex + 6],swr_buffer[commandStartIndex + 7]);
//         if ( passKey== 1497712748)
//         {
//           if (commandType == TS_CMD_UBITX_REBOOT)
//           {
//             FrequencyToVFO(1);  //Save current Frequency and Mode to eeprom
//             #if defined(NANO33IOT)  || defined(NANOBLE) || defined(NANORP2040) || defined(RASPBERRYPIPICO)
//               NVIC_SystemReset();
//             #else
//               #if defined(TEENSY)
//                 SCB_AIRCR = 0x05FA0004;
//               #else
//                 asm volatile ("  jmp 0");
//               #endif
//             #endif
//           }
//           else
//           {
//             for (unsigned int i = 0; i < 32; i++) //factory setting range
//               EEPROMTYPE.write(i, EEPROMTYPE.read(FACTORY_VALUES + i)); //65~96 => 0~31
//           }
//         }
//       }
//       setFrequency(frequency);
//       SetCarrierFreq();
//       updateDisplay(); 
// //     }
//   }

// }

char checkCount = 0;
char checkCountSMeter = 0;

//execute interval : 0.25sec
void idle_process()
{
//   //S-Meter Display
//   if (((displayOption1 & 0x08) == 0x08 && (sdrModeOn == 0)) && (++checkCountSMeter > SMeterLatency))
//   {
// #ifdef USE_I2CSMETER 
//     scaledSMeter = GetI2CSmeterValue(I2CMETER_CALCS);
// #else
//     int newSMeter;
    
//     newSMeter = analogRead(ANALOG_SMETER) / 4;
//     currentSMeter = newSMeter;
  
//     scaledSMeter = 0;
//     for (byte s = 8; s >= 1; s--) {
//       if (currentSMeter > sMeterLevels[s]) {
//         scaledSMeter = s;
//         break;
//       }
//     }

// #endif  
//     checkCountSMeter = 0; //Reset Latency time
//   } //end of S-Meter

  sendUIData(1);
}

//When boot time, send data
void SendUbitxData(void)
{
  // //Wait for ready other device (LCD, DSP and more)
  // //delay(500);
  // delay_background(500, 2);
  
  // SendCommandL(CMD_AR_TUNE1, arTuneStep[0]);
  // SendCommandL(CMD_AR_TUNE2, arTuneStep[1]);
  // SendCommandL(CMD_AR_TUNE3, arTuneStep[2]);
  // SendCommandL(CMD_AR_TUNE4, arTuneStep[3]);
  // SendCommandL(CMD_AR_TUNE5, arTuneStep[4]);
  
  // SendCommand1Num(CMD_IS_CW_SHIFT_DISPLAY, isShiftDisplayCWFreq);
  // SendCommandL(CMD_CW_SHIFT_ADJUST, shiftDisplayAdjustVal);
  // SendCommandL(CMD_COMM_OPTION, commonOption0);
  // SendCommandL(CMD_DISP_OPTION1, displayOption1);  

  // unsigned long nextionDisplayOption;
  // EEPROMTYPE.get(EXTERNAL_DEVICE_OPT1, nextionDisplayOption); 
  // SendCommandUL(CMD_DISP_OPTION2, nextionDisplayOption);


  // SendCommandStr(CMD_VERSION, (char *)(FIRMWARE_VERSION_INFO)); //Version  mjh - set to right constant
  // SendEEPromData(CMD_CALLSIGN, 0, userCallsignLength -1, 0);

  // /*
  // //Frequency of Bands
  // for (int i = 0; i < 11; i++)
  //   SERIALPORT.write(SpectrumHeader[i]);

  // byte *tmpByte;
  // tmpByte = (byte *)hamBandRange;
  // for (byte i = 0; i < (useHamBandCount -1) * 4; i++) 
  // {
  //   SERIALPORT.write(HexCodes[*tmpByte >> 4]);
  //   SERIALPORT.write(HexCodes[*tmpByte & 0xf]);
  //   tmpByte++;
  // }
      
  // for (int i = 0; i < 4; i++)
  //   SERIALPORT.write(SpectrumFooter[i]);
  // */    
    
  // //Complte Send Info
  // SendCommand1Num(CMD_UBITX_INFO, 1);

  // //Page Init
  // L_nowdisp = 0;
  // SendCommand1Num(CMD_NOW_DISP, L_nowdisp);
}


//AutoKey LCD Display Routine
void Display_AutoKeyTextIndex(byte textIndex)
{
  // byte diplayAutoCWLine = 0;
  
  // if ((displayOption1 & 0x01) == 0x01)
  //   diplayAutoCWLine = 1;
  // //LCD_SetCursor(0, diplayAutoCWLine);

  // softBuffLines[diplayAutoCWLine][0] = byteToChar(textIndex);
  // softBuffLines[diplayAutoCWLine][1] = ':';

  // SendTextLineBuff(diplayAutoCWLine);
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

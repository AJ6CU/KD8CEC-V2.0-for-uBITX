/*************************************************************************
  KD8CEC's Memory Keyer for HAM
  
  This source code is written for All amateur radio operator, 
  I have not had amateur radio communication for a long time. CW has been 
  around for a long time, and I do not know what kind of keyer and keying 
  software is fashionable. So I implemented the functions I need mainly.

  To minimize the use of memory space, we used bitwise operations.
  For the alphabet, I put Morsecode in 1 byte. The front 4Bit is the length 
  and the 4Bit is the Morse code. Because the number is fixed in length, 
  there is no separate length information. The 5Bit on the right side is 
  the Morse code.

  I wrote this code myself, so there is no license restriction. 
  So this code allows anyone to write with confidence.
  But keep it as long as the original author of the code.
  DE Ian KD8CEC
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
/*
-----------------------------------------------------------------------------
    Instructions on use of Memory Keyer

These are the instructions for use that KD8CEC apparently did not get around to
providing.

1. Enter the Memory Keyer through the LCD menus (yes even on Nextion) - short press
of encoder.  
2. Turn to Memory Keyer memory item and press the encoder again to enter the memory keyer menu
3. At this point DO NOT PRESS the Encoder Again unless you want to exit the memory keyer
4. You should be presented with a screen that shows the available messages to send with "PTT to Send"
PTT will not really send. A quick PTT will switch between message to send and set frequency for TX
5. Select message to send by turning the encoder. When you have the message you wnat, quick press on ptt
6. NOw you can reset the Freq using the encoder knob. when you are happy you can click ptt again
7. Note that now you back at the select text menu item. Click again and you are back to the Tuen freq.
8. To Actually send a message, long press of ptt
9. To pause a message mid sending, push and hold the ptt
10. to repeat a message, quick ptt during the message will resend
11. to terminate sending mid stream, click the encoder quickly.
**************************************************************************/    





#include <avr/pgmspace.h>

//27 + 10 + 18 + 1(SPACE) = //56 
const PROGMEM uint8_t cwAZTable[27] = {0b00100100 , 0b01001000 , 0b01001010 , 0b00111000 , 0b00010000, 0b01000010, 0b00111100, 0b01000000 , //A ~ H
0b00100000, 0b01000111 ,0b00111010, 0b01000100, 0b00101100, 0b00101000 , 0b00111110, 0b01000110, 0b01001101, 0b00110100, //I ~ R
0b00110000, 0b00011000, 0b00110010, 0b01000001, 0b00110110, 0b01001001, 0b01001011, 0b01001100};  //S ~ Z
PGM_P pCwAZTable = reinterpret_cast<PGM_P>(cwAZTable);

const PROGMEM uint8_t cw09Table[27] = {0b00011111, 0b00001111, 0b00000111, 0b00000011, 0b00000001, 0b00000000, 0b00010000, 0b00011000, 0b00011100, 0b00011110};
PGM_P pcw09Table = reinterpret_cast<PGM_P>(cw09Table);

//# : AR, ~:BT, [:AS, ]:SK, ^:KN
const PROGMEM uint8_t cwSymbolIndex[] =  {'.',         ',',        '?',         '"',       '!',         '/',      '(',       ')',        '&',        ':',        ';',         '=',        '+',        '-',        '_',        '\'',       '@',          '#',         '~',        '[',        ']',        '^' };
PGM_P pCwSymbolIndex = reinterpret_cast<PGM_P>(cwSymbolIndex);

const PROGMEM uint8_t cwSymbolTable[]  = {0b11010101, 0b11110011, 0b11001100, 0b11011110, 0b11101011, 0b10100100, 0b10101100, 0b11101101, 0b10010000, 0b11111000, 0b11101010, 0b10100010, 0b10010100, 0b11100001, 0b11001101, 0b11010010,  0b11011010,  0b10010100, 0b10100010, 0b10010000, 0b11000101, 0b10101100};
PGM_P pCwSymbolTable = reinterpret_cast<PGM_P>(cwSymbolTable);
////const PROGMEM uint8_t cwSymbolLength[] = {6,          6,          6,         6,           6,          5,          5,          6,          5,          6,          6,          5,          5,          6,          6,          6,         6,         5,          5,          5,           6,          5};

// ":(Start"),   ':(End "), >: My callsign, <:QSO Callsign (Second Callsign), #:AR, ~:BT, [:AS, ]:SK

byte knobPosition = 0;
//byte cwTextData[30];                        //Maximum 30  Remarked by KD8CE -> Direct Read EEPROM
byte autoCWSendCharEndIndex = 0;
byte autoCWSendCharIndex = 0;
unsigned long autoCWbeforeTime = 0;         //for interval time between chars
byte pttBeforeStatus = 1;                   //PTT : default high
byte isKeyStatusAfterCWStart = 0;           //0 : Init, 1 : Keyup after auto CW Start, 2 : Keydown after
byte selectedCWTextIndex = 0;               //Tracks which message number is currently being sent.
static unsigned long autoCWKeydownCheckTime = 0;   //for interval time between chars
byte changeReserveStatus = 0;
byte isAutoCWHold = 0;                      //auto CW Pause => Manual Keying => auto
//
// *********************State Variables***************
// isCWAutoMode is one of 3 states: 
//      0: Not in any autokeyer mode
//      1: In the autokeyer menu (LCD overlay for Nextion), but not sending
//      2: In sending mode
//
// isKeyStatusAfterCWStart one of 4 states:
//      0 : Init, 
//      1 : PTT released after auto CW Start 
//      2 : PTT pushed after auto CW Start
//      3 : PTT is a long push and we are in hold state waiting for a release (PTT=HIGH)
// ********************End State Variables************

void autoSendPTTCheck()
{
    if (isCWAutoMode == 2) {                           //Sending Mode
        //check PTT Button
        //short Press => reservation or cancel
        //long Press => Hold


        if (digitalRead(PTT) == LOW)                    // PTT is pressed (pin grounded)
        {

          //if (isKeyStatusAfterCWStart == 0)          //Yet Press PTT from start TX
          //{
          //}
          //
          // MJH Clarification
          // isKeyStatusAfterCWStart tracks the state of the last time thru of the PTT
          // key *after we started sending*.  If isKeyStatusAfterStart == 1 (as in the following if statement
          // We had detected the release of the PTT Key after start previously.
          // But now, the PTT button has been pushed again.If a long push, that means a hold. If a short
          // push that means a repeat of the currently sending text.
          //
          
          if (isKeyStatusAfterCWStart == 1)            //while auto cw send, ptt up and ptt down again
                                                        // So this means we hav a new push and we need to determine whether
                                                        // it is a long push or a short push. That is the purpose of tracking
                                                        // the time in autoCWKeydownCheckTime
                                                        //
          {
            //Start Time
            autoCWKeydownCheckTime = millis() + 200;   //Long push time
            isKeyStatusAfterCWStart = 2;               //Change status => ptt down agian
          }
          else if ((isKeyStatusAfterCWStart == 2) && (autoCWKeydownCheckTime < millis()))
                                                      // So this means that we detected the push on last time through
                                                      // and current millis is more than the check time. This means a 
                                                      // Long PTT push
          {
            //Hold Mode
            isAutoCWHold = 1;                         // This measns "hold off sending any more CW"
            isKeyStatusAfterCWStart = 3;              // Setting isKeyStatusAfterCEStart to 3 means that we are in
                                                      // waiting for the PTT to be released. So keep checking for a release
                                                      // every 200 ms.
          }
          else if (isKeyStatusAfterCWStart == 3)
          {
            autoCWKeydownCheckTime = millis() + 200;  // check again in 200ms
          }
          
        }
        else                                          // This else means that the PPT key has been detected as being released (high)
        { 
          //PTT UP
          if (isKeyStatusAfterCWStart == 2)            //0 (down before cw start) -> 1 (up while cw sending) -> 2 (down while cw sending)
                                                        // A state of 2 here, means that the PTT was previously detected as down, and now
                                                        // It is being detected as up. So the question is whether it was a short or long 
                                                        // push
          {
            if (autoCWKeydownCheckTime > millis())     //Short : Reservation or cancel Next Text
                                                        // So a short push was detected.This means that the currently sending CW message
                                                        // should be repeated
            {
              if (autoCWSendReservCount == 0 ||         // is a global byte variable that is declared in ubitx_20.ino
                  (autoCWSendReservCount < AUTO_CW_RESERVE_MAX &&
                autoCWSendReserv[autoCWSendReservCount - 1] != selectedCWTextIndex))
              {
                //Reserve
                autoCWSendReserv[autoCWSendReservCount++] = selectedCWTextIndex;
                changeReserveStatus = 1;
              }
              else if (autoCWSendReservCount > 0 && autoCWSendReserv[autoCWSendReservCount - 1] == selectedCWTextIndex)
              {
                autoCWSendReservCount--;
                changeReserveStatus = 1;
              }
            } // end of Short Key up
          }
          else if (isKeyStatusAfterCWStart == 3)    //play from Hold (pause Auto CW Send)
          {
            isAutoCWHold = 0;
          }
          isKeyStatusAfterCWStart = 1;                        //Change status => ptt up (while cw send mode)
        }     //end of PTT UP  
    }
}

//Send 1 char
void sendCWChar(char cwKeyChar)
{
  byte sendBuff[7];
  byte i, j, charLength;
  byte tmpChar;

  //For Macrofunction
  //replace > and  < to My callsign, qso callsign, use recursive function call
  if (cwKeyChar == '>' || cwKeyChar == '<')
  {
    uint16_t callsignStartIndex = 0;
    uint16_t callsignEndIndex = 0;
    
    if (cwKeyChar == '>') //replace my callsign
    {
      if (userCallsignLength > 0)
      {
        callsignStartIndex = 0;
        callsignEndIndex = userCallsignLength;
      }
    }
    else if (cwKeyChar == '<')  //replace qso callsign
    {
      //ReadLength
      callsignEndIndex = EEPROMTYPE.read(CW_STATION_LEN);
      if (callsignEndIndex > 0)
      {
        callsignStartIndex = CW_STATION_LEN - callsignEndIndex - USER_CALLSIGN_DAT;
        callsignEndIndex = callsignStartIndex + callsignEndIndex;
      }
    }

    if (callsignStartIndex == 0 && callsignEndIndex == 0)
      return;

    for (uint16_t i = callsignStartIndex; i <= callsignEndIndex; i++)
    {
      sendCWChar(EEPROMTYPE.read(USER_CALLSIGN_DAT + i));
      autoSendPTTCheck(); //for reserve and cancel next CW Text
      if (changeReserveStatus == 1)
      {
        changeReserveStatus = 0;
        updateDisplay();
      }
      
      if (i < callsignEndIndex) delay_background(cwSpeed * 3, 4); //
    }
    
    return;
  }
  else if (cwKeyChar >= 'A' && cwKeyChar <= 'Z')  //Encode Char by KD8CEC
  {
    tmpChar = pgm_read_byte(pCwAZTable + (cwKeyChar - 'A'));
    charLength = (tmpChar >> 4) & 0x0F;
    for (i = 0; i < charLength; i++)
      sendBuff[i] = (tmpChar << i) & 0x08;
  }
  else if (cwKeyChar >= '0' && cwKeyChar <= '9')
  {
    charLength = 5;
    for (i = 0; i < charLength; i++)
      sendBuff[i] = (pgm_read_byte(pcw09Table + (cwKeyChar - '0')) << i) & 0x10;
  }
  else if (cwKeyChar == ' ')
  {
    charLength = 0;
    delay_background(cwSpeed * 4, 4); //7 -> basic interval is 3
  }
  else if (cwKeyChar == '$')  //7 digit
  {
    charLength = 7;
    for (i = 0; i < 7; i++)
      sendBuff[i] = (0b00010010 << i) & 0x80; //...1..1
  }
  else
  {
    //symbol
    for (i = 0; i < 22; i++)
    {
      if (pgm_read_byte(pCwSymbolIndex + i) == cwKeyChar)
      {
        tmpChar = pgm_read_byte(pCwSymbolTable + i);
        charLength = ((tmpChar >> 6) & 0x03) + 3;
        
        for (j = 0; j < charLength; j++)
          sendBuff[j] = (tmpChar << (j + 2)) & 0x80;

        break;
      }
      else
      {
        charLength = 0;
      }
    }
  }

  for (i = 0; i < charLength; i++)
  {

    cwKeydown();

    if (sendBuff[i] == 0) {
      delay_background(cwSpeed, 4);
    }
    else {
      delay_background(cwSpeed * 3, 4);
    }

    cwKeyUp();
    if (i != charLength -1)
      delay_background(cwSpeed, 4);
  }
}

byte isNeedScroll = 0;
unsigned long scrollDispayTime = 0;
#define scrollSpeed 500
byte displayScrolStep = 0;

void controlAutoCW(){
    int knob = 0;
    byte i;

    static byte cwStartIndex, cwEndIndex;       //mjh no static declaration originally. Assumed values were preserved between calls
    
    if (cwAutoDialType == 0)
      knob = enc_read();
  
    if (knob != 0 || beforeCWTextIndex == 255 || isNeedScroll == 1){ //start display
      if (knobPosition > 0 && knob < 0)
        knobPosition--;
      if (knobPosition < cwAutoTextCount * 10 -1 && knob > 0)
        knobPosition++;

      selectedCWTextIndex = knobPosition / 10;

      if ((beforeCWTextIndex != selectedCWTextIndex) || 
        ((isNeedScroll == 1) && (beforeCWTextIndex == selectedCWTextIndex) && (scrollDispayTime < millis()))) {
          //Read CW Text Data Position From EEProm
          EEPROMTYPE.get(CW_AUTO_DATA + (selectedCWTextIndex * 2), cwStartIndex);
          EEPROMTYPE.get(CW_AUTO_DATA + (selectedCWTextIndex * 2 + 1), cwEndIndex);


          if (beforeCWTextIndex == selectedCWTextIndex)
          {
            if (++displayScrolStep > cwEndIndex - cwStartIndex)
              displayScrolStep = 0;
          }
          else
          {
              displayScrolStep = 0;
          }

#ifdef UBITX_DISPLAY_NEXTION
          //Not need Scroll
          //Display_AutoKeyTextIndex(selectedCWTextIndex);
          SendCommand1Num('w', selectedCWTextIndex);                                              //Index
          SendEEPromData('a', cwStartIndex + CW_DATA_OFSTADJ, cwEndIndex + CW_DATA_OFSTADJ, 0) ;  //Data
          SendCommand1Num('y', 1);                                            //Send YN
          isNeedScroll = 0;
#else          
          printLineFromEEPRom(0, 2, cwStartIndex + displayScrolStep + CW_DATA_OFSTADJ, cwEndIndex + CW_DATA_OFSTADJ, 0); 
          isNeedScroll = (cwEndIndex - cwStartIndex) > 14 ? 1 : 0;
          Display_AutoKeyTextIndex(selectedCWTextIndex);
#endif
          scrollDispayTime = millis() + scrollSpeed;
          beforeCWTextIndex = selectedCWTextIndex;
      }
    } //end of check knob

    if (isCWAutoMode == 1) {                                    //ready status
      if (digitalRead(PTT) == LOW)                              //PTT Down : Start Auto CW or DialMode Change
      {
        if (pttBeforeStatus == 1)                               //High to Low Change
        {
          autoCWbeforeTime = millis() + 500;                    //Long push time
          pttBeforeStatus = 0;
        }
        else if (autoCWbeforeTime < millis())                   //while press PTT, OK Long push then Send Auto CW Text
        {
          sendingCWTextIndex = selectedCWTextIndex;
          
          //Information about Auto Send CW Text
          autoCWSendCharEndIndex = cwEndIndex;                  //length of CW Text     //ianlee
          autoCWSendCharIndex = cwStartIndex;                   //position of Sending Char  //ianlee

          isCWAutoMode = 2;                                     //auto sending start
          autoCWbeforeTime = 0;                                 //interval between chars, 0 = always send
          isKeyStatusAfterCWStart = 0;                          //Init PTT Key status
          autoCWSendReservCount = 0;                            //Init Reserve Count
          isAutoCWHold = 0;
          if (!inTx){                                           //if not TX Status, change RX -> TX
            keyDown = 0;
            startTx(TX_CW, 0);  //disable updateDisplay Command for reduce latency time
            updateDisplay();
        
            delay_background(delayBeforeCWStartTime * 2, 2);    //for External AMP or personal situation
          }
        }
      }
      else if (pttBeforeStatus == 0 && autoCWbeforeTime > 0)    //while reade status LOW -> HIGH (before Auto send Before)
      {
          pttBeforeStatus = 1;  //HIGH
          if (autoCWbeforeTime > millis())                      //short Press -> ? DialModeChange
          {
            cwAutoDialType = (cwAutoDialType == 1 ? 0 : 1);     //Invert DialMode between select CW Text and Frequency Tune
            if (cwAutoDialType == 0)
              printLineF1(F("Dial:Select Text"));
            else
              printLineF1(F("Dial:Freq Tune"));

            delay_background(1000, 0);
            updateDisplay();
          }
      }
    } //end of isCWAutoMode == 1 condition
    
    if (isCWAutoMode == 2) {                                    //Sending Mode
        autoSendPTTCheck();
        //check interval time, if you want adjust interval between chars, modify below


        if (isAutoCWHold == 0 && (millis() - autoCWbeforeTime > cwSpeed * 3))
        {
          if (!inTx){                                           //if not TX Status, change RX -> TX
            keyDown = 0;
            startTx(TX_CW, 0);  //disable updateDisplay Command for reduce latency time
          }
          // Serial.print(EEPROMTYPE.read(CW_AUTO_DATA + autoCWSendCharIndex));
          // Serial.print(":");
          sendCWChar(EEPROMTYPE.read(CW_AUTO_DATA + autoCWSendCharIndex++));

          if (autoCWSendCharIndex > autoCWSendCharEndIndex) {          //finish auto cw send
            //check reserve status
            if (autoCWSendReservCount > 0)
            {
              //prepare
              sendingCWTextIndex = autoCWSendReserv[0];

              for (i = 0; i < AUTO_CW_RESERVE_MAX -1; i++)
                autoCWSendReserv[i] = autoCWSendReserv[i + 1];

              EEPROMTYPE.get(CW_AUTO_DATA + (sendingCWTextIndex * 2), cwStartIndex);
              EEPROMTYPE.get(CW_AUTO_DATA + (sendingCWTextIndex * 2 + 1), cwEndIndex);

              //Information about Auto Send CW Text
              autoCWSendCharEndIndex = cwEndIndex;                  //length of CW Text     //ianlee
              autoCWSendCharIndex = cwStartIndex;                   //position of Sending Char  //ianlee
              autoCWSendReservCount--;                              //Decrease

              sendCWChar(' ');    //APPLY SPACE between CW Texts
              changeReserveStatus = 1;
            }
            else
            {
              isCWAutoMode = 1; //ready status
              delay_background(cwDelayTime * 10, 2);
              stopTx();
            }
          }

          autoCWbeforeTime = millis();
          
          if (changeReserveStatus == 1)
          {
            changeReserveStatus = 0;
            updateDisplay();
          }
        }
    }

    //abort if this button is down
    if (btnDown())
    {
      isCWAutoMode = 0; //dsiable Auto CW Mode
      printLine2ClearAndUpdate();
      delay_background(1000, 0);
    }
}

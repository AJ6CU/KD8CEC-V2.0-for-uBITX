/***
    The purpose of this sketch is to copy the existing contents of the internal EEPROM of the nano to the external eeprom. 

    You need to do *this* using your existing Nano and the external EEPROM installed *BEFORE* you install your new Nano 33 IOT or BLE.
    This will write every byte read from your original Nano to the Serial console, then write that byte to the external EEPROM.

    

    This Sketch is based on a "get_example" originally written by Christopher Andrews in 2015. All licensing is inhereted from his 
    original MIT license.
    
    _get example.

    This shows how to use the EEPROM.get() method.

    To pre-set the EEPROM data, run the example sketch eeprom_put.
    This sketch will run without it, however, the values shown
    will be shown from what ever is already on the EEPROM.

    This may cause the serial object to print out a large string
    of garbage if there is no null character inside one of the strings
    loaded.

    Written by Christopher Andrews 2015
    Released under MIT licence.
***/

#include <EEPROM.h>
#include <Wire.h>

// Note I installed the SparkFun library into the system area. If you keep it local to this project, you need to replace
// the <> with ""

#include <SparkFun_External_EEPROM.h> // Click here to get the library: http://librarymanager/All#SparkFun_External_EEPROM
ExternalEEPROM myMem;

void setup() {

  byte aByte;

  Serial.begin(115200);
  Wire.begin();
  delay(1000);

 if (myMem.begin() == false)
  {
    Serial.println("No memory detected. Freezing.");
    while (1)
      ;
  }
  
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Staring transfer ");

for (int i=0; i<1024; i++){

  //Get the float data from the EEPROM at position 'eeAddress'
  aByte=EEPROM.read(i);
  Serial.print(" At memory location: ");
  Serial.print(i);
  Serial.print(" read byte*");
  Serial.print(aByte);
  Serial.println("*");

  myMem.write(i, aByte);
}
  

}


void loop() {
  /* Empty loop */
}

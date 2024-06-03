/***
  eeprom_counter example.
  Written by by Maxint R&D for CH32.

	The purpose of this example is to demonstrate non-volatile storage of a simple counter that counts
  the times that the setup() function was called. It also shows the entire contents of the EEPROM.
***/
#include <EEPROM.h>

void setup()
{
  Serial.begin(115200);
  delay(2000);   // allow some time for Arduino IDE v2 serial console to show after recompiling
  Serial.println("\n--- EEPROM COUNTER EXAMPLE ---");
  Serial.printf("Option bytes: 0x%08x\r\n", EEPROM.ReadOptionBytes()); //  usually FF FF to start with
  EEPROM.begin();

  // Read the counter at EEPROM address 0, increment and write it back.
  uint8_t bootcnt=EEPROM.read(0);
	bootcnt++;
  EEPROM.write(0, bootcnt);
	Serial.printf("Boot count is %d [0x%02X]\n",bootcnt, bootcnt);

  // Use EEPROM.commit() to write data to permanent storage
  EEPROM.commit();

  // Show the contents of the entire EEPROM memory
  Serial.print("EEPROM contents:");
  for(int n=0; n<EEPROM.length(); n++)
  {
    if(n%8==0) Serial.println("");
	  Serial.printf("%02X ", EEPROM[n]);
  }
  Serial.println(".");
}

void loop()
{   // nothing to do

}
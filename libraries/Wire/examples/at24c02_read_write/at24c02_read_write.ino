/*
  EEPROM AT24C02 read and write
 
  This example read/write an EEPROM AT24C02 by I2C port.

  AT24c02 Circuit:

  A0/A1/A2/GND ----- GND (So device address is 0xA0 (0x50 << 1) )
  VCC -------------- VCC
  WP  -------------- GND
  SCL -------------- SCL(MCU)
  SDA ---------------SDA(MCU)

  Created 30 June 2023 by TempersLee
*/

#include <Wire.h>

#define AT24C02_ADDR  0x50   //will left shift 1 bits in library
void setup()
{
  uint16_t i=0;
  Serial.begin(115200);
  Serial.print("Hello CH32duino!  ");
  Serial.print("Chip ID: ");
  Serial.println(DBGMCU_GetDEVID(), HEX);
  
  Wire.begin();  // join i2c bus (address optional for master)

  Serial.println("Write AT24C02...");
  for(i=0;i<256;i++)
  {
      deviceWriteOneByte(i,i);
      delay(5);
  }
  Serial.println("Write Finish!");

  Serial.println("Read AT24C02:");
  for(i=0;i<256;i++)
  {
    if(i%16 == 0 && i!=0)Serial.println("");
    Serial.print(deviceReadOneByte(i), HEX);
    Serial.print(" ");
  }

  Serial.println("\r\nEND!");
}

void loop()
{
  
}

void deviceWriteOneByte(uint8_t addr, uint8_t data)
{
  Wire.beginTransmission(AT24C02_ADDR);  //transmit to device AT24C02
  Wire.write(addr);
  Wire.write(data);
  Wire.endTransmission();
}

uint8_t deviceReadOneByte(uint8_t addr)
{
  Wire.requestFrom(AT24C02_ADDR,1,addr,1,1);
  // requestFrom() has already read the byte; 0xFF if the EEPROM didn't answer
  if (Wire.available())
  {
    return Wire.read();
  }
  return 0; // Should not get here
}

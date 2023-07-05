/*
  EEPROM AT24C02 read and write
 
  This example read/write an EEPROM AT24C02 by I2C port.

  AT24c02 Circuit:

  A0/A1/A2/GND ----- GND (So device address is 0xA0 (0x50 << 1) )
  VCC -------------- VCC
  WP  -------------- GND
  SCL -------------- SCL(MCU)
  SDA ---------------SDA(MCU)

  created 30 Jurn 2023 
  by TempersLee

 */

#include <Wire.h>


#define AT24C02_ADDR  0x50   //will left shift 1 bits in library
void setup()
{
  uint16_t i=0;
  Serial.begin(115200);
  Serial.printf("%s  Chip ID: 0x%08x\r\n", "Hello CH32duino!", DBGMCU_GetDEVID());
  Wire.begin();  // join i2c bus (address optional for master)

  Serial.printf("Write AT24C02:\r\n");
  for(i=0;i<256;i++)
  {
      deviceWriteOneByte(i,i);
      delay(5);  
  }
  Serial.printf("Write Finish!\r\n");

  Serial.printf("Read AT24C02:\r\n");
  for(i=0;i<256;i++)
  {
    if(i%16 == 0 && i!=0)Serial.printf("\n");
    Serial.printf("%02x ",deviceReadOneByte(i));     
  }

  Serial.printf("\r\nEND!\r\n");
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
  while (Wire.available())
  {
    return Wire.read();
  }
}

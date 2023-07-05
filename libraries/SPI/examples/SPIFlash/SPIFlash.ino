/*
  Winbond SPIFlash read and write
 
  This example read/write a W25QXX Flash by SPI port.
  Pins:
  CS:   PA2 (003 is PC4)
  MOSI: PA7 (003 is PC6) 
  MISO: PA6 (003 is PC7)
  SCK:  PA5 (003 is PC5)
  created 30 Jurn 2023
  by TempersLee
 */

#include <SPI.h>

// Winbond SPIFLASH ID
#define W25Q80 	              0xEF13
#define W25Q16 	              0xEF14
#define W25Q32 	              0xEF15
#define W25Q64 	              0xEF16
#define W25Q128	              0xEF17

/* Winbond SPIFalsh Instruction List */
#define W25X_WriteEnable		  0x06
#define W25X_WriteDisable		  0x04
#define W25X_ReadStatusReg		0x05
#define W25X_WriteStatusReg		0x01
#define W25X_ReadData			    0x03
#define W25X_FastReadData		  0x0B
#define W25X_FastReadDual		  0x3B
#define W25X_PageProgram		  0x02
#define W25X_BlockErase			  0xD8
#define W25X_SectorErase		  0x20
#define W25X_ChipErase			  0xC7
#define W25X_PowerDown			  0xB9
#define W25X_ReleasePowerDown	0xAB
#define W25X_DeviceID			    0xAB
#define W25X_ManufactDeviceID	0x90
#define W25X_JedecDeviceID		0x9F


//define CS pin
#define chipSelectPin A2


uint16_t flashType;
uint8_t readBuffer[256];
uint8_t writeBuffer[256];

void setup() {
  uint16_t i=0;
  Serial.begin(115200);
  // start the SPI library:
  SPI.begin();
  Serial.printf("%s  Chip ID: 0x%08x\r\n", "Hello CH32duino!", DBGMCU_GetDEVID());

  // initialize the chip select pins:
  pinMode(chipSelectPin, OUTPUT);
  digitalWrite(chipSelectPin, HIGH);
  delay(100);

  flashType = readFlashID();
  switch (flashType)
  {
    case W25Q16:
      Serial.printf("W25Q16 is OK!\r\n");  
      break;
    case W25Q32:
      Serial.printf("W25Q32 is OK!\r\n");
      break;
    case W25Q64:
      Serial.printf("W25Q64 is OK!\r\n");
      break;
    case W25Q80:
      Serial.printf("W25Q80 is OK!\r\n");
      break;
    case W25Q128:
      Serial.printf("W25Q128 is OK!\r\n");
      break; 
  default:
     Serial.printf("Flash is not valid!\r\n");
    break;
  }

  //initial the writeBuffer
  for(i=0; i<256; i++)
  {
    writeBuffer[i] = i;
  }

  //erase sector 0
  Serial.printf("Erase Flash:\r\n");
  eraseSector(0);
  readFlashToBuff(readBuffer, 0, 256);
  for(i=0; i<256; i++)
  {
    if(i%16==0 && i!=0)  Serial.printf("\r\n");
    Serial.printf("%02x ",readBuffer[i]);
  }
  Serial.printf("\r\n");

  Serial.printf("Write Flash:\r\n");
  writeFlashPage(writeBuffer, 0, 256);
  readFlashToBuff(readBuffer, 0, 256);
  for(i=0; i<256; i++)
  {
    if(i%16==0 && i!=0)  Serial.printf("\r\n");
    Serial.printf("%02x ",readBuffer[i]);
  }
  Serial.printf("\r\n");

}

void loop() {

  
  
}




//Read Flash ID
uint16_t readFlashID(void)
{
  uint16_t id;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(W25X_ManufactDeviceID);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  SPI.transfer(0x00);
  id |= SPI.transfer(0xFF) << 8;
  id |= SPI.transfer(0xFF);
  digitalWrite(chipSelectPin, HIGH);
  return id;
}

//Read Flash SR
//bit7   6   5   4   3   2   1   0
//SPR    RV  TB  BP2 BP1 BP0 WEL BSY
uint8_t readFlashSR(void)
{
  uint8_t status;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(W25X_ReadStatusReg);
  status = SPI.transfer(0xff);
  digitalWrite(chipSelectPin, HIGH);
  return status;
}

//Write Flash SR
void writeFlashSR(uint8_t status)
{
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(W25X_WriteStatusReg);
  SPI.transfer(status);
  digitalWrite(chipSelectPin, HIGH);
}

//Write Enable
void writeEnable(void)
{
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer( W25X_WriteEnable );
  digitalWrite(chipSelectPin, HIGH);
}

//Write Disable
void writeDisable(void)
{
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer( W25X_WriteDisable );
  digitalWrite(chipSelectPin, HIGH);
}

//Erase sector,,4K bytes per sector
void eraseSector(uint32_t sectorNum)
{
  sectorNum *= 4096;
  writeEnable();
  while( ( readFlashSR() & 0x01 ) == 0x01 );  //waite for busy
  
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(W25X_SectorErase);
  SPI.transfer((uint8_t)(sectorNum >> 16));
  SPI.transfer((uint8_t)(sectorNum >> 8));
  SPI.transfer((uint8_t)(sectorNum));
  digitalWrite(chipSelectPin, HIGH);

  while( ( readFlashSR() & 0x01 ) == 0x01 );  //waite for busy
}


//read flash data to pBuf
void readFlashToBuff(uint8_t *pBuf, uint32_t readAddr, uint16_t size)
{
  uint16_t i;
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(W25X_ReadData);
  SPI.transfer((uint8_t)(readAddr >> 16));
  SPI.transfer((uint8_t)(readAddr >> 8));
  SPI.transfer((uint8_t)(readAddr));
  for(i=0; i<size; i++)
  {
    pBuf[i] = SPI.transfer(0xff);
  }
  digitalWrite(chipSelectPin, HIGH);
}



//write data to flash
void writeFlashPage(uint8_t *pBuf, uint32_t writeAddr, uint16_t size)
{
  uint16_t i;
  writeEnable();
  while( ( readFlashSR() & 0x01 ) == 0x01 );  //waite for busy 
  digitalWrite(chipSelectPin, LOW);
  SPI.transfer(W25X_PageProgram);
  SPI.transfer((uint8_t)(writeAddr >> 16));
  SPI.transfer((uint8_t)(writeAddr >> 8));
  SPI.transfer((uint8_t)(writeAddr));
  for(i=0; i<size; i++)
  {
     SPI.transfer(pBuf[i]);
  }
  digitalWrite(chipSelectPin, HIGH);
  while( ( readFlashSR() & 0x01 ) == 0x01 );  //waite for busy 
}

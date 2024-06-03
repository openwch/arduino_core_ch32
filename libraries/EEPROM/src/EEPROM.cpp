/*
  EEPROM - Enables reading and writing to non-volatile storage in the processor.
  
  Ported by Maxint R&D to CH32, based on multiple sources, see EEPROM.h.

  From the CH32V004 Data Sheet:
   "Built-in 1920 bytes of system storage (System FLASH) for system bootloader storage (factory-cured
   bootloader) 64 bytes are used for the system non-volatile configuration information storage area and 64 bytes
   are used for the user select word storage area."
  So next to 16kB of Code FLASH, the chip features 2kB Flash for boot, configuration and storage. That 64 bytes
  user select word storage area is the area we can use to emulate on chip EEPROM memory.

  From the CH32V003 Reference Manual:
   "The user-option bytes is solidified in FLASH and will be reloaded into the corresponding register after system
    reset, and can be erased and programmed by the user at will. The user option bytes information block has a
    total of 8 bytes (4 bytes for write protection, 1 byte for read protection, 1 byte for configuration options, and
    2 bytes for storing user data), and each bit has its inverse code bit for checksum during loading."
   These 8 bytes use 16 bytes of flash. The total storage area page is 64 bytes, leaving 48 bytes available, 
   (including required inverse values). This is 24 bytes that can be used plus the two data0 and data1 bytes. 
   Layout for uint8_t _data[26]: { ob[4], ob[6], ob[16...62] ].
*/
#include <EEPROM.h>

#define OB_AVAIL_DATA_START 8  // valid for CH32V003; how about others?

EEPROMClass::EEPROMClass(void) {
}

EEPROMClass::~EEPROMClass() {
  end();
}

uint32_t EEPROMClass::ReadOptionBytes()
{ // Read data0 and data1 option bytes
  // These bytes are special, as in that they are 16-bit words, each having their value in the first byte and the inverse value as second byte.
  // Their registers are automatically filled at boot time, so they can be read before calling EEPROM.begin();
   uint32_t uRes=0;
   uRes|=OB->Data0;
   uRes<<=16;
   uRes|=OB->Data1;
   return uRes;
}


void EEPROMClass::begin(void)
{
  _size = 26;   // Option bytes available on CH32V003: 64 - 16 = 48; 48/2=24, 2+24=26

  // Allocate data buffer and copy the current content from storage
  if(!_data)
  {
    // Allocate the RAM data buffer
    _data = new uint8_t[_size];
    
    // Copy data from the option byte words
    // Using a simple (uint8_t) cast will ignore the inversed value in the second half-word
    uint16_t *ob16p=(uint16_t *)OB_BASE;
    _data[0]=(uint8_t)ob16p[2];   // simple cast ignores the inversed second half-word
    _data[1]=(uint8_t)ob16p[3];   // simple cast ignores the inversed second half-word
    for(int i=2; i<_size; i++)
      _data[i]=(uint8_t)ob16p[OB_AVAIL_DATA_START+(i-2)];
  }
  _dirty = false;
}


uint8_t * EEPROMClass::getDataPtr() {
  _dirty = true;
  return &_data[0];
}

uint8_t const * EEPROMClass::getConstDataPtr() const {
  return &_data[0];
}

uint8_t EEPROMClass::read( int const idx ) {
  if(_data && idx>=0 && (size_t)idx<_size)
    return(_data[idx]);
  return(0);
}

void EEPROMClass::write( int const idx, uint8_t  const val ) {
  if(_data && idx>=0 && (size_t)idx<_size) {
    _dirty = true;
    _data[idx]=val;
  }
}

void EEPROMClass::erase(void) {
  _dirty = true;
  for (int i=0;i<_size; i++)
    _data[i]=0xFF;
}


bool EEPROMClass::commit()
{ // Write the _data array to the available area for option bytes
  // Based on example code https://github.com/cnlohr/ch32v003fun/tree/master/examples/optiondata by @cnlohr
  if(!_dirty)
    return(true);

  volatile uint16_t hold[8]; 		// array to hold reserved values while erasing
  // The entire 64 byte data block of the "User-selected words" will be erased
  // so we need to keep a copy of the reserved content for re-writing after erase.
  // Save a few (20) bytes code space by moving 32 bits at a time.
  //  hold[0]=OB->RDPR;       // reserved for read protection
  //  hold[1]=OB->USER;       // reserved for user settings
  //  hold[2]=data0;          // available user data byte (second byte holds inverse)
  //  hold[3]=data1;          // available user data byte (second byte holds inverse)
  //  hold[4]=OB->WRPR0;      // reserved for 16kB write protection
  //  hold[5]=OB->WRPR1;      // reserved for 16kB write protection
  //  hold[6]=OB->WRPR2;      // reserved value
  //  hold[7]=OB->WRPR3;      // reserved value
  uint32_t *hold32p=(uint32_t *)hold;
  uint32_t *ob32p=(uint32_t *)OB_BASE;
  hold32p[0]=ob32p[0];    // Copy RDPR and USER
  hold32p[1]=_data[0]+(_data[1]<<16);	// Copy in the two Data values that are not reserved
  hold32p[2]=ob32p[2];    // Copy WRPR0 and WEPR1
  hold32p[3]=ob32p[3];    // Copy reserved WRPR2 and WEPR3

  // Unlock both the general Flash and the User-selected words
  FLASH->KEYR = FLASH_KEY1;
  FLASH->KEYR = FLASH_KEY2;
  FLASH->OBKEYR = FLASH_KEY1;
  FLASH->OBKEYR = FLASH_KEY2;

  FLASH->CTLR |= CR_OPTER_Set;        // OBER RW Perform user-selected word erasure	
  FLASH->CTLR |= CR_STRT_Set;         // STRT RW1 Start. Set 1 to start an erase action,hw automatically clears to 0
  while (FLASH->STATR & FLASH_BUSY);  // Wait for flash operation to be done
  FLASH->CTLR &= CR_OPTER_Reset;      // Disable erasure mode	

  // Write the held values back one-by-one
  FLASH->CTLR |= CR_OPTPG_Set;   			// OBG  RW Perform user-selected word programming
  uint16_t *ob16p=(uint16_t *)OB_BASE;
  for (int i=0;i<sizeof(hold)/sizeof(hold[0]); i++) {
    ob16p[i]=hold[i];
    while (FLASH->STATR & FLASH_BUSY);	// Wait for flash operation to be done
  }

  // Then write the remainder of the data block
  // Note that when a byte is written as a word, it automatically gets its inverse value as second byte
  if(_data && _size)
  {
    uint16_t *ob16p=(uint16_t *)OB_BASE;
    for (int i=2;i<_size; i++) {
      if(_data[i]!=0xFF) {
        ob16p[OB_AVAIL_DATA_START+(i-2)]=_data[i];
        while (FLASH->STATR & FLASH_BUSY);	// Wait for flash operation to be done
      }
    }
  }
    
  FLASH->CTLR &= CR_OPTPG_Reset;			// Disable programming mode
  FLASH->CTLR|=CR_LOCK_Set;				// Lock flash memories again
  return(true);
}

bool EEPROMClass::end() {
  bool retval;

  retval = commit();
  _data = nullptr;
  _size = 0;
  return(retval);
}

#if !defined(NO_GLOBAL_INSTANCES) && !defined(NO_GLOBAL_EEPROM)
EEPROMClass EEPROM;
#endif

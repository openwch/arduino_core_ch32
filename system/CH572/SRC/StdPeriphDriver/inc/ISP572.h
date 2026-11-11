/* CH572 Flash-ROM & Data-Flash  */
/* Website:  http://wch.cn       */
/* Email:    tech@wch.cn         */
/* Author:   W.ch 2024.12        */
/* V1.0 FlashROM library for USER/BOOT */
/* for the target in USER code area on the chip divided into USER code area and BOOT area */
/* 用于具有用户代码区和引导区的芯片、操作目标为用户代码区的情况，
   可以在用户代码中被调用（IAP，擦写自身），也可以在引导代码中被调用（更新用户代码） */

/* Flash-ROM feature:
     for store program code, support block erasing, dword and page writing, dword verifying, unit for Length is byte,
     minimal quantity for write or verify is one dword (4-bytes),
     256 bytes/page for writing, FLASH_ROM_WRITE support one dword or more dword writing, but multiple of 256 is the best,
     4KB (4096 bytes) bytes/block for erasing, so multiple of 4096 is the best */

#ifndef FLASH_MIN_WR_SIZE
#define FLASH_MIN_WR_SIZE   4                         // Flash-ROM minimal size for writing
#endif
#ifndef FLASH_ROM_MAX_SIZE
#define FLASH_ROM_MAX_SIZE  0x03C000                  // Flash-ROM maximum program size, 240KB
#endif
#ifndef FLASH_BLOCK_SIZE
#define FLASH_BLOCK_SIZE    4096
#endif
// CMD_* for caller from FlashROM or RAM, auto execute CMD_FLASH_ROM_SW_RESET before command
#ifndef CMD_FLASH_ROM_SW_RESET
#define CMD_FLASH_ROM_START_IO  0x00        // start FlashROM I/O, without parameter
#define CMD_FLASH_ROM_SW_RESET  0x04        // software reset FlashROM, without parameter
#define CMD_GET_ROM_INFO        0x06        // get information from FlashROM, parameter @Address,Buffer
#define CMD_GET_UNIQUE_ID       0x07        // get 64 bit unique ID, parameter @Buffer
#define CMD_FLASH_ROM_PWR_DOWN  0x0D        // power-down FlashROM, without parameter
#define CMD_FLASH_ROM_PWR_UP    0x0C        // power-up FlashROM, without parameter
#define CMD_FLASH_ROM_ERASE     0x01        // erase FlashROM block, return 0 if success, parameter @StartAddr,Length
#define CMD_FLASH_ROM_WRITE     0x02        // write FlashROM data block, minimal block is dword, return 0 if success, parameter @StartAddr,Buffer,Length
#define CMD_FLASH_ROM_VERIFY    0x03        // read FlashROM data block, minimal block is dword, return 0 if success, parameter @StartAddr,Buffer,Length
#endif

#define ROM_CFG_MAC_ADDR        0x3F018     // address for MAC address information
#define ROM_CFG_BOOT_INFO       0x3DFF8     // address for BOOT information

/**
 * @brief   execute Flash/EEPROM command, caller from FlashROM or RAM
 *
 * @param   cmd         - CMD_* for caller from FlashROM or RAM.
 * @param   StartAddr   - Address of the data to be process.
 * @param   Buffer      - Pointer to the buffer where data should be process, Must in RAM and be aligned to 4 bytes.
 * @param   Length      - Size of data to be process, in bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
extern uint32_t FLASH_EEPROM_CMD( uint8_t cmd, uint32_t StartAddr, void *Buffer, uint32_t Length );

/**
 * @brief   start FlashROM I/O
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_START_IO( )                       FLASH_EEPROM_CMD( CMD_FLASH_ROM_START_IO, 0, NULL, 0 )

/**
 * @brief   software reset FlashROM
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_SW_RESET( )                       FLASH_EEPROM_CMD( CMD_FLASH_ROM_SW_RESET, 0, NULL, 0 )

/**
 * @brief   get 6 bytes MAC address
 *
 * @param   Buffer      - Pointer to the buffer where data should be stored, Must in RAM and be aligned to 4 bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define GetMACAddress(Buffer)                       FLASH_EEPROM_CMD( CMD_GET_ROM_INFO, ROM_CFG_MAC_ADDR, Buffer, 0 )

/**
 * @brief   get 8 bytes BOOT information
 *
 * @param   Buffer      - Pointer to the buffer where data should be stored, Must in RAM and be aligned to 4 bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define GET_BOOT_INFO(Buffer)                       FLASH_EEPROM_CMD( CMD_GET_ROM_INFO, ROM_CFG_BOOT_INFO, Buffer, 0 )

/**
 * @brief   power-down FlashROM
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_PWR_DOWN( )                       FLASH_EEPROM_CMD( CMD_FLASH_ROM_PWR_DOWN, 0, NULL, 0 )

/**
 * @brief   power-up FlashROM
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_PWR_UP( )                         FLASH_EEPROM_CMD( CMD_FLASH_ROM_PWR_UP, 0, NULL, 0 )

/**
 * @brief   erase FlashROM block
 *
 * @param   StartAddr   - Address of the data to be erased.
 * @param   Length      - Size of data to be erased, in bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_ERASE(StartAddr,Length)           FLASH_EEPROM_CMD( CMD_FLASH_ROM_ERASE, StartAddr, NULL, Length )

/**
 * @brief   write FlashROM data block, minimal block is dword.
 *
 * @param   StartAddr   - Address of the data to be written.
 * @param   Buffer      - Pointer to the source buffer, Must in RAM and be aligned to 4 bytes.
 * @param   Length      - Size of data to be written, in bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_WRITE(StartAddr,Buffer,Length)    FLASH_EEPROM_CMD( CMD_FLASH_ROM_WRITE, StartAddr, Buffer, Length )

/**
 * @brief   verify FlashROM data block, minimal block is dword.
 *
 * @param   StartAddr   - Address of the data to verify.
 * @param   Buffer      - Pointer to the source buffer, Must in RAM and be aligned to 4 bytes.
 * @param   Length      - Size of data to verify, in bytes.
 *
 * @return  0-SUCCESS  (!0)-FAILURE
 */
#define FLASH_ROM_VERIFY(StartAddr,Buffer,Length)   FLASH_EEPROM_CMD( CMD_FLASH_ROM_VERIFY, StartAddr, Buffer, Length )


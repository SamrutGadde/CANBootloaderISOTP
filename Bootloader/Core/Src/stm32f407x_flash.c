#include "stm32f4xx_hal.h"
#include "stm32f407x_flash.h"
#include "stdio.h"

/* STM32F407VET6 has:-
 *  Sector 0 to Sector 3 each 16KB
 *  Sector 4 as 64KB
 *  Sector 5 to Sector 11 each 128KB
 */

static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if ((Address < 0x08003FFF) && (Address >= 0x08000000))
  {
    sector = FLASH_SECTOR_0;
  }
  else if ((Address < 0x08007FFF) && (Address >= 0x08004000))
  {
    sector = FLASH_SECTOR_1;
  }
  else if ((Address < 0x0800BFFF) && (Address >= 0x08008000))
  {
    sector = FLASH_SECTOR_2;
  }
  else if ((Address < 0x0800FFFF) && (Address >= 0x0800C000))
  {
    sector = FLASH_SECTOR_3;
  }
  else if ((Address < 0x0801FFFF) && (Address >= 0x08010000))
  {
    sector = FLASH_SECTOR_4;
  }
  else if ((Address < 0x0803FFFF) && (Address >= 0x08020000))
  {
    sector = FLASH_SECTOR_5;
  }
  else if ((Address < 0x0805FFFF) && (Address >= 0x08040000))
  {
    sector = FLASH_SECTOR_6;
  }
  else if ((Address < 0x0807FFFF) && (Address >= 0x08060000))
  {
    sector = FLASH_SECTOR_7;
  }
  else if ((Address < 0x0809FFFF) && (Address >= 0x08080000))
  {
    sector = FLASH_SECTOR_8;
  }
  else if ((Address < 0x080BFFFF) && (Address >= 0x080A0000))
  {
    sector = FLASH_SECTOR_9;
  }
  else if ((Address < 0x080DFFFF) && (Address >= 0x080C0000))
  {
    sector = FLASH_SECTOR_10;
  }
  else if ((Address < 0x080FFFFF) && (Address >= 0x080E0000))
  {
    sector = FLASH_SECTOR_11;
  }
  return sector;
}

/*
 * Before this, call flashEraseAppSectors() to
 * erase flash before writing
 * @param offset   - word offset from start of application flash
 * @param data	   - data array
 * @param numWords - amount of words in the array
 * @ret	  0 on success, otherwise failure.
 */
uint32_t flashWriteApp(uint32_t offset, uint32_t *data, uint16_t numWords)
{
  /* Convert offset -> address */
  uint32_t startAddr = FLASH_APP_ADDR + offset;
  size_t soFar = 0;

  HAL_FLASH_Unlock();

  while (soFar < numWords)
  {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, startAddr, data[soFar]) == HAL_OK)
    {
      startAddr += 4;
      soFar++;
    }
    else
    {
      printf("Error while writing flash memory...\n");
      return HAL_FLASH_GetError();
    }
  }

  HAL_FLASH_Lock();

  printf("Wrote %d words successfully!\n", soFar);

  return 0;
}

/*
 * @param startAddr   - starting address to read from
 * @param rxBuf	      - data buffer
 * @param numWords    - amount of words to read
 */
void flashReadData(uint32_t startAddr, uint32_t *rxBuf, uint16_t numWords)
{
  while (numWords--)
  {
    *rxBuf = *(__IO uint32_t *)startAddr;
    startAddr += 4;
    rxBuf++;
  }
}

/*
 * Erases all application sectors from FLASH_APP_ADDR to end of flash.
 */
uint32_t flashEraseAppSectors()
{
  static FLASH_EraseInitTypeDef EraseInitStruct;
  uint32_t sectorErr;

  HAL_FLASH_Unlock();

  uint32_t sectorNum = GetSector(FLASH_APP_ADDR);

  /* Fill EraseInit structure*/
  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  EraseInitStruct.Sector = sectorNum;
  EraseInitStruct.NbSectors = FLASH_SECTOR_11 - sectorNum;

  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
     you have to make sure that these data are rewritten before they are accessed during code
     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
     DCRST and ICRST bits in the FLASH_CR register. */
  if (HAL_FLASHEx_Erase(&EraseInitStruct, &sectorErr) != HAL_OK || sectorErr != 0xFFFFFFFFU)
  {
    printf("Failed Erasing App Sectors...\n");
    return HAL_FLASH_GetError();
  }

  HAL_FLASH_Lock();

  printf("Erased App Sectors Successfully.\n");

  return 0;
}

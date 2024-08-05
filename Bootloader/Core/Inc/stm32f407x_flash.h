/*
 * stm32f407x_flash.h
 *
 *  Created on: Dec 23, 2023
 *      Author: Samrut Gadde
 */

#ifndef INC_STM32F407X_FLASH_H_
#define INC_STM32F407X_FLASH_H_

#define FLASH_APP_ADDR 0x08008000
#define FLASH_BOOTLOADER_ADDR 0x08000000

uint32_t flashWriteApp (uint32_t offset, uint32_t *data, uint16_t numWords);
void flashReadData(uint32_t startAddr, uint32_t *rxBuf, uint16_t numWords);
uint32_t flashEraseAppSectors();


#endif /* INC_STM32F407X_FLASH_H_ */

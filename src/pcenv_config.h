/*
 * pcenv_config.h
 *
 *  Created on: 2020年10月6日
 *      Author: kcchang
 */

#ifndef PCENV_CONFIG_H_
#define PCENV_CONFIG_H_

// KC_DBG start
#define PC_ENV
#include <stdint.h>

typedef struct
{
  uint32_t TypeErase;   /*!< Mass erase or sector Erase.
                             This parameter can be a value of @ref FLASHEx_Type_Erase */

  uint32_t Banks;       /*!< Select banks to erase when Mass erase is enabled.
                             This parameter must be a value of @ref FLASHEx_Banks */

  uint32_t Sector;      /*!< Initial FLASH sector to erase when Mass erase is disabled
                             This parameter must be a value of @ref FLASHEx_Sectors */

  uint32_t NbSectors;   /*!< Number of sectors to be erased.
                             This parameter must be a value between 1 and (max number of sectors - value of Initial sector)*/

  uint32_t VoltageRange;/*!< The device voltage range which defines the erase parallelism
                             This parameter must be a value of @ref FLASHEx_Voltage_Range */

  uint32_t NbPages;
  uint32_t PageAddress;

} FLASH_EraseInitTypeDef;

#define HAL_FLASH_Unlock() {}
#define FLASH_TYPEERASE_PAGES  1
#define FLASH_PAGE_SIZE 1
#define FLASH_TYPEPROGRAM_WORD        0x00000002U  /*!< Program a word (32-bit) at a specified address        */

typedef enum
{
  HAL_OK       = 0x00U,
  HAL_ERROR    = 0x01U,
  HAL_BUSY     = 0x02U,
  HAL_TIMEOUT  = 0x03U
} HAL_StatusTypeDef;
// KC_DBG end

#endif /* PCENV_CONFIG_H_ */

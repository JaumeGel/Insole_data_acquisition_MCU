/*
 * user_spi_interface.h
 *
 *  Created on: Dec 12, 2025
 *      Author: Jaume Gelabert Crespo
 *      Code adapted from pcap04-sample-code available in github
 *      https://github.com/sciosense/pcap04-sample-code
 *
 */

#ifndef USER_SPI_INTERFACE_H
#define USER_SPI_INTERFACE_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdint.h>
#include "stm32u5xx_hal.h"
#include "main.h"

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

#define LOW ((uint8_t)0)
#define HIGH ((uint8_t)1)

#define ON	((uint8_t)1)
#define OFF ((uint8_t)0)

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/

/* USER CODE BEGIN EFP */
extern void Set_SSN              (uint8_t channel, uint8_t level);

extern void Write_Opcode            (uint8_t chan, uint8_t one_byte);
extern void Write_Byte_Auto_Incr    (uint8_t chan, int opcode, int address, uint8_t *byte_array, int to_addr); //used by PICOCAP
extern void Read_Byte_Auto_Incr     (uint8_t chan, int opcode, int address, uint8_t *spiRX, int to_addr); //used by PICOCAP

extern uint8_t  Read_Byte2     (uint8_t chan, uint8_t rd_opcode); //used by PICOCAP
extern uint32_t Read_Dword_Lite(uint8_t chan, uint8_t rd_opcode, uint8_t address); //function has to be configured

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* USER_SPI_INTERFACE_H */

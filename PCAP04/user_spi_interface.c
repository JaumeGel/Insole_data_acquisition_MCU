/*
 * user_spi_interface.c
 *
 *  Created on: Dec 12, 2025
 *      Author: Jaume Gelabert Crespo
 *      Code adapted from pcap04-sample-code available in github
 *      https://github.com/sciosense/pcap04-sample-code
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "user_spi_interface.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

extern SPI_HandleTypeDef hspi2;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*                                  Set SSN                                   */
/******************************************************************************/
/**
  * @brief  Set or clear the level of selected SSN pin
  * @param	channel specifies the SPI CS channel to use
  * 		@arg 1: CS1 Device 1
  * 		@arg 2: CS2 Device 2
  * 		@arg 3: CS3 Device 3
  * @param  level specifies the level to be written to SSN pin
  * 		@arg PCAP_LOW: to clear the SSN pin
  * 		@arg PCAP_HIGH: to set the SSN pin
  * @retval none
  */
void Set_SSN(uint8_t channel, uint8_t level)
{
	GPIO_TypeDef* SSN_Port;
	uint16_t SSN_Pin;

	if(channel == 1)
	{
		SSN_Port = SPI2_CS1_GPIO_Port;
		SSN_Pin = SPI2_CS1_Pin;
	}else if(channel == 2)
	{
		SSN_Port = SPI2_CS2_GPIO_Port;
		SSN_Pin = SPI2_CS2_Pin;
	}else if(channel == 3)
	{
		SSN_Port = SPI2_CS3_GPIO_Port;
		SSN_Pin = SPI2_CS3_Pin;
	}else
	{
		return;
	}
	if(level == PCAP_LOW) {
		HAL_GPIO_WritePin(SSN_Port, SSN_Pin, GPIO_PIN_RESET);
	}else if(level == PCAP_HIGH) {
		HAL_GPIO_WritePin(SSN_Port, SSN_Pin, GPIO_PIN_SET);
	}else
	{
		return;
	}

  return;
}

/******************************************************************************/
/*                            Write one byte Opcode                           */
/******************************************************************************/
/**
  * @brief  Write one byte Opcode.
  * @param  chan: channel to write into (1, 2 or 3)
  * @param  one_byte
  * @retval none
  */
void Write_Opcode(uint8_t chan, uint8_t one_byte)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;

  /* 1. Put SSN low - Activate */
  Set_SSN(chan, PCAP_LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi2, &one_byte, 1, timeout);

  /* 3. Put SSN high - Deactivate */
  Set_SSN(chan, PCAP_HIGH);

  return;
}

/******************************************************************************/
/*                       Write bytes auto incrementally                       */
/* Note: Used for PICOCAP devices                                             */
/******************************************************************************/
/**
  * @brief  Write bytes incrementally.
  * @param  chan: channel to write into (1, 2 or 3)
  * @param  opcode (byte)
  * @param  address (byte)
  * @param  byte_array
  * @param  to_addr (32 bit)
  * @retval none
  */
void Write_Byte_Auto_Incr(uint8_t chan, int opcode, int address, uint8_t *byte_array, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2]; //max index

  //opcodes for memory access
  //WR_CFG = 0xA3C0
  //RD_CFG = 0x23C0
  //opcodes for memory access
  //WR = 0xA0
  //RD = 0x20

  if (opcode<0x100) {
	  spiTX[0] = (uint8_t)opcode | (uint8_t)(address>>8);
	  spiTX[1] = (uint8_t)(address);
  } else {
	  spiTX[0] = (uint8_t)(opcode>>8);
	  spiTX[1] = (uint8_t)(opcode) | (uint8_t)(address);
  }

  /* 1. Put SSN low - Activate */
  Set_SSN(chan, PCAP_LOW);

  /* 2.a Transmit register address */
  HAL_SPI_Transmit(&hspi2, spiTX, 2, timeout);

  /* 2.b Transmit register address incrementally */
  for (int i = address; i <= to_addr; i++) {
    HAL_SPI_Transmit(&hspi2, byte_array, 1, timeout);

    byte_array++;
  }

  /* 3. Put SSN high - Deactivate */
  Set_SSN(chan, PCAP_HIGH);

  return;
}

/******************************************************************************/
/*                        Read bytes auto incrementally                       */
/* Note: Used for PICOCAP devices                                             */
/******************************************************************************/
/**
  * @brief  Read byte array auto incrementally.
  * @param  chan: channel to read from (1, 2 or 3)
  * @param  opcode (byte),       "Byte2",       Bit[7:2]
  * @param  address (byte 1),    "Byte2+Byte1", Bit[9:0]
  * @param  byte array (byte 0), "Byte0"
  * @param  to address
  * @retval none
  */
void Read_Byte_Auto_Incr(uint8_t chan, int opcode, int address, uint8_t *spiRX, int to_addr)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[2]; //max index

  uint16_t n_byte = 0;
  n_byte = (to_addr - address) + 1;

  //opcodes for memory access
  //WR_CFG = 0xA3C0
  //RD_CFG = 0x23C0
  //opcodes for memory access
  //WR = 0xA0
  //RD = 0x20

  if (opcode<0x100) {
	  spiTX[0] = (uint8_t)(opcode) | (uint8_t)(address>>8);
	  spiTX[1] = (uint8_t)(address);
  } else {
	  spiTX[0] = (uint8_t)(opcode>>8);
	  spiTX[1] = (uint8_t)(opcode) | (uint8_t)(address);
  }

  /* 1. Put SSN low - Activate */
  Set_SSN(chan, PCAP_LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi2, spiTX, 2, timeout);

  /* 3. Read n bytes */
  HAL_SPI_Receive(&hspi2, spiRX, n_byte, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(chan, PCAP_HIGH);

  return;
}

/******************************************************************************/
/*                                 Read byte2                                 */
/*              E.g., used by PICOCAP by performing a TEST READ               */
/******************************************************************************/
/**
  * @brief  Read byte.
  * @param  chan: channel to read from (1, 2 or 3)
  * @param  opcode (byte)
  * @retval 8-bit value
  */
uint8_t Read_Byte2(uint8_t chan, uint8_t rd_opcode)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  uint8_t spiRX[1];

  spiTX[0] = rd_opcode;

  /* 1. Put SSN low - Activate */
  Set_SSN(chan, PCAP_LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi2, spiTX, 1, timeout);

  /*3. Read one bytes */
  HAL_SPI_Receive(&hspi2, spiRX, 1, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(chan, PCAP_HIGH);

  return spiRX[0];
}

/******************************************************************************/
/*                              Read double word                              */
/******************************************************************************/
/**
  * @brief  Read double word.
  * @param  chan: channel to read from (1, 2 or 3)
  * @param  opcode (byte)
  * @param  address (byte)
  * @retval 32-bit value
  */
uint32_t Read_Dword_Lite(uint8_t chan, uint8_t rd_opcode, uint8_t address)
{
  /* Timeout duration in millisecond [ms] */
  uint8_t timeout = 10;
  uint8_t spiTX[1];
  uint8_t spiRX[4];
  uint32_t temp_u32 = 0;

  spiTX[0] = rd_opcode | address;

  /* 1. Put SSN low - Activate */
  Set_SSN(chan, PCAP_LOW);

  /* 2. Transmit register address */
  HAL_SPI_Transmit(&hspi2, spiTX, 1, timeout);

  /*3. Read four bytes */
  HAL_SPI_Receive(&hspi2, spiRX, 4, timeout);

  /* 4. Put SSN high - Deactivate */
  Set_SSN(chan, PCAP_HIGH);

  /*Concatenate of bytes (from LSB to MSB), e.g. used by PICOCAP */
  temp_u32 = ((uint32_t)spiRX[3] << 24) | ((uint32_t)spiRX[2] << 16) | ((uint32_t)spiRX[1] << 8) | ((uint32_t)spiRX[0]);

  return temp_u32;
}

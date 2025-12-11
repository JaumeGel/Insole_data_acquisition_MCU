/*
 * app_bluenrg_2.h
 *
 *  Created on: Dec 10, 2025
 *      Author: Jaume Gelabert
 *      Adapted from SensorDemo_BLESensor-App application from X-Cube-BLE2
 */

#ifndef APP_BLUENRG_2_H
#define APP_BLUENRG_2_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported Defines ----------------------------------------------------------*/
/*************** Don't Change the following defines *************/

/* Package Version only numbers 0->9 */
#define PACK_VERSION_MAJOR '1'
#define PACK_VERSION_MINOR '0'

/* Define the application Name (MUST be 7 char long) */
#define APP_NAME 'R','C','S','E','N',PACK_VERSION_MAJOR,PACK_VERSION_MINOR

/* Package Name */
#define BLUENRG_PACKAGENAME "X-CUBE-BLE2"

/* USER CODE BEGIN ED */

/* USER CODE END ED */

/* Exported Variables --------------------------------------------------------*/

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/* Exported Functions Prototypes ---------------------------------------------*/
void MX_BlueNRG_2_Init(uint8_t TxPower);
void MX_BlueNRG_2_UpdateData(uint16_t Resistive[], uint32_t Capacitive[], uint8_t Dataselect);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

#ifdef __cplusplus
}
#endif

#endif /* APP_BLUENRG_2_H */

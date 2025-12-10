/*
 * gatt_db.h
 *
 *  Created on: Dec 9, 2025
 *      Author: Jaume Gelabert
 *      Adapted from SensorDemo_BLESensor-App application from X-Cube-BLE2
 */

#ifndef GATT_DB_H
#define GATT_DB_H

/* Includes ------------------------------------------------------------------*/
#include "hci.h"

/* Exported defines ----------------------------------------------------------*/

/**
 * @brief Number of application services
 */
#define NUMBER_OF_APPLICATION_SERVICES (2)

/* Exported typedef ----------------------------------------------------------*/


/* Exported function prototypes ----------------------------------------------*/
tBleStatus Add_HWServW2ST_Service(void);
void Read_Request_CB(uint16_t handle);
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle,
                                   uint16_t Offset, uint8_t data_length, uint8_t *att_data);
tBleStatus Res_Update(uint16_t resistive[]);
tBleStatus Cap_Update(uint32_t capacitive[]);

#endif /* GATT_DB_H */

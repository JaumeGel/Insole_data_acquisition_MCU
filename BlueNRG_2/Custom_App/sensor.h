/*
 * sensor.h
 *
 *  Created on: Dec 9, 2025
 *      Author: Jaume Gelabert
 *      Copied from SensorDemo_BLESensor-App application from X-Cube-BLE2
 */

#ifndef SENSOR_H
#define SENSOR_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
#define SENSOR_DEMO_NAME   'B','l','u','e','N','R','G'
#define BDADDR_SIZE        6

/* Exported function prototypes ----------------------------------------------*/
void Set_DeviceConnectable(void);
void APP_UserEvtRx(void *pData);


#endif /* SENSOR_H */

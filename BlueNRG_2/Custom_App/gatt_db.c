/*
 * gatt_db.c
 *
 *  Created on: Dec 9, 2025
 *      Author: Jaume Gelabert
 *      Adapted from SensorDemo_BLESensor-App application from X-Cube-BLE2
 */

/* Includes ------------------------------------------------------------------*/
#include <stdio.h>
#include <stdlib.h>
#include "gatt_db.h"
#include "bluenrg1_aci.h"
#include "bluenrg1_hci_le.h"
#include "bluenrg1_gatt_aci.h"
#include "app_bluenrg_2.h"

/* Private macros ------------------------------------------------------------*/
/** @brief Macro that stores Value into a buffer in Little Endian Format (2 bytes)*/
#define HOST_TO_LE_16(buf, val)    ( ((buf)[0] =  (uint8_t) (val)    ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8) ) )

/** @brief Macro that stores Value into a buffer in Little Endian Format (4 bytes) */
#define HOST_TO_LE_32(buf, val)    ( ((buf)[0] =  (uint8_t) (val)     ) , \
                                   ((buf)[1] =  (uint8_t) (val>>8)  ) , \
                                   ((buf)[2] =  (uint8_t) (val>>16) ) , \
                                   ((buf)[3] =  (uint8_t) (val>>24) ) )

#define COPY_UUID_128(uuid_struct, uuid_15, uuid_14, uuid_13, uuid_12, uuid_11, uuid_10, uuid_9, uuid_8, uuid_7, uuid_6, uuid_5, uuid_4, uuid_3, uuid_2, uuid_1, uuid_0) \
do {\
    uuid_struct[0] = uuid_0; uuid_struct[1] = uuid_1; uuid_struct[2] = uuid_2; uuid_struct[3] = uuid_3; \
        uuid_struct[4] = uuid_4; uuid_struct[5] = uuid_5; uuid_struct[6] = uuid_6; uuid_struct[7] = uuid_7; \
            uuid_struct[8] = uuid_8; uuid_struct[9] = uuid_9; uuid_struct[10] = uuid_10; uuid_struct[11] = uuid_11; \
                uuid_struct[12] = uuid_12; uuid_struct[13] = uuid_13; uuid_struct[14] = uuid_14; uuid_struct[15] = uuid_15; \
}while(0)

/* Hardware Characteristics Service */
#define COPY_HW_SENS_W2ST_SERVICE_UUID(uuid_struct)	COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0x9a,0xb4,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_RES_W2ST_CHAR_UUID(uuid_struct)		COPY_UUID_128(uuid_struct,0x00,0x00,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)
#define COPY_CAP_W2ST_CHAR_UUID(uuid_struct)		COPY_UUID_128(uuid_struct,0x00,0xE0,0x00,0x00,0x00,0x01,0x11,0xe1,0xac,0x36,0x00,0x02,0xa5,0xd5,0xc5,0x1b)

/* Private variables ---------------------------------------------------------*/
uint16_t HWServW2STHandle, ResCharHandle, CapCharHandle;

/* UUIDS */
Service_UUID_t service_uuid;
Char_UUID_t char_uuid;

extern uint16_t ResMeasurements[15];
extern uint32_t CapMeasurements[15];
__IO uint8_t send_res;
__IO uint8_t send_cap;

extern __IO uint16_t connection_handle;
extern uint32_t start_time;

/* Private functions ---------------------------------------------------------*/
/**
 * @brief  Add the 'HW' service (and the Resistive and Capacitive characteristics).
 * @param  None
 * @retval tBleStatus Status
 */
tBleStatus Add_HWServW2ST_Service(void)
{
  tBleStatus ret;
  uint8_t uuid[16];
  /* num of characteristics of this service */
  uint8_t char_number = 2;
  /* number of attribute records that can be added to this service */
  uint8_t max_attribute_records = 1+(3*char_number);

  /* add HW_SENS_W2ST service */
  COPY_HW_SENS_W2ST_SERVICE_UUID(uuid);
  BLUENRG_memcpy(&service_uuid.Service_UUID_128, uuid, 16);
  ret = aci_gatt_add_service(UUID_TYPE_128, &service_uuid, PRIMARY_SERVICE,
                             max_attribute_records, &HWServW2STHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Resistive BLE Characteristc */
  COPY_RES_W2ST_CHAR_UUID(uuid);
  uuid[14] |= 0x04; /* One Temperature value*/ /*Check these parameters for sending 15 resistive values*/
  //uuid[14] |= 0x10; /* Pressure value*/	//Possibly remove this line
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  //Char_Value_Length set to 2 time bytes + 2bytes*15channels resistive values (each byte is 8 bits)
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid,
                           2+2*15,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &ResCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  /* Fill the Capacitive BLE Characteristc */
  COPY_CAP_W2ST_CHAR_UUID(uuid);
  BLUENRG_memcpy(&char_uuid.Char_UUID_128, uuid, 16);
  //Char_Value_Length set to 2 time bytes + 4bytes*15channels capacitive values (each byte is 8 bits)
  ret =  aci_gatt_add_char(HWServW2STHandle, UUID_TYPE_128, &char_uuid,
                           2+4*15,
                           CHAR_PROP_NOTIFY|CHAR_PROP_READ,
                           ATTR_PERMISSION_NONE,
                           GATT_NOTIFY_READ_REQ_AND_WAIT_FOR_APPL_RESP,
                           16, 0, &CapCharHandle);
  if (ret != BLE_STATUS_SUCCESS)
    return BLE_STATUS_ERROR;

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Resistive characteristic value
 * @param  uint16_t Array of 15 Resistive readings
 * @retval tBleStatus Status
 */
tBleStatus Res_Update(uint16_t resistive[])
{
  tBleStatus ret;
  uint8_t buff[2+2*15];
  HOST_TO_LE_16(buff, HAL_GetTick()>>3);
  uint8_t i = 0;

  for(i=0; i < 15;i++)
  {
	  HOST_TO_LE_16(buff + 2*(i+1), resistive[i]);
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, ResCharHandle,
                                   0, 2+2*15, buff);

  if (ret != BLE_STATUS_SUCCESS){
    PRINT_DBG("Error while updating RES characteristic: 0x%04X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update Capacitive characteristic value
 * @param  uint32_t Array of 15 Capacitive readings
 * @retval tBleStatus Status
 */
tBleStatus Cap_Update(uint32_t capacitive[])
{
  uint8_t buff[2+4*15];
  tBleStatus ret;
  uint8_t i = 0;

  HOST_TO_LE_16(buff,(HAL_GetTick()>>3));

  for(i=0; i < 15;i++)
  {
	  HOST_TO_LE_32(buff + 2 + 4*i, capacitive[i]);
  }

  ret = aci_gatt_update_char_value(HWServW2STHandle, CapCharHandle,
				   0, 2+4*15, buff);
  if (ret != BLE_STATUS_SUCCESS){
    PRINT_DBG("Error while updating CAP characteristic: 0x%02X\r\n",ret) ;
    return BLE_STATUS_ERROR ;
  }

  return BLE_STATUS_SUCCESS;
}

/**
 * @brief  Update the sensor value
 *
 * @param  Handle of the characteristic to update
 * @retval None
 */
void Read_Request_CB(uint16_t handle)
{
  tBleStatus ret;

  if(handle == CapCharHandle + 1)
  {
    Cap_Update(CapMeasurements);
  }
  else if (handle == ResCharHandle + 1)
  {
    Res_Update(ResMeasurements);
  }

  if(connection_handle !=0)
  {
    ret = aci_gatt_allow_read(connection_handle);
    if (ret != BLE_STATUS_SUCCESS)
    {
      PRINT_DBG("aci_gatt_allow_read() failed: 0x%02x\r\n", ret);
    }
  }
}

/**
 * @brief  This function is called when there is a change on the gatt attribute.
 *         With this function it's possible to understand if one application
 *         is subscribed to the one service or not.
 *
 * @param  uint16_t att_handle Handle of the attribute
 * @param  uint8_t  *att_data attribute data
 * @param  uint8_t  data_length length of the data
 * @retval None
 */
void Attribute_Modified_Request_CB(uint16_t Connection_Handle, uint16_t attr_handle, uint16_t Offset, uint8_t data_length, uint8_t *att_data)
{
  if(attr_handle == ResCharHandle + 2) {
    if (att_data[0] == 1) {
      send_res = TRUE;
    } else if (att_data[0] == 0){
      send_res = FALSE;
    }
  }
  else if (attr_handle == CapCharHandle +2) {
    if (att_data[0] == 1) {
      send_cap = TRUE;
    } else if (att_data[0] == 0){
      send_cap = FALSE;
    }
  }
}

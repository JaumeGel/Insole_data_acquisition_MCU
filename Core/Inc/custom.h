/*
 * custom.h
 *
 *  Created on: Dec 11, 2025
 *      Author: Jaume Gelabert Crespo
 */

#ifndef CUSTOM_H
#define CUSTOM_H


/**
 * @brief Selector of data to be updated
 */
#define NONE		(0x00)	//No value to be updated
#define RES_ONLY	(0x01)	//Only Resistive values to be updated
#define CAP_ONLY	(0x02)	//Only Capacitive values to be updated
#define RES_CAP		(0x03)	//Both Resistive and Capacitive values to be updated

#endif /* CUSTOM_H */

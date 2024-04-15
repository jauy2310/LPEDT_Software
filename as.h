/*
 * as.h
 *
 * Header file for the AS7331 sensor module
 *
 *  Created on: Mar 29, 2024
 *      Author: Jake Uyechi
 */

#ifndef AS_H_
#define AS_H_

/***************************************************************************//**
 * Get Module Device ID
 ******************************************************************************/
/**
 * as_dev_id()
 *
 * Retrieves the Device ID of the AS7331
 */
void as_dev_id(void);

/***************************************************************************//**
 * Initialize Module
 ******************************************************************************/
/**
 * as_init()
 *
 * Initializes the AS7331 sensor module and compares device ID to expected value
 * as defined in as.c
 */
void as_init(void);

/***************************************************************************//**
 * Report Sensor Data
 ******************************************************************************/
/**
 * as_report()
 *
 * Reports the relevant sensor data after reading and processing
 */
void as_report(void);

/***************************************************************************//**
 * Ticking function
 ******************************************************************************/
/**
 * as_process_action()
 *
 * Performs a full measurement cycle for the AS7331
 */
void as_process_action(void);


#endif /* AS_H_ */

/*
 * mpl.h
 *
 * Header file for the MPL3115A2 sensor module
 *
 *  Created on: Apr 2, 2024
 *      Author: Jake Uyechi
 */

#ifndef MPL_H_
#define MPL_H_

/***************************************************************************//**
 * Get Module Device ID
 ******************************************************************************/
/**
 * mpl_dev_id()
 *
 * Retrieves the device ID of the MPL3115A2
 */
void mpl_dev_id(void);

/***************************************************************************//**
 * Initialize Module
 ******************************************************************************/
/**
 * mpl_init()
 *
 * Initializes the MPL7331 sensor module and compares device ID to expected value
 * as defined in mpl.c
 */
void mpl_init(void);

/***************************************************************************//**
 * Report Sensor Data
 ******************************************************************************/
/**
 * mpl_report()
 *
 * Reports the relevant sensor data after reading and processing
 */
void mpl_report(void);

/***************************************************************************//**
 * Ticking function
 ******************************************************************************/
/**
 * mpl_process_action()
 *
 * Performs a full measurement cycle for the MPL7331
 */
void mpl_process_action(void);

#endif /* MPL_H_ */

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
 * Ticking function
 ******************************************************************************/
/**
 * as_process_action()
 *
 * Performs a full measurement cycle for the AS7331
 */
void as_process_action();


#endif /* AS_H_ */

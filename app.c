/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/

#include <si.h>
#include <as.h>
#include <mpl.h>
#include <stdio.h>
#include "sl_sleeptimer.h"

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
    /**
     * Clear Screen
     */
    printf("\033[2J");

    /**
     * Startup Delay
     */
    sl_sleeptimer_delay_millisecond(500);

    /**
     * Application Start Message - process action following this block of code
     */
    printf("=============================================================\r\n");
    printf(">>> Starting Application");
    for(int i = 0; i < 10; i++) {
            printf(".");
            sl_sleeptimer_delay_millisecond(50);
    }
    printf("\r\n");
    printf("Project: Team 3 - LPHR Weather Station\r\n");
    printf("Authors: Tommy Ramirez and Jake Uyechi\r\n");
    printf("Term: CU SP'24 ECEN5833\r\n");

    /**
     * Application Header - check for device identification numbers
     */
    printf("=============================================================\r\n");
    printf(">>> Retrieving Device Information...\r\n");

    /**
     * Device Identification - readout of device ID numbers
     */
    si_dev_id();
    as_dev_id();
    mpl_dev_id();

    /**
     * Module Initialization - begin initializing devices for measurements
     */
    printf("=============================================================\r\n");
    printf(">>> Beginning Sensor Initialization...\r\n");
    si_init();
    as_init();
    mpl_init();
    printf("=============================================================\r\n");
    printf(">>> Sensors Initialized!\r\n");
    printf("=============================================================\r\n");
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
    /**
     * Ticking Functions for Main Application
     */
//  i2cspm_app_process_action();
    as_process_action();
	mpl_process_action();
    sl_sleeptimer_delay_millisecond(1000);

    /**
     * Report sensor data
     */
    as_report();
    mpl_report();
    printf("=============================================================\r\n");
}

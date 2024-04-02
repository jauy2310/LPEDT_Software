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
#include <stdio.h>

/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
    /**
     * Application Header - begin module initialization
     */
    printf("==============================================\r\n");
    printf("Beginning Module Initialization...\r\n");
    printf("==============================================\r\n");

    /**
     * Module Initialization - call sensor API to initialize all sensors
     */
    i2cspm_app_init();
    as_init();
    //  mpl_init();

    /**
     * Application Start Message - process action following this block of code
     */
    printf("==============================================\r\n");
    printf("Sensors Initialized!\r\n");
    printf("==============================================\r\n");
    printf("Welcome to Team 3 - LPHR Software Application.\r\n");
    printf("Authors: Tommy Ramirez and Jake Uyechi\r\n");
    printf("Term: CU SP'24 ECEN5833\r\n");
    printf("==============================================\r\n");
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  i2cspm_app_process_action();
}

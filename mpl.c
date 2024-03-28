/*
 * mpl.c
 *
 *  Created on: Mar 27, 2024
 *      Author: Jake Uyechi
 */
/*********************************************************
 * Includes
 ********************************************************/
#include <mpl.h>
#include <stdio.h>
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_sleeptimer.h"

/*********************************************************
 * Macros
 ********************************************************/
// define the MPL3115A2 configuration settings
#define MPL_CONFIG_I2C_DEVICE               (sl_i2cspm_sensor)
#define MPL_CONFIG_I2C_BUS_ADDRESS          0x40
#define MPL_CONFIG_SLAVE_ADDRESS            0x60
#define MPL_CONFIG_SLAVE_ADDRESS_READ       0xC1
#define MPL_CONFIG_SLAVE_ADDRESS_WRITE      0xC0
#define MPL_CONFIG_DEVICE_ID                0xC4

// define macros for special registers
#define MPL_REG_WHOAMI                      0x0C

/*********************************************************
 * Local Variables
 ********************************************************/

/*********************************************************
 * Local Functions
 ********************************************************/
/*
 * MPL3115A2_transaction
 */
static I2C_TransferReturn_TypeDef MPL3115A2_transaction(uint16_t flag,
                                                        uint8_t *writeCmd,
                                                        size_t writeLen,
                                                        uint8_t *readCmd,
                                                        size_t readLen)
{
  I2C_TransferSeq_TypeDef seq;
  I2C_TransferReturn_TypeDef ret;

  seq.addr = MPL_CONFIG_I2C_BUS_ADDRESS << 1;
  seq.flags = flag;

  switch (flag) {
    // Send the write command from writeCmd
    case I2C_FLAG_WRITE:
      seq.buf[0].data = writeCmd;
      seq.buf[0].len  = writeLen;

      break;

    // Receive data into readCmd of readLen
    case I2C_FLAG_READ:
      seq.buf[0].data = readCmd;
      seq.buf[0].len  = readLen;

      break;

    // Send the write command from writeCmd
    // and receive data into readCmd of readLen
    case I2C_FLAG_WRITE_READ:
      seq.buf[0].data = writeCmd;
      seq.buf[0].len  = writeLen;

      seq.buf[1].data = readCmd;
      seq.buf[1].len  = readLen;

      break;

    default:
      return i2cTransferUsageFault;
  }

  // Perform the transfer and return status from the transfer
  ret = I2CSPM_Transfer(MPL_CONFIG_I2C_DEVICE, &seq);

  return ret;
}

/*********************************************************
 * Global Functions
 ********************************************************/
/*
 * mpl_init()
 *
 * Initializes the MPL3115A2 module and retrieves device ID
 */
void mpl_init()
{
    I2C_TransferReturn_TypeDef ret;
    uint8_t cmdReadId2[3] = {MPL_CONFIG_SLAVE_ADDRESS_WRITE, MPL_REG_WHOAMI, MPL_CONFIG_SLAVE_ADDRESS_READ};
    uint8_t deviceId[8];

    // Wait for sensor to become ready
    sl_sleeptimer_delay_millisecond(80);

    // Check for device presence  and compare device ID
    // TODO: create custom I2C_Transfer function for this particular device; current does not work with the datasheet
    ret = MPL3115A2_transaction(I2C_FLAG_WRITE_READ, cmdReadId2, 3, deviceId, 8);

    // Print Device ID
    printf("\r\nMPL3115A2 Device ID: 0x%02X\r\n", deviceId[0]);

    // Make sure transfer was successful
    EFM_ASSERT(ret == i2cTransferDone);

    // Check the Received Device ID
    EFM_ASSERT(deviceId[0] == MPL_CONFIG_DEVICE_ID);
}


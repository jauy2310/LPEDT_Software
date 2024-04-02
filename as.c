/*
 * as.c
 *
 *  Created on: Mar 29, 2024
 *      Author: Jake Uyechi
 */

/*********************************************************
 * Includes
 ********************************************************/
#include <as.h>
#include <stdio.h>
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_i2cspm_sensor_config.h"
#include "sl_sleeptimer.h"

/*********************************************************
 * Macros
 ********************************************************/
// define an instance of the AS sensor
//typedef I2C_TypeDef as_i2cspm_t;
//as_i2cspm_t *as_i2cspm_sensor = SL_I2CSPM_SENSOR_PERIPHERAL;

// define the AS3771 configuration settings
#define AS_CONFIG_I2C_DEVICE                    (sl_i2cspm_sensor)
#define AS_CONFIG_SLAVE_ADDRESS                 0x74
#define AS_CONFIG_SLAVE_ADDRESS_READ            (AS_CONFIG_SLAVE_ADDRESS << 1 || 1)
#define AS_CONFIG_SLAVE_ADDRESS_WRITE           (AS_CONFIG_SLAVE_ADDRESS << 1 || 0)
#define AS_CONFIG_DEVICE_ID                     0x21

// define macros for special registers
#define AS_REG_AGEN                             0x2
/*********************************************************
 * Local Variables
 ********************************************************/

/*********************************************************
 * Local Functions
 ********************************************************/
/*
 * MPL3115A2_transaction
 */
static I2C_TransferReturn_TypeDef AS3771_transaction(uint16_t flag,
                                                     uint8_t *writeCmd,
                                                     size_t writeLen,
                                                     uint8_t *readCmd,
                                                     size_t readLen)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    seq.addr = AS_CONFIG_SLAVE_ADDRESS << 1;
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

    // default case
    default:
        return i2cTransferUsageFault;
    }

    // Perform the transfer and return status from the transfer
    ret = I2CSPM_Transfer(AS_CONFIG_I2C_DEVICE, &seq);

    return ret;
}

/*********************************************************
 * Global Functions
 ********************************************************/
/*
 * as_init()
 *
 * Initializes the AS7331 module and retrieves device ID
 */
void as_init()
{
    I2C_TransferReturn_TypeDef ret;
    uint8_t as_read_id_sequence[1] = {AS_REG_AGEN};
    uint8_t as_device_id[8];

    // Wait for sensor to become ready
    sl_sleeptimer_delay_millisecond(80);

    // Check for device presence  and compare device ID
    ret = AS3771_transaction(I2C_FLAG_WRITE_READ, as_read_id_sequence, 1, as_device_id, 8);

    // Print Device ID
    printf("AS3771 Device ID: 0x%02X\r\n", as_device_id[0]);

    // Make sure transfer was successful
    EFM_ASSERT(ret == i2cTransferDone);

    // Check the Received Device ID
    EFM_ASSERT(as_device_id[0] == AS_CONFIG_DEVICE_ID);
}

/*
 * mpl.c
 *
 * Implementation file for the MPL3115A2 sensor module
 *
 *  Created on: Apr 2, 2024
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
// define the AS7331 configuration settings
#define MPL_CONFIG_I2C_DEVICE                    (sl_i2cspm_sensor)
#define MPL_CONFIG_SLAVE_ADDRESS                 0x60
#define MPL_CONFIG_SLAVE_ADDRESS_READ            (AS_CONFIG_SLAVE_ADDRESS << 1 || 1)
#define MPL_CONFIG_SLAVE_ADDRESS_WRITE           (AS_CONFIG_SLAVE_ADDRESS << 1 || 0)
#define MPL_CONFIG_DEVICE_ID                     0xC4

// define macros for special registers
#define MPL_REG_WHOAMI                           0x0C
/*********************************************************
 * Local Variables
 ********************************************************/

/*********************************************************
 * Local Functions
 ********************************************************/
/**
 * MPL7331_transaction()
 *
 * Constructs an I2C sequence for a single transaction between the master (EFR32)
 * and slave (MPL7331).
 *
 * @param flag              The type of I2C transaction being made
 * @param writeCmd          The array of write addresses to send from master to slave
 * @param writeLen          The length of the write array
 * @param readCmd           The array of read addresses to store data from slave to master
 * @param readLen           The length of the read array
 *
 * @return                  The return status of the I2C transaction
 */
static I2C_TransferReturn_TypeDef MPL3115A2_transaction(uint16_t flag,
                                                     uint8_t *writeCmd,
                                                     size_t writeLen,
                                                     uint8_t *readCmd,
                                                     size_t readLen)
{
    I2C_TransferSeq_TypeDef seq;
    I2C_TransferReturn_TypeDef ret;

    seq.addr = MPL_CONFIG_SLAVE_ADDRESS << 1;
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
    ret = I2CSPM_Transfer(MPL_CONFIG_I2C_DEVICE, &seq);

    return ret;
}

/*********************************************************
 * Global Functions
 ********************************************************/
void mpl_init()
{
    I2C_TransferReturn_TypeDef ret;
    uint8_t mpl_read_id_sequence[1] = {MPL_REG_WHOAMI};
    uint8_t mpl_device_id[8];

    // Wait for sensor to become ready
    sl_sleeptimer_delay_millisecond(80);

    // Check for device presence  and compare device ID
    ret = MPL3115A2_transaction(I2C_FLAG_WRITE_READ, mpl_read_id_sequence, 1, mpl_device_id, 8);

    // Print Device ID
    printf("%-10s Device ID: 0x%02X\r\n", "MPL3115A2", mpl_device_id[0]);

    // Make sure transfer was successful
    EFM_ASSERT(ret == i2cTransferDone);

    // Check the Received Device ID
    EFM_ASSERT(mpl_device_id[0] == MPL_CONFIG_DEVICE_ID);
}

void mpl_process_action()
{
    return;
}

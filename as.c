/*
 * as.c
 *
 * Implementation file for the AS7331 sensor module
 *
 *  Created on: Mar 29, 2024
 *      Author: Jake Uyechi
 */

/*********************************************************
 * Includes
 ********************************************************/
#include <as.h>
#include <stdio.h>
#include <string.h>
#include "sl_i2cspm.h"
#include "sl_i2cspm_instances.h"
#include "sl_i2cspm_sensor_config.h"
#include "sl_sleeptimer.h"

/*********************************************************
 * Macros
 ********************************************************/
// define sensor name
#define AS_NAME                                 ("AS7331")

// define the AS7331 configuration settings
#define AS_CONFIG_I2C_DEVICE                    (sl_i2cspm_sensor)
#define AS_CONFIG_SLAVE_ADDRESS                 0x74
#define AS_CONFIG_SLAVE_ADDRESS_READ            (AS_CONFIG_SLAVE_ADDRESS << 1 || 1)
#define AS_CONFIG_SLAVE_ADDRESS_WRITE           (AS_CONFIG_SLAVE_ADDRESS << 1 || 0)
#define AS_CONFIG_DEVICE_ID                     0x21
#define AS_CONFIG_MODE_CONFIGURATION            0
#define AS_CONFIG_MODE_MEASUREMENT              1

// define macros for special registers
// WO - Write-only
// RO - Read-only
// RW - Read/Write
// 1B - 1-byte access
// 2B - 2-byte access

// Configuration Mode Register Access
#define AS_REG_CONFIG_OSR                       0x00 // WO
#define AS_REG_CONFIG_AGEN                      0x02 // RO
#define AS_REG_CONFIG_CREG1                     0x06 // WO
#define AS_REG_CONFIG_CREG2                     0x07 // WO
#define AS_REG_CONFIG_CREG3                     0x08 // WO
#define AS_REG_CONFIG_BREAK                     0x09 // WO
#define AS_REG_CONFIG_EDGES                     0x0A // WO
#define AS_REG_CONFIG_OPTREG                    0x0B // WO

// Measurement Mode Register Access
#define AS_REG_MEAS_OSR                         0x00 // 1BWO
#define AS_REG_MEAS_OSR_STATUS                  0x00 // 2BRO
#define AS_REG_MEAS_TEMP                        0x01 // 2BRO
#define AS_REG_MEAS_MRES1_A                     0x02 // 2BRO
#define AS_REG_MEAS_MRES2_B                     0x03 // 2BRO
#define AS_REG_MEAS_MRES3_C                     0x04 // 2BRO
#define AS_REG_MEAS_OUTCONV_L                   0x05 // 2BRO
#define AS_REG_MEAS_OUTCONV_H                   0x06 // 2BRO

// Register Bitwise Operators

// OSR - Operational State Register
#define AS_REG_CONFIG_OSR_SS_SHIFT              7
#define AS_REG_CONFIG_OSR_SS_MASK               ((0b1) << (AS_REG_CONFIG_OSR_SS_SHIFT))
#define AS_REG_CONFIG_OSR_SS(x)                 (((x) << (AS_REG_CONFIG_OSR_SS_SHIFT)) & (AS_REG_CONFIG_OSR_SS_MASK))

#define AS_REG_CONFIG_OSR_PD_SHIFT              6
#define AS_REG_CONFIG_OSR_PD_MASK               ((0b1) << (AS_REG_CONFIG_OSR_PD_SHIFT))
#define AS_REG_CONFIG_OSR_PD(x)                 (((x) << (AS_REG_CONFIG_OSR_PD_SHIFT)) & (AS_REG_CONFIG_OSR_PD_MASK))

#define AS_REG_CONFIG_OSR_SW_RES_SHIFT          3
#define AS_REG_CONFIG_OSR_SW_RES_MASK           ((0b1) << (AS_REG_CONFIG_OSR_SW_RES_SHIFT))
#define AS_REG_CONFIG_OSR_SW_RES(x)             (((x) << (AS_REG_CONFIG_OSR_SW_RES_SHIFT)) & (AS_REG_CONFIG_OSR_SW_RES_MASK))

#define AS_REG_CONFIG_OSR_DOS_SHIFT             0
#define AS_REG_CONFIG_OSR_DOS_MASK              ((0b111) << (AS_REG_CONFIG_OSR_DOS_SHIFT))
#define AS_REG_CONFIG_OSR_DOS(x)                (((x) << (AS_REG_CONFIG_OSR_DOS_SHIFT)) & (AS_REG_CONFIG_OSR_DOS_MASK))

// CREG1 - Control Register 1
#define AS_REG_CONFIG_CREG1_GAIN_SHIFT          4
#define AS_REG_CONFIG_CREG1_GAIN_MASK           ((0b1111) << (AS_REG_CONFIG_CREG1_GAIN_SHIFT))
#define AS_REG_CONFIG_CREG1_GAIN(x)             (((x) << (AS_REG_CONFIG_CREG1_GAIN_SHIFT)) & (AS_REG_CONFIG_CREG1_GAIN_MASK))
typedef enum {
    // gain multiplier for irradiance responsivity
    // this assumes CREG3:CCLK = 00b (1 MHz), upper range changes based on higher clock frequencies
    CREG1_GAIN_2048 = 0b0000,
    CREG1_GAIN_1024 = 0b0001,
    CREG1_GAIN_512  = 0b0010,
    CREG1_GAIN_256  = 0b0011,
    CREG1_GAIN_128  = 0b0100,
    CREG1_GAIN_64   = 0b0101,
    CREG1_GAIN_32   = 0b0110,
    CREG1_GAIN_16   = 0b0111,
    CREG1_GAIN_8    = 0b1000,
    CREG1_GAIN_4    = 0b1001,
    CREG1_GAIN_2    = 0b1010,
    CREG1_GAIN_1    = 0b1011
} CREG1_GAIN_t;

#define AS_REG_CONFIG_CREG1_TIME_SHIFT          0
#define AS_REG_CONFIG_CREG1_TIME_MASK           ((0b1111) << (AS_REG_CONFIG_CREG1_TIME_SHIFT))
#define AS_REG_CONFIG_CREG1_TIME(x)             (((x) << (AS_REG_CONFIG_CREG1_TIME_SHIFT)) & (AS_REG_CONFIG_CREG1_TIME_MASK))
typedef enum {
    // integration time for measurement (in milliseconds)
    // this assumes a clock frequency of 1024 MHz
    CREG1_TIME_1     = 0b0000,
    CREG1_TIME_2     = 0b0001,
    CREG1_TIME_4     = 0b0010,
    CREG1_TIME_8     = 0b0011,
    CREG1_TIME_16    = 0b0100,
    CREG1_TIME_32    = 0b0101,
    CREG1_TIME_64    = 0b0110,
    CREG1_TIME_128   = 0b0111,
    CREG1_TIME_256   = 0b1000,
    CREG1_TIME_512   = 0b1001,
    CREG1_TIME_1024  = 0b1010,
    CREG1_TIME_2048  = 0b1011,
    CREG1_TIME_4096  = 0b1100,
    CREG1_TIME_8192  = 0b1101,
    CREG1_TIME_16384 = 0b1110,
} CREG1_TIME_t;

// CREG2 - Control Register 2
#define AS_REG_CONFIG_CREG2_EN_TM_SHIFT         6
#define AS_REG_CONFIG_CREG2_EN_TM_MASK          ((0b1) << (AS_REG_CONFIG_CREG2_EN_TM_SHIFT))
#define AS_REG_CONFIG_CREG2_EN_TM(x)            (((x) << (AS_REG_CONFIG_CREG2_EN_TM_SHIFT)) & (AS_REG_CONFIG_CREG2_EN_TM_MASK))

#define AS_REG_CONFIG_CREG2_EN_DIV_SHIFT        3
#define AS_REG_CONFIG_CREG2_EN_DIV_MASK         ((0b1) << (AS_REG_CONFIG_CREG2_EN_DIV_SHIFT))
#define AS_REG_CONFIG_CREG2_EN_DIV(x)           (((x) << (AS_REG_CONFIG_CREG2_EN_DIV_SHIFT)) & (AS_REG_CONFIG_CREG2_EN_DIV_MASK))

#define AS_REG_CONFIG_CREG2_DIV_SHIFT           0
#define AS_REG_CONFIG_CREG2_DIV_MASK            ((0b111) << (AS_REG_CONFIG_CREG2_DIV_SHIFT))
#define AS_REG_CONFIG_CREG2_DIV(x)              (((x) << (AS_REG_CONFIG_CREG2_DIV_SHIFT)) & (AS_REG_CONFIG_CREG2_DIV_MASK))

// CREG3 - Control Register 3
#define AS_REG_CONFIG_CREG3_MMODE_SHIFT         6
#define AS_REG_CONFIG_CREG3_MMODE_MASK          ((0b11) << (AS_REG_CONFIG_CREG3_MMODE_SHIFT))
#define AS_REG_CONFIG_CREG3_MMODE(x)            (((x) << (AS_REG_CONFIG_CREG3_MMODE_SHIFT)) & (AS_REG_CONFIG_CREG3_MMODE_MASK))

#define AS_REG_CONFIG_CREG3_SB_SHIFT            4
#define AS_REG_CONFIG_CREG3_SB_MASK             ((0b1) << (AS_REG_CONFIG_CREG3_SB_SHIFT))
#define AS_REG_CONFIG_CREG3_SB(x)               (((x) << (AS_REG_CONFIG_CREG3_SB_SHIFT)) & (AS_REG_CONFIG_CREG3_SB_MASK))

#define AS_REG_CONFIG_CREG3_RDYOD_SHIFT         3
#define AS_REG_CONFIG_CREG3_RDYOD_MASK          ((0b1) << (AS_REG_CONFIG_CREG3_RDYOD_SHIFT))
#define AS_REG_CONFIG_CREG3_RDYOD(x)            (((x) << (AS_REG_CONFIG_CREG3_RDYOD_SHIFT)) & (AS_REG_CONFIG_CREG3_RDYOD_MASK))

#define AS_REG_CONFIG_CREG3_CCLK_SHIFT          0
#define AS_REG_CONFIG_CREG3_CCLK_MASK           ((0b11) << (AS_REG_CONFIG_CREG3_CCLK_SHIFT))
#define AS_REG_CONFIG_CREG3_CCLK(x)             (((x) << (AS_REG_CONFIG_CREG3_CCLK_SHIFT)) & (AS_REG_CONFIG_CREG3_CCLK_MASK))

// STATUS - Status Register
#define AS_REG_MEAS_STATUS_OUTCONVOF_SHIFT      7
#define AS_REG_MEAS_STATUS_OUTCONVOF_MASK       ((0b1) << (AS_REG_MEAS_STATUS_OUTCONVOF_SHIFT))
#define AS_REG_MEAS_STATUS_OUTCONVOF(x)         (((x) << (AS_REG_MEAS_STATUS_OUTCONVOF_SHIFT)) & (AS_REG_MEAS_STATUS_OUTCONVOF_MASK))

#define AS_REG_MEAS_STATUS_MRESOF_SHIFT         6
#define AS_REG_MEAS_STATUS_MRESOF_MASK          ((0b1) << (AS_REG_MEAS_STATUS_MRESOF_SHIFT))
#define AS_REG_MEAS_STATUS_MRESOF(x)            (((x) << (AS_REG_MEAS_STATUS_MRESOF_SHIFT)) & (AS_REG_MEAS_STATUS_MRESOF_MASK))

#define AS_REG_MEAS_STATUS_ADCOF_SHIFT          5
#define AS_REG_MEAS_STATUS_ADCOF_MASK           ((0b1) << (AS_REG_MEAS_STATUS_ADCOF_SHIFT))
#define AS_REG_MEAS_STATUS_ADCOF(x)             (((x) << (AS_REG_MEAS_STATUS_ADCOF_SHIFT)) & (AS_REG_MEAS_STATUS_ADCOF_MASK))

#define AS_REG_MEAS_STATUS_LDATA_SHIFT          4
#define AS_REG_MEAS_STATUS_LDATA_MASK           ((0b1) << (AS_REG_MEAS_STATUS_LDATA_SHIFT))
#define AS_REG_MEAS_STATUS_LDATA(x)             (((x) << (AS_REG_MEAS_STATUS_LDATA_SHIFT)) & (AS_REG_MEAS_STATUS_LDATA_MASK))

#define AS_REG_MEAS_STATUS_NDATA_SHIFT          3
#define AS_REG_MEAS_STATUS_NDATA_MASK           ((0b1) << (AS_REG_MEAS_STATUS_NDATA_SHIFT))
#define AS_REG_MEAS_STATUS_NDATA(x)             (((x) << (AS_REG_MEAS_STATUS_NDATA_SHIFT)) & (AS_REG_MEAS_STATUS_NDATA_MASK))

#define AS_REG_MEAS_STATUS_NOTREADY_SHIFT       2
#define AS_REG_MEAS_STATUS_NOTREADY_MASK        ((0b1) << (AS_REG_MEAS_STATUS_NOTREADY_SHIFT))
#define AS_REG_MEAS_STATUS_NOTREADY(x)          (((x) << (AS_REG_MEAS_STATUS_NOTREADY_SHIFT)) & (AS_REG_MEAS_STATUS_NOTREADY_MASK))

#define AS_REG_MEAS_STATUS_STANDBYSTATE_SHIFT   1
#define AS_REG_MEAS_STATUS_STANDBYSTATE_MASK    ((0b1) << (AS_REG_MEAS_STATUS_STANDBYSTATE_SHIFT))
#define AS_REG_MEAS_STATUS_STANDBYSTATE(x)      (((x) << (AS_REG_MEAS_STATUS_STANDBYSTATE_SHIFT)) & (AS_REG_MEAS_STATUS_STANDBYSTATE_MASK))

#define AS_REG_MEAS_STATUS_POWERSTATE_SHIFT     0
#define AS_REG_MEAS_STATUS_POWERSTATE_MASK      ((0b1) << (AS_REG_MEAS_STATUS_POWERSTATE_SHIFT))
#define AS_REG_MEAS_STATUS_POWERSTATE(x)        (((x) << (AS_REG_MEAS_STATUS_POWERSTATE_SHIFT)) & (AS_REG_MEAS_STATUS_POWERSTATE_MASK))

/*********************************************************
 * Local Variables
 ********************************************************/
// sensor variables
float temperature;
uint16_t mres1, mres2, mres3;

// i2c buffer
#define I2C_BUFFER_LENGTH 16
uint8_t i2c_write_buf[I2C_BUFFER_LENGTH];
uint8_t i2c_read_buf[I2C_BUFFER_LENGTH];

/*********************************************************
 * Local Functions
 ********************************************************/
/**
 * AS7331_transaction()
 *
 * Constructs an I2C sequence for a single transaction between the master (EFR32)
 * and slave (AS7331).
 *
 * @param flag              The type of I2C transaction being made
 * @param writeCmd          The array of write addresses to send from master to slave
 * @param writeLen          The length of the write array
 * @param readCmd           The array of read addresses to store data from slave to master
 * @param readLen           The length of the read array
 *
 * @return                  The return status of the I2C transaction
 */
static I2C_TransferReturn_TypeDef AS7331_transaction(uint16_t flag,
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

/**
 * flush_buffers
 *
 * Clears I2C buffers
 */
void flush_buffers(void)
{
    memset(i2c_write_buf, 0, I2C_BUFFER_LENGTH);
    memset(i2c_read_buf, 0, I2C_BUFFER_LENGTH);
}

/**
 * AS7331_ChangeMode()
 *
 * Changes the mode (measurement/configuration) of the AS7331 sensor
 *
 * @param new_mode                      Use AS_CONFIG_MODE_XXXXX
 */
void AS7331_ChangeMode(uint8_t new_mode)
{
    // initialize I2C transaction variables
    I2C_TransferReturn_TypeDef ret;
    flush_buffers();
    i2c_write_buf[0] = AS_REG_CONFIG_OSR;

    // read the DOS bits of the OSR
    ret = AS7331_transaction(I2C_FLAG_WRITE_READ, i2c_write_buf, 1, i2c_read_buf, 2);

    // set to configuration mode
    if(new_mode ==  AS_CONFIG_MODE_CONFIGURATION) {
            // check if mode is not already in configuration mode
            if((i2c_read_buf[0] & AS_REG_CONFIG_OSR_DOS_MASK) != 0b010){
                    // create I2C transaction variable to rewrite the OSR
                    uint8_t new_dos = ((i2c_read_buf[0] & ~(AS_REG_CONFIG_OSR_DOS_MASK)) | 0b010);
                    flush_buffers();
                    i2c_write_buf[0] = AS_REG_CONFIG_OSR;
                    i2c_write_buf[1] = new_dos;

                    // perform transaction
                    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);

                    // assert successful transaction and indicate mode change
                    EFM_ASSERT(ret == i2cTransferDone);
            }
    // set to measurement mode
    } else if (new_mode == AS_CONFIG_MODE_MEASUREMENT){
            // check if mode is not already in measurement mode
            if((i2c_read_buf[0] & AS_REG_CONFIG_OSR_DOS_MASK) != 0b011){
                    // create I2C transaction variable to rewrite the OSR
                    uint8_t new_dos = ((i2c_read_buf[0] & ~(AS_REG_CONFIG_OSR_DOS_MASK)) | 0b011);
                    flush_buffers();
                    i2c_write_buf[0] = AS_REG_CONFIG_OSR;
                    i2c_write_buf[1] = new_dos;

                    // perform transaction
                    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);

                    // assert successful transaction and indicate mode change
                    EFM_ASSERT(ret == i2cTransferDone);
            }
    }


    // assert a finished transfer and return
    EFM_ASSERT(ret == i2cTransferDone);
}

void AS7331_GetTemperature()
{
    // read the current OSR value
    I2C_TransferReturn_TypeDef ret;
    flush_buffers();
    i2c_write_buf[0] = AS_REG_CONFIG_OSR;
    ret = AS7331_transaction(I2C_FLAG_WRITE_READ, i2c_write_buf, 1, i2c_read_buf, 1);
    EFM_ASSERT(ret == i2cTransferDone);

    // keep the OSR the same, but force the SS bit to 1 and write value back to OSR
    uint8_t osr_new = (i2c_read_buf[0] | AS_REG_CONFIG_OSR_SS(1));
    i2c_write_buf[1] = osr_new;
    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);
    EFM_ASSERT(ret == i2cTransferDone);

    // create an I2C transaction to read temperature register
    flush_buffers();
    i2c_write_buf[0] = AS_REG_MEAS_TEMP;
    ret = AS7331_transaction(I2C_FLAG_WRITE_READ, i2c_write_buf, 1, i2c_read_buf, 2);
    EFM_ASSERT(ret == i2cTransferDone);

    // with the result buffer, store into local file variable
    uint16_t tmp = (((uint16_t)i2c_read_buf[1]) << 8) | ((uint16_t)i2c_read_buf[0]);
    temperature = (((float)tmp * 0.05) - 66.9);
}

/*********************************************************
 * Global Functions
 ********************************************************/
void as_dev_id(void)
{
    // set the current mode to configuration mode
    AS7331_ChangeMode(AS_CONFIG_MODE_CONFIGURATION);

    // create I2C variables
    I2C_TransferReturn_TypeDef ret;
    flush_buffers();
    i2c_write_buf[0] = AS_REG_CONFIG_AGEN;

    // Wait for sensor to become ready
    sl_sleeptimer_delay_millisecond(80);

    // Check for device presence  and compare device ID
    ret = AS7331_transaction(I2C_FLAG_WRITE_READ, i2c_write_buf, 1, i2c_read_buf, 1);

    // Print Device ID
    printf("%-10s Device ID: 0x%02X\r\n", AS_NAME, i2c_read_buf[0]);

    // Make sure transfer was successful
    EFM_ASSERT(ret == i2cTransferDone);

    // Check the Received Device ID
    EFM_ASSERT(i2c_read_buf[0] == AS_CONFIG_DEVICE_ID);
}

void as_init(void)
{
    // set the current mode to configuration mode
    printf("[%10s] Changing Sensor Mode to Configuration Mode...\r\n", AS_NAME);
    AS7331_ChangeMode(AS_CONFIG_MODE_CONFIGURATION);

    // create I2C parameters
    I2C_TransferReturn_TypeDef ret;
    flush_buffers();

    // OSR - Operational State Register

    // set the OSR
    uint8_t osr = (AS_REG_CONFIG_OSR_SS(0)) | \
                  (AS_REG_CONFIG_OSR_PD(1)) | \
                  (AS_REG_CONFIG_OSR_SW_RES(0)) | \
                  (AS_REG_CONFIG_OSR_DOS(0b010));

    // send an I2C transaction to change OSR
    i2c_write_buf[0] = AS_REG_CONFIG_OSR;
    i2c_write_buf[1] = osr;
    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);
    printf("[%10s] OSR set - new value: 0x%02X\r\n", AS_NAME, osr);
    EFM_ASSERT(ret == i2cTransferDone);

    // CREG1 - Control Register 1

    // set the CREG1 to default values
    uint8_t creg1 = (AS_REG_CONFIG_CREG1_GAIN(CREG1_GAIN_2)) | \
                    (AS_REG_CONFIG_CREG1_TIME(CREG1_TIME_64));

    // send an I2C transaction to change CREG1
    i2c_write_buf[0] = AS_REG_CONFIG_CREG1;
    i2c_write_buf[1] = creg1;
    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);
    printf("[%10s] CREG1 set - new value: 0x%02X\r\n", AS_NAME, creg1);
    EFM_ASSERT(ret == i2cTransferDone);

    // CREG2 - Control Register 2

    // set the CREG2 to default values
    uint8_t creg2 = (AS_REG_CONFIG_CREG2_EN_TM(1)) | \
                    (AS_REG_CONFIG_CREG2_EN_DIV(0));

    // send and I2C transaction to change CREG2
    i2c_write_buf[0] = AS_REG_CONFIG_CREG2;
    i2c_write_buf[1] = creg2;
    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);
    printf("[%10s] CREG2 set - new value: 0x%02X\r\n", AS_NAME, creg2);
    EFM_ASSERT(ret == i2cTransferDone);

    // CREG3 - Control Register 3

    // set the CREG3 to default values
    uint8_t creg3 = (AS_REG_CONFIG_CREG3_MMODE(0b01)) | \
                    (AS_REG_CONFIG_CREG3_SB(0)) | \
                    (AS_REG_CONFIG_CREG3_RDYOD(0)) | \
                    (AS_REG_CONFIG_CREG3_CCLK(0b00));

    // send and I2C transaction to change CREG3
    i2c_write_buf[0] = AS_REG_CONFIG_CREG3;
    i2c_write_buf[1] = creg3;
    ret = AS7331_transaction(I2C_FLAG_WRITE, i2c_write_buf, 2, i2c_read_buf, 1);
    printf("[%10s] CREG3 set - new value: 0x%02X\r\n", AS_NAME, creg3);
    EFM_ASSERT(ret == i2cTransferDone);

    // change to measurement mode once all configuration settings have been made
    printf("[%10s] Changing Sensor Mode to Measurement Mode...\r\n", AS_NAME);
    AS7331_ChangeMode(AS_CONFIG_MODE_MEASUREMENT);
}

void as_process_action(void)
{
    /*********************
     * TEMPERATURE
     *********************/
    AS7331_GetTemperature();
    printf("[%10s] Temperature: %.2f\r\n", AS_NAME, temperature);

    return;
}

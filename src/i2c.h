/*******************************************************************************
 * @file i2c.h
 * @brief I2C communication header file
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  ecen5823-assignment3-PeterBraganza
 * @date:        15-09-2021
 * @author:      Peter Braganza
 * Description:  Contains function prototype for Initializing, writing and
 *               reading from the Si7021 and  I2C disable.
 ******************************************************************************/

#ifndef SRC_I2C_H_
#define SRC_I2C_H_

void i2cInit();
//void i2cWriteTOSi7021_polling();
//void i2cReadTOSi7021_polling();
void i2cWriteTOSi7021_irq();
void i2cReadTOSi7021_irq();

//MPU6050 function declaration
//Who am I gets the slave address of the MPU6050
void i2cWriteReadToWhoAmIMPU6050();

//functions to set PWR to 0x01 and read register
void i2cWritePWRValueToMPU6050();
void i2cWriteReadPWRToMPU6050();

//function to set Sample rate to 4Hz
void i2cWriteSampleRateValueToMPU6050();
void i2cWriteReadSampleRateToMPU6050();

void i2cWriteConfigValueToMPU6050();
void i2cWriteReadConfigToMPU6050();

void i2cWriteReadAccelConfigToMPU6050();
void i2cWriteAccelConfigValueToMPU6050();

void i2cWriteFIFOEnableValueToMPU6050();
void i2cWriteReadFIFOEnableToMPU6050();

void i2cWriteReadAccelXHighToMPU6050();

void i2cWriteInterruptConfigValueToMPU6050();
void i2cWriteReadInterruptConfigToMPU6050();

void i2cWriteReadInterruptStatusToMPU6050();

void i2cWriteUserConfigValueToMPU6050();
void i2cWriteReadUserConfigToMPU6050();

void i2cWriteIntPinConfigValueToMPU6050();
void i2cWriteReadIntPinConfigToMPU6050();

void i2cWriteReadFIFOCountHToMPU6050();
void i2cWriteReadFIFOCountLToMPU6050();

void i2cWriteReadFIFORWToMPU6050();

void i2cWriteReadAccelXYZToMPU6050();

int writeFIFODataToBuffer();
void writeAccelXYZCalibrate();

void display_Reg_values();

uint32_t convertToC();

#endif /* SRC_I2C_H_ */

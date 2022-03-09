/*******************************************************************************
 * @file i2c.c
 * @brief I2C communication functions
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 *
 * @assignment:  ecen5823-assignment3-PeterBraganza
 * @date:        15-09-2021
 * @author:      Peter Braganza
 * Description:  This file contains functions to initialize I2C communication
 *               and read and write function to the Si7021 tempreature sensor
 *               using polling
 ******************************************************************************/

#include "em_i2c.h"
#include "sl_i2cspm.h"
#include "gpio.h"
#include "em_i2c.h"
#include "ble.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"



//made global for IC2 interrupt transfer
I2C_TransferSeq_TypeDef transferSequence;
uint8_t cmd_data = 0xF3;
uint8_t read_data[2];

//variables for MPU6050
uint8_t MPU6050_slave_address = 0x68;
uint8_t ACCEL_X_H = 0x3B;
uint8_t ACCEL_X_L = 0x3C;
uint8_t Config[2] = { 0x1A, 0x00 };
uint8_t Accel_Config[2] = { 0x1C, 0x10 };    //configure accelerometer to +/-8g
uint8_t SampleRate[2] = { 0x19, 0xF9 };      //configure sample rate to F9 for sample rate 1k * 1/(1+249) = 4hz i.e. .25sec
uint8_t PWR_MGMT_1[2]= { 0x6B, 0x01};
uint8_t FIFO_EN[2] = { 0x23 , 0x08 };     //enable accelerometer values(59-64 dec) to be written in FIFO buffer
uint8_t Who_am_I = 0x75;
uint8_t Int_Config[2] = { 0x38, 0x10 };  //enable FIFO overflow interrupt bit
uint8_t Int_Status = 0x3A;
uint8_t User_Config[2] = { 0x6A, 0x40 };  //enable FIFO buffer in user config using bit 6
uint8_t Int_Pin_Config[2] = { 0x37, 0x20 };
uint8_t FIFO_Count_H = 0x72;   //Higher register must be read before reading lower register
uint8_t FIFO_Count_L = 0x73;
uint8_t FIFO_R_W = 0x74;


//store reg values
uint8_t read_X_H;
uint8_t read_X_L;
uint8_t PWR_MGMT_1_data;
uint8_t Who_am_I_data;
uint8_t sample_rate_data;
uint8_t config_data;
uint8_t accel_config_data;
uint8_t FIFO_enable_data;
uint8_t int_config_data;
uint8_t int_status_data;
uint8_t user_config_data;
uint8_t int_pin_config_data;
uint8_t FIFO_count_h_data;
uint8_t FIFO_count_l_data;
uint16_t FIFO_count;
uint8_t FIFO_data[36];
uint8_t FIFO_buffer[6];

int i = 0;
int16_t axccel_x, axccel_y, axccel_z;
int16_t axccel_x_off_set, axccel_y_off_set, axccel_z_off_set;


//Initialize I2C to communicate with the SI7021 temp sensor
void i2cInit()
{
  I2CSPM_Init_TypeDef I2C_Config;
  I2C_Config.port = I2C0;
  I2C_Config.sclPort = I2C_SCL_port;
  I2C_Config.sclPin = I2C_SCL_pin;
  I2C_Config.sdaPort = I2C_SDA_port;
  I2C_Config.sdaPin = I2C_SDA_pin;
  I2C_Config.portLocationScl = I2C_SCL_port_location;
  I2C_Config.portLocationSda = I2C_SDA_port_location;
  I2C_Config.i2cRefFreq = 0;
  I2C_Config.i2cMaxFreq = I2C_FREQ_STANDARD_MAX;
  I2C_Config.i2cClhr = i2cClockHLRStandard;

  I2CSPM_Init(&I2C_Config);

} //i2cInit()

/*
//sends a write cmd to the Si7021 requesting for a temperature reading
void i2cWriteTOSi7021_polling()
{
  //send read rq using I2CSPM_Transfer()
  //TODO make this data global
  I2C_TransferReturn_TypeDef write_status;
  I2C_TransferSeq_TypeDef write_seq;

  uint8_t write_buffer_data = 0xF3;

  write_seq.addr = ( 0x40 << 1 );
  write_seq.flags = I2C_FLAG_WRITE;
  write_seq.buf[0].data = &write_buffer_data;
  write_seq.buf[0].len = sizeof(uint8_t);

  write_status = I2CSPM_Transfer(I2C0, &write_seq);
  if (write_status < 0)
  {
      LOG_ERROR("%d",write_status);
      LOG_ERROR("%d",loggerGetTimestamp());
  }
}

//read the temperature data the data bus and LOG the value
void i2cReadTOSi7021_polling()
{
  //read I2C using I2CSPM
  I2C_TransferSeq_TypeDef read_seq;
  I2C_TransferReturn_TypeDef read_status;

  read_seq.addr = ( 0x40 << 1 );
  read_seq.flags = I2C_FLAG_READ;
  read_seq.buf[0].data = &read_buffer_data[0];
  read_seq.buf[0].len = sizeof(uint16_t);

  read_status = I2CSPM_Transfer(I2C0, &read_seq);
  if (read_status < 0)
  {
      LOG_ERROR("%d",read_status);
  }
  uint16_t temp_value = 256*read_buffer_data[0] + read_buffer_data[1];
  uint32_t C = ((175.72 * temp_value) / 65536 ) - 46.85;

  LOG_INFO("TEMP is %d", C);
}
*/




void i2cWriteTOSi7021_irq()
{
  //Transfer structure
  //I2C_TransferSeq_TypeDef write_seq;
  I2C_TransferReturn_TypeDef transferStatus;

  i2cInit();

  transferSequence.addr = ( 0x40 << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &cmd_data;
  transferSequence.buf[0].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }


  //need to disable NVIC once interrupt write_status == transferDone

} //i2cWriteTOSi7021_irq()

void i2cReadTOSi7021_irq()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( 0x40 << 1 );
  transferSequence.flags = I2C_FLAG_READ;
  transferSequence.buf[0].data = &read_data[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }


} //i2cReadToSi7021_irq()


//******************************************************************************
//All MPU6050 I2C functions
//******************************************************************************
void i2cWriteReadToWhoAmIMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &Who_am_I;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &Who_am_I_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }

} //i2cWriteToWhoAmIMPU6050()



void i2cWritePWRValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &PWR_MGMT_1[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }

}


void i2cWriteReadPWRToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &PWR_MGMT_1[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &PWR_MGMT_1_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }

}


void i2cWriteSampleRateValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &SampleRate[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }

}

void i2cWriteReadSampleRateToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &SampleRate[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &sample_rate_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}


void i2cWriteConfigValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &Config[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadConfigToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &Config[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &config_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}


void i2cWriteAccelConfigValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &Accel_Config[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadAccelConfigToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &Accel_Config[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &accel_config_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteFIFOEnableValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &FIFO_EN[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadFIFOEnableToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &FIFO_EN[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &FIFO_enable_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadAccelXHighToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &ACCEL_X_H;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &read_X_H;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteInterruptConfigValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &Int_Config[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadInterruptConfigToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &Int_Config[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &int_config_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteUserConfigValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &User_Config[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadUserConfigToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &User_Config[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &user_config_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadInterruptStatusToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &Int_Status;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &int_status_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteIntPinConfigValueToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE;
  transferSequence.buf[0].data = &Int_Pin_Config[0];
  transferSequence.buf[0].len = sizeof(uint16_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadIntPinConfigToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &Int_Pin_Config[0];
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &int_pin_config_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadFIFOCountHToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &FIFO_Count_H;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &FIFO_count_h_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadFIFOCountLToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &FIFO_Count_L;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &FIFO_count_l_data;
  transferSequence.buf[1].len = sizeof(uint8_t);

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadFIFORWToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &FIFO_R_W;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &FIFO_data[0];
  transferSequence.buf[1].len = sizeof(uint8_t) * 36;

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

void i2cWriteReadAccelXYZToMPU6050()
{
  //Transfer structure
  I2C_TransferReturn_TypeDef transferStatus;

  transferSequence.addr = ( MPU6050_slave_address << 1 );
  transferSequence.flags = I2C_FLAG_WRITE_READ;
  transferSequence.buf[0].data = &ACCEL_X_H;
  transferSequence.buf[0].len = sizeof(uint8_t);
  transferSequence.buf[1].data = &FIFO_buffer[0];
  transferSequence.buf[1].len = 6;

  NVIC_EnableIRQ(I2C0_IRQn);
  transferStatus = I2C_TransferInit(I2C0, &transferSequence);
  if (transferStatus < 0 )
  {
    LOG_ERROR("Transfer Error %d", transferStatus);
  }
}

int writeFIFODataToBuffer()
{
   ble_data_struct_t *bleDataPtr = getBleDataPtr();
//  for ( i = 0 ; i < 36 ; i++ )
//  {
//    //LOG_INFO("FIFO[%d]: %d\r", i, FIFO_data[i]);
//  }
//
//  for (int j = 0 ; j < 6 ; j++ )
//  {
//    axccel_x[j] = (int16_t)(( FIFO_buffer[0+j] << 8 ) | FIFO_buffer[1+j]);
//    axccel_y[j] = (int16_t)(( FIFO_buffer[2+j] << 8 ) | FIFO_buffer[3+j]);
//    axccel_z[j] = (int16_t)(( FIFO_buffer[4+j] << 8 ) | FIFO_buffer[5+j]);
//  }

//  LOG_INFO("axccel_x call: %f\r", (float)axccel_x_off_set/4096);
//  LOG_INFO("axccel_y call: %f\r", (float)axccel_y_off_set/4096);
//  LOG_INFO("axccel_z call: %f\r", (float)axccel_z_off_set/4096);

  axccel_x = (int16_t)(( FIFO_buffer[0] << 8 ) | FIFO_buffer[1]) ;
  axccel_y = (int16_t)(( FIFO_buffer[2] << 8 ) | FIFO_buffer[3]) ;
  axccel_z = (int16_t)(( FIFO_buffer[4] << 8 ) | FIFO_buffer[5]) ;
//  LOG_INFO("axccel_x[0]: %f\r", (float)axccel_x/4096);
//  LOG_INFO("axccel_y[0]: %f\r", (float)axccel_y/4096);
//  LOG_INFO("axccel_z[0]: %f\r", (float)axccel_z/4096);

  if( ((float)axccel_x/4096 > 5) || ((float)axccel_y/4096 > 5) || ((float)axccel_z/4096 > 5))
  {
     //LOG_INFO("jerk\r");
     bleDataPtr->motion_violation_count += 1;
     gpioLed0SetOn();
     bleDataPtr->count = 12;

  }
  if( ((float)axccel_x/4096 < -5) || ((float)axccel_y/4096 < -5) || ((float)axccel_z/4096 < -5))
  {
     //LOG_INFO("jerk bellow\r");
     bleDataPtr->motion_violation_count += 1;
     gpioLed0SetOn();
     bleDataPtr->count = 12;
  }
  LOG_INFO("Motion violation %d\r", bleDataPtr->motion_violation_count);

  return bleDataPtr->motion_violation_count;

}

void writeAccelXYZCalibrate()
{

  axccel_x_off_set = (int16_t)(( FIFO_buffer[0] << 8 ) | FIFO_buffer[1]);
  axccel_y_off_set = (int16_t)(( FIFO_buffer[2] << 8 ) | FIFO_buffer[3]);
  axccel_z_off_set = (int16_t)(( FIFO_buffer[4] << 8 ) | FIFO_buffer[5]);
//  LOG_INFO("axccel_x call: %f\r", (float)axccel_x_off_set/4096);
//  LOG_INFO("axccel_y call: %f\r", (float)axccel_y_off_set/4096);
//  LOG_INFO("axccel_z call: %f\r", (float)axccel_z_off_set/4096);

}




uint32_t convertToC()
{
  uint16_t temp_value = 256*read_data[0] + read_data[1];
  uint32_t C = ((175.72 * temp_value) / 65536 ) - 46.85;

  //LOG_INFO("TEMP is %d", C);

  return C;

} //convertToC()


void display_Reg_values()
{
  FIFO_count = ( FIFO_count_h_data << 8 ) | FIFO_count_l_data;
  //FIFO_count = FIFO_count_h_data;
//  LOG_INFO("Who am I: 0x%02x\r", Who_am_I_data);
//  LOG_INFO("SampleRatebuf: 0x%02x\r",sample_rate_data);
//  LOG_INFO("Power mode reg: 0x%02x\r",PWR_MGMT_1_data);
//  LOG_INFO("Config reg: 0x%02x\r",config_data);
//  LOG_INFO("Accel Config reg: 0x%02x\r", accel_config_data);
//  LOG_INFO("FIFO Enable reg: 0x%02x\r", FIFO_enable_data);
//  LOG_INFO("Accel X H: 0x%02x\r", read_X_H);
//  LOG_INFO("Interrupt Config: 0x%02x\r", int_config_data);
//  LOG_INFO("Interrupt Status: 0x%02x\r", int_status_data);
//  LOG_INFO("User Config: 0x%02x\r", user_config_data);
//  LOG_INFO("Int pin config: 0x%02x\r", int_pin_config_data);
 // LOG_INFO("FIFO count: %d\r", FIFO_count);
 // LOG_INFO("FIFO data: %d\r", FIFO_data);

}



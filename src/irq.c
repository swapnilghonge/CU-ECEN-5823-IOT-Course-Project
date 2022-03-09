/*******************************************************************************
 * @file irq.c
 * @brief Interrupt handler code
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  Course Project
 * @date:        10-09-2021
 * @author:      Peter Braganza and Swapnil Ghonge
 * Description:  Contains function to handle LETIMER IRQ which checks if an UF
 *               Interrupt has occurred and sets the UF event in the scheduler
 ******************************************************************************/


#include "timers.h"
#include "gpio.h"
#include "irq.h"
#include "ble.h"
#include "lcd.h"

#include "sl_i2cspm.h"
#include "em_i2c.h"
#include "i2c.h"
#include "scheduler.h"

#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

uint32_t time = 0;
//bool PB0Pressed = false;

//checks if UF interrupt has occurred and sets the corresponding event in the
//scheduler
void LETIMER0_IRQHandler()
{
  //determine interrupts pending
  uint32_t flags = LETIMER_IntGetEnabled(LETIMER0);

  //clear flags
  //LETIMER_IntClear(LETIMER0, LETIMER0->IEN); // DOS: ??? you have the IF bits in variable flags with their coresponding IEN bits set, why not write that back to clear the IF bits?
  LETIMER_IntClear(LETIMER0, flags);

  if(flags & (1<<2))     //checks if UF is set
  {
    time = time + 3000;
    schedularSetLetime0UFEvent();
  }

  // DOS both IF could be set at the time!!!, no else clause!!!!
  //else if (flags & (1<<1))    //checks if COMP1 is set
  if (flags & (1<<1))    //checks if COMP1 is set
  {
    schedularSetCOMP1Event();
    //LOG_ERROR("COMP1 event set");
  }
} //LETIMER0_IRQHandler()


//Performs the Interrupt service routine when a temperature reading needs to be
//taken. It performs the load power management steps and calls required I2C
//functions to perform the serial communication
void Read_Temp_From_Si7021()
{
  i2cInit();
  gpioSensorEnable();
  timerWaitUs_irq(80000);   //80ms
  i2cInit();
  i2cWriteTOSi7021_irq();
  timerWaitUs_polled(10800);   //10.8ms
  i2cReadTOSi7021_irq();
  gpioSensorDisable();
  i2cDisable();
  sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

} //Read_Temp_From_Si7021()


//function returns time in milliseconds since the board is powered on up to
//runtime
uint32_t letimerMilliseconds()
{

  uint32_t count = LETIMER_CounterGet(LETIMER0);

  //CORE_DECLARE_IRQ_STATE;

  //CORE_ENTER_CRITICAL();
  //uint32_t time_to_add = (ACTUAL_LFXO_CLK_FREQ *(COMP0_LFXO -count))/1000;

  uint32_t time_to_add = (COMP0_LFXO -count)*4000/32768;
  //time = time + time_to_add;
  // CORE_EXIT_CRITICAL();

  return time + time_to_add;

} //letimerMilliseconds()

void I2C0_IRQHandler()
{
  I2C_TransferReturn_TypeDef transferStatus;

  transferStatus = I2C_Transfer(I2C0);

  if (transferStatus == i2cTransferDone)
  {
    schedularSetI2CTransferDoneEvent();
    NVIC_DisableIRQ(I2C0_IRQn);
  }
  if (transferStatus < 0)
  {
    LOG_ERROR("Transfer Error %d\r", transferStatus);
  }

} //I2C_IRQHandler()



void GPIO_EVEN_IRQHandler()
{

  uint32_t flags = GPIO_IntGet();
  GPIO_IntClear(flags);

  //LOG_INFO("Flag value %d:\r", flags);
  //check is PB0(Pin6) interrupt occurs
  if (flags & (1<<6))
  {
    schedularSetPB0Event();

  }

  if (flags & (1<<12))
  {
    schedularSetEchoEvent();
    //LOG_INFO("Flag value %d:\r", flags);
  }

} //GPIO_EVEN_IRQHandler()

void GPIO_ODD_IRQHandler()
{

  uint32_t flags = GPIO_IntGet();
  GPIO_IntClear(flags);
  //LOG_INFO("ODD interrupt occurred Flag : %d\r", flags);

  //check is PB1(Pin7) interrupt occurs
  if (flags & (1<<7))
  {
    schedularSetPB1Event();
  }


} //GPIO_ODD_IRQHandler()




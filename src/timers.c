/*******************************************************************************
 * @file timers.c
 * @brief Timer code for LETIMER0 and delays
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  Course Project
 * @date:        9-09-2021
 * @author:      Peter Braganza and Swapnil Ghonge
 * Description:  Contains functions to Initialize LETIMER0. Also includes a
 *               delay function which will wait for a required amount of time
 *               as specified in the function argument
 ******************************************************************************/

#include "timers.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

//LETIMER initializing function depending on the Lowest energy mode defined
void timerInit(){

  LETIMER_Init_TypeDef letimer0Init = LETIMER_INIT_DEFAULT;

  letimer0Init.comp0Top = true;
  letimer0Init.repMode = letimerRepeatFree;

  if (LOWEST_ENERGY_MODE == 3)
  {
    letimer0Init.topValue = COMP0_ULF;
  }
  else
  {
    letimer0Init.topValue = COMP0_LFXO;
  }
  LETIMER_Init(LETIMER0, &letimer0Init);

  LETIMER_Enable(LETIMER0, true);

  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_UF);
}

//function will cause a delay of wait_us micro seconds using the LETIMER0
//to compare the current value and the calculated value count_to_wait
//when the TIMER reached count_to_wait the delay has occured for wait_us micro s
void timerWaitUs_polled(uint32_t wait_us)
{

  uint32_t count = LETIMER_CounterGet(LETIMER0);
  uint32_t count_to_wait = count - (wait_us * ACTUAL_ULFRCO_CLK_FREQ) / 1000000;

  //check if current value is less than required count value
  //if so then only count to the maximum possible value and LOG and error
  if (count_to_wait > count)
  {
    LOG_INFO("Delay value too high");
    count_to_wait = count;
  }
  while(count >= count_to_wait)
  {
    count = LETIMER_CounterGet(LETIMER0);
  }
}

void timerWaitUs_irq(uint32_t wait_us)
{
  uint32_t count = LETIMER_CounterGet(LETIMER0);
  uint32_t count_to_wait = count - (wait_us * ACTUAL_LFXO_CLK_FREQ) / 1000000;

  //LOG_INFO("count :%d",count);
  //LOG_INFO("wait : %d",count_to_wait);

  //range checking
  if (count_to_wait > count)
  {
    LOG_INFO("Delay value too high");
    count_to_wait = count;
  }

  LETIMER_CompareSet(LETIMER0, 1, count_to_wait);
  LETIMER_IntClear(LETIMER0, LETIMER_IF_COMP1);
  LETIMER_IntEnable(LETIMER0, LETIMER_IEN_COMP1);
  //TODO professors fix

}




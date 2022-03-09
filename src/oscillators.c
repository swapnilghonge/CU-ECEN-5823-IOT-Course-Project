/*******************************************************************************
 * @file oscillators.c
 * @brief Oscillators Configuration functions
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
 * @author:      Swapnil Ghonge
 * Description:  This file contains the function to configure the oscillator and
 *               clocks depending on the Lowest Energy Mode selected.
 ******************************************************************************/

 
#include "oscillators.h"

//Configures the oscillator and enables clocks to the LETIMER0 based on
//the LOWEST_ENERGY_MODE
void oscillatorsInit()
{
  if (LOWEST_ENERGY_MODE == 3)
  {
    CMU_OscillatorEnable(cmuOsc_ULFRCO,true,true);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_ULFRCO);
    CMU_ClockEnable(cmuClock_LFA, true);
    CMU_ClockEnable(cmuClock_LETIMER0,true);
  }
  else
  {
    CMU_OscillatorEnable(cmuOsc_LFXO,true,true);
    CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
    CMU_ClockEnable(cmuClock_LFA, true);
    CMU_ClockDivSet(cmuClock_LETIMER0, cmuClkDiv_4);
    CMU_ClockEnable(cmuClock_LETIMER0,true);
  }
} //oscillatorsInit()


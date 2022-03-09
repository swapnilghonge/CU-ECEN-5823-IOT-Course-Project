/*******************************************************************************
 * @file timers.h
 * @brief LETIMER0 configuration value definitions
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
 * Description:  This file contains the configuration values for the LETIMER0
 ******************************************************************************/

#ifndef SRC_TIMERS_H_
#define SRC_TIMERS_H_

#include "em_letimer.h"
#include "app.h"

//constants for when ULFRCO is used when LOWEST_ENEGRY_MODE is EM3
#define ACTUAL_ULFRCO_CLK_FREQ 1000
#define COMP0_ULF (LETIMER_PERIOD_MS * ACTUAL_ULFRCO_CLK_FREQ) / 1000
#define COMP1_ULF ((LETIMER_PERIOD_MS-LETIMER_ON_TIME_MS)*ACTUAL_ULFRCO_CLK_FREQ)/1000

//constants for when LFXO is used when LOWEST_ENEGRY_MODE is EM0/1/2
#define PRESCALER_VALUE 4
#define ACTUAL_LFXO_CLK_FREQ 32768/PRESCALER_VALUE
#define COMP0_LFXO (LETIMER_PERIOD_MS * ACTUAL_LFXO_CLK_FREQ) / 1000
#define COMP1_LFXO ((LETIMER_PERIOD_MS-LETIMER_ON_TIME_MS)*ACTUAL_LFXO_CLK_FREQ)/1000

//function prototypes
void timerInit();
void timerWaitUs_polled(uint32_t wait_us);
void timerWaitUs_irq(uint32_t wait_us);

#endif /* SRC_TIMERS_H_ */

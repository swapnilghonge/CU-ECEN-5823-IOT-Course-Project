/*******************************************************************************
 * @file irq.h
 * @brief Interrupt handler header file
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
 * Description:  Function prototypes for the Interrupt handler
 ******************************************************************************/

#ifndef SRC_IRQ_H_
#define SRC_IRQ_H_
#include <stdint.h>

void LETIMER0_IRQHandler();
void Read_Temp_From_Si7021();
uint32_t letimerMilliseconds();
void I2C0_IRQHandler();
void GPIO_EVEN_IRQHandler();
void GPIO_ODD_IRQHandler();

#endif /* SRC_IRQ_H_ */

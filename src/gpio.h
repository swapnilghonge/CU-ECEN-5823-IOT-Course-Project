/*******************************************************************************
 * @file gpio.h
 * @brief GPIO pin definition file
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  Course Project
 * @date:        1-11-2021
 * @author:      Swapnil Ghonge
 * Description:  Contains gpio pins initialization code and gpio set reset code
 ******************************************************************************/

#ifndef SRC_GPIO_H_
#define SRC_GPIO_H_
#include <stdbool.h>
#include "em_gpio.h"
#include <string.h>


// Student Edit: Define these, 0's are placeholder values.
// See the radio board user guide at https://www.silabs.com/documents/login/user-guides/ug279-brd4104a-user-guide.pdf
// and GPIO documentation at https://siliconlabs.github.io/Gecko_SDK_Doc/efm32g/html/group__GPIO.html
// to determine the correct values for these.

#define	LED0_port  gpioPortF
#define LED0_pin   4
#define LED1_port  gpioPortF
#define LED1_pin   5
#define I2C_SCL_port gpioPortC
#define I2C_SCL_pin 10
#define I2C_SDA_port gpioPortC
#define I2C_SDA_pin 11
#define I2C_SCL_port_location 14
#define I2C_SDA_port_location 16
#define SENSOR_ENABLE_port gpioPortD
#define SENSOR_ENABLE_pin 15
#define EXTCOMIN_port gpioPortD
#define EXTCOMIN_pin 13
#define PB0_port  gpioPortF
#define PB0_pin   6
#define PB1_port  gpioPortF
#define PB1_pin   7

#define trigger_port gpioPortD   //ultrasonic sensor trigger port //P9 pin on board External P10
#define trigger_pin 11
#define echo_port gpioPortD      //ultrasonic sensor echo port // P11 pin on board External P12
#define echo_pin 12

// Function prototypes
void gpioInit();
void gpioLed0SetOn();
void gpioLed0SetOff();
void gpioLed1SetOn();
void gpioLed1SetOff();
void gpioSensorEnable();
void gpioSensorDisable();
void i2cDisable();
void gpioSetDisplayExtcomin(bool last_extcomin_state);
void ultrasonic_sensor_trigger_enable();
void ultrasonic_sensor_trigger_disable();


#endif /* SRC_GPIO_H_ */

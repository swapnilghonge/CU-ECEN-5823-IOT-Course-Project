/*******************************************************************************
 * @file gpio.c
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




#include "gpio.h"


// Set GPIO drive strengths and modes of operation
void gpioInit()
{

  // Student Edit:
  //LED0
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(LED0_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED0_port, LED0_pin, gpioModePushPull, false);

  //LED1
	GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthStrongAlternateStrong);
	//GPIO_DriveStrengthSet(LED1_port, gpioDriveStrengthWeakAlternateWeak);
	GPIO_PinModeSet(LED1_port, LED1_pin, gpioModePushPull, false);

  //SENDOR Enable
	GPIO_DriveStrengthSet(SENSOR_ENABLE_port, gpioDriveStrengthStrongAlternateStrong);
	GPIO_PinModeSet(SENSOR_ENABLE_port, SENSOR_ENABLE_pin, gpioModePushPull, false);

  //GPIO_DriveStrengthSet(EXTCOMIN_port, gpioDriveStrengthStrongAlternateStrong);
  GPIO_PinModeSet(EXTCOMIN_port, EXTCOMIN_pin, gpioModePushPull, false);


  //GPIO PB0 configured for interrupt on rising edge and falling edge
  GPIO_PinModeSet(PB0_port, PB0_pin, gpioModeInputPullFilter , true);

  GPIO_ExtIntConfig (PB0_port, PB0_pin, PB0_pin, true, true, true);

  //GPIO PB1 configured for interrupt on rising edge
  GPIO_PinModeSet(PB1_port, PB1_pin, gpioModeInputPullFilter , true);

  GPIO_ExtIntConfig (PB1_port, PB1_pin, PB1_pin, true, false, true);

  //GPIO config for trigger and echo
  GPIO_DriveStrengthSet(trigger_port, gpioDriveStrengthStrongAlternateStrong);
  GPIO_PinModeSet(trigger_port , trigger_pin, gpioModePushPull, false);

  GPIO_PinModeSet(echo_port , echo_pin, gpioModeInputPull, true);
  GPIO_ExtIntConfig (echo_port, echo_pin, echo_pin, true, true, true);

  NVIC_EnableIRQ(GPIO_EVEN_IRQn);
  NVIC_EnableIRQ(GPIO_ODD_IRQn);

} // gpioInit()


void gpioLed0SetOn()
{
	GPIO_PinOutSet(LED0_port,LED0_pin);
}


void gpioLed0SetOff()
{
	GPIO_PinOutClear(LED0_port,LED0_pin);
}


void gpioLed1SetOn()
{
	GPIO_PinOutSet(LED1_port,LED1_pin);
}


void gpioLed1SetOff()
{
	GPIO_PinOutClear(LED1_port,LED1_pin);
}


void gpioSensorEnable()
{

  GPIO_PinOutSet(SENSOR_ENABLE_port,SENSOR_ENABLE_pin);
}

void gpioSensorDisable()
{
  GPIO_PinOutClear(SENSOR_ENABLE_port,SENSOR_ENABLE_pin);
}

//Disable the I2C GPIO pins
void i2cDisable()
{
  GPIO_PinModeSet(gpioPortC, 10, gpioModeDisabled, 1);  // Disabling the SDA port
  GPIO_PinModeSet(gpioPortC, 11, gpioModeDisabled, 1);  // Disabling the SDL port

} //i2cDisable()

void gpioSetDisplayExtcomin(bool last_extcomin_state)
{
  if (last_extcomin_state == true )
  {
    GPIO_PinOutSet(EXTCOMIN_port, EXTCOMIN_pin);
  }
  if (last_extcomin_state == false)
  {
    GPIO_PinOutClear(EXTCOMIN_port, EXTCOMIN_pin);
  }

} //gpioSetDisplayExtcomin(bool last_extcomin_state)


//Ultrasonic trigger enable function
void ultrasonic_sensor_trigger_enable()
{
  GPIO_PinOutSet(trigger_port, trigger_pin);

}

void ultrasonic_sensor_trigger_disable()
{
  GPIO_PinOutClear(trigger_port , trigger_pin);

}



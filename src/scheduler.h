/*******************************************************************************
 * @file schedulars.h
 * @brief Timer code for LETIMER0 and delays
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
 * Description:  Scheduler and temperature state machine function prototypes and
 *               event and state structure definitions
 ******************************************************************************/

#ifndef SRC_SCHEDULER_H_
#define SRC_SCHEDULER_H_

#include <stdint.h>
#include "sl_bt_api.h"

//event list
typedef enum {
  evtNoEvent =0 ,
  evtLETIMER0_UF = 1,
  evtCOMP1 = 2,
  evtI2CTransferDone = 3,
  evtPB0Pressed = 4,
  evtPB1Pressed = 5,
  evtEcho = 6,
  evtStartUltrasonic = 7,
  evtSoftTimer = 8

} events_t;

//State list for temperature state machine
typedef enum {
  stateIdle,
  state1,
  state2,
  state3,
  state4,

} State_Temperature_t;

//State list for Discovery state machine
typedef enum {
  scanning,
  discoverTempService,
  discoverButtonStateService,
  discoverTempCharacteristic,
  discoverButtonStateCharacteristic,
  EnableButtonStateIndication,
  EnableTempIndication,
  waitForTempReading

} State_Discovery_t;


//State list for getting MPU6050 Accelerometers readings
typedef enum {
  state_Idle,
  state_1,
  state_2,
  state_3,
  state_4,
  state_5,
  state_6,
  state_7,
  state_8,
  state_9,
  state_10,
  state_11,
  state_12,
  state_13,
  state_14,
  state_15,
  state_16,
  state_17,
  state_18,
  state_19,
  state_20,
  state_21,
  state_22,
  final_state

} State_MPU6050_t;

typedef enum {
  start,
  enable_trigger,
  trigger_state,
  echo_state,
  calculate_distance,


} State_Ultrasonic_t;

typedef enum {

  Idle,
  something,
  connection_open,
  send_motion_count,
  send_distance_count,

} State_Server_t;


//set and reset scheduler event function prototypes
void schedularSetLetime0UFEvent();
void scedulatRestLetimer0UFEvent();
void schedularSetCOMP1Event();
void scedulatResetCOMP1Event();
void schedularSetI2CTransferDoneEvent();
void scedulatResetI2CTransferDoneEvent();
void schedularSetPB0Event();
void schedularSetPB1Event();
void schedularSetEchoEvent();
void schedularSetStartUltrasonicEvent();
void schedularSetSoftTimerEvent();


int32_t gattFloat32ToInt(const uint8_t *value_start_little_endian);

uint32_t getNextEvent();

//temperature state machine
//void temperature_state_machine(uint32_t event);
void temperature_state_machine (sl_bt_msg_t *evt);

//discovery state machine
void discovery_state_machine(sl_bt_msg_t *evt);

//MPU6050 state machine
void MPU6050_state_machine(sl_bt_msg_t *evt);

//HC-SR04 Ultrasonic state machine
void ultrasonic_state_machine(sl_bt_msg_t* evt);

void server_state_machine(sl_bt_msg_t*evt);

#endif /* SRC_SCHEDULER_H_ */

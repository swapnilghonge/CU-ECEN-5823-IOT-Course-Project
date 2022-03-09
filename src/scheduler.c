/*******************************************************************************
 * @file schedulars.c
 * @brief Scheduler and state machine implementation
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  Course Project
 * @date:        15-09-2021
 * @author:      Peter Braganza
 * Description:  Scheduler function definitions and Temperature state machine
 *               and Discovery State machine are defined
 ******************************************************************************/

#include "scheduler.h"
#include "em_core.h"
#include "i2c.h"
#include "gpio.h"
#include "timers.h"
#include "app.h"
#include "ble.h"
#include "lcd.h"
#include <math.h>


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"


events_t events = evtNoEvent;

uint32_t echo_start_time, echo_stop_time;
uint32_t echo_time;

float distance;
bool server_connection = false;


//Set and reset events
void schedularSetLetime0UFEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  //events |= evtLETIMER0_UF;
  sl_bt_external_signal(evtLETIMER0_UF);
  CORE_EXIT_CRITICAL();

} //schedularSetLetime0UFEvent()

void schedularResetLetime0UFEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  events &= ~(evtLETIMER0_UF);
  CORE_EXIT_CRITICAL();

} //schedularResetLetime0UFEvent()

void schedularSetCOMP1Event()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  //events |= evtCOMP1;
  sl_bt_external_signal(evtCOMP1);
  CORE_EXIT_CRITICAL();

} //schedularSetCOMP1Event()

void scedulatResetCOMP1Event()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  events &= ~(evtCOMP1);
  CORE_EXIT_CRITICAL();

} //scedulatResetCOMP1Event()

void schedularSetI2CTransferDoneEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  //events |= evtI2CTransferDone;
  sl_bt_external_signal(evtI2CTransferDone);
  CORE_EXIT_CRITICAL();

} //schedularSetI2CTransferDoneEvent()

void scedulatResetI2CTransferDoneEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  events &= ~(evtI2CTransferDone);
  CORE_EXIT_CRITICAL();

} //scedulatResetI2CTransferDoneEvent()

void schedularSetPB0Event()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(evtPB0Pressed);
  CORE_EXIT_CRITICAL();
}

void schedularSetPB1Event()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(evtPB1Pressed);
  CORE_EXIT_CRITICAL();
}

void schedularSetEchoEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(evtEcho);
  CORE_EXIT_CRITICAL();
}

void schedularSetStartUltrasonicEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(evtStartUltrasonic);
  CORE_EXIT_CRITICAL();
}

void schedularSetSoftTimerEvent()
{
  CORE_DECLARE_IRQ_STATE;

  CORE_ENTER_CRITICAL();
  sl_bt_external_signal(evtSoftTimer);
  CORE_EXIT_CRITICAL();
}



// Original code from Dan Walkes. I (Sluiter) fixed a sign extension bug with the mantissa.
// convert IEEE-11073 32-bit float to integer
int32_t gattFloat32ToInt(const uint8_t *value_start_little_endian)
{
  uint8_t signByte = 0;
  int32_t mantissa;
  // data format pointed at by value_start_little_endian is:
  // [0] = contains the flags byte
  // [3][2][1] = mantissa (24-bit 2’s complement)
  // [4] = exponent (8-bit 2’s complement)
  int8_t exponent = (int8_t)value_start_little_endian[4];

  // sign extend the mantissa value if the mantissa is negative
  if (value_start_little_endian[3] & 0x80)
  {
      signByte = 0xFF;          // msb of [3] is the sign of the mantissa
  }
  mantissa = (int32_t) (value_start_little_endian[1] << 0) |
  (value_start_little_endian[2] << 8) |
  (value_start_little_endian[3] << 16) |
  (signByte << 24) ;
  // value = 10^exponent * mantissa, pow() returns a double type

  return (int32_t) (pow(10, exponent) * mantissa);

} // gattFloat32ToInt


//based on events flag this function will return the next event
uint32_t getNextEvent()
{
  uint32_t theEvent;

  //LOG_ERROR("events: %d",events);
  if(events & evtLETIMER0_UF)
  {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    theEvent = evtLETIMER0_UF;
    CORE_EXIT_CRITICAL();
    schedularResetLetime0UFEvent();
  }
  else if(events & evtCOMP1)
  {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    theEvent = evtCOMP1;
    CORE_EXIT_CRITICAL();
    scedulatResetCOMP1Event();
  }
  else if(events & evtI2CTransferDone)
  {
    CORE_DECLARE_IRQ_STATE;

    CORE_ENTER_CRITICAL();
    theEvent = evtI2CTransferDone;
    CORE_EXIT_CRITICAL();
    scedulatResetI2CTransferDoneEvent();
  }

  return theEvent;

} //getNextEvent()



void temperature_state_machine (sl_bt_msg_t *evt)
{

  State_Temperature_t currentState;
  static State_Temperature_t nextState = stateIdle;
  currentState = nextState;

  uint8_t htm_temperature_buffer[5];        // Stores the temperature data in the Health Thermometer (HTM) format.
                                            // format of the buffer is: flags_byte + 4-bytes of IEEE-11073 32-bit float
  uint8_t *p = htm_temperature_buffer;      // Pointer to HTM temperature buffer needed for converting values to bitstream.
  uint32_t htm_temperature_flt;             // Stores the temperature data read from the sensor in the IEEE-11073 32-bit float format
  uint8_t flags = 0x00;                     // HTM flags set as 0 for Celsius, no time stamp and no temperature type.

  ble_data_struct_t *bleDataPtr = getBleDataPtr();
  queue_t *circularBufferPtr = getQueuePtr();
  sl_status_t sc;

  //check if connection is open and indications are set. Then only run the state machine
  if (bleDataPtr->temperatureIndicationSet == true  && bleDataPtr->connectionOpen == true )
  {

    //check if an external signal has occurred
    if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
    {
      switch (currentState)
      {
        case stateIdle:
          nextState = stateIdle;
          if ( evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF )
          {
            //gpioSensorEnable();
            timerWaitUs_irq(80000);   //80ms

            nextState = state1;
          }
          break;
        case state1:
          nextState = state1;
          if ( evt->data.evt_system_external_signal.extsignals == evtCOMP1 )
          {
            LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
            //DOS you already added this on line 93 in app.c ???? sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM2);
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
            i2cWriteTOSi7021_irq();

            nextState = state2;
          }
          break;
        case state2:
          nextState = state2;
          if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
          {
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
            timerWaitUs_irq(10800);   //10.8ms

            nextState = state3;
          }
          break;
        case state3:
          nextState = state3;
          if ( evt->data.evt_system_external_signal.extsignals == evtCOMP1 )
          {
            LETIMER_IntDisable(LETIMER0, LETIMER_IEN_COMP1);
            sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
            i2cReadTOSi7021_irq();

            nextState = state4;
          }
          break;
        case state4:
          nextState = state4;
          if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
          {
            sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
            uint32_t temp_in_C = convertToC();
            //gpioSensorDisable();
            i2cDisable();

                                                                         // "bitstream" refers to the order of bytes and bits sent. byte[0] is sent first, followed by byte[1]...
            UINT8_TO_BITSTREAM(p, flags);                                // put the flags byte in first, "convert" is a strong word, it places the byte into the buffer
                                                                         // Convert sensor data to IEEE-11073 32-bit floating point format.
            htm_temperature_flt = UINT32_TO_FLOAT(temp_in_C*1000, -3);
                                                                         // Convert temperature to bitstream and place it in the htm_temperature_buffer
            UINT32_TO_BITSTREAM(p, htm_temperature_flt);

            // Write our local GATT DB
            sc = sl_bt_gatt_server_write_attribute_value(
            gattdb_temperature_measurement, // handle from gatt_db.h
            0, // offset
            5, // length
            p // pointer to buffer where data is
            );

            if (sc != SL_STATUS_OK) {
              LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
            }

            //send indications if indication is set and connection is open
            //Could remove as state machine only runs when this condition is true
            if (bleDataPtr->temperatureIndicationSet == true  && bleDataPtr->connectionOpen == true )
            {
              if (bleDataPtr->inFlight == false)
              {
                sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionHandle,gattdb_temperature_measurement,
                                                         sizeof(htm_temperature_buffer),
                                                         &htm_temperature_buffer[0]);
                if (sc != SL_STATUS_OK)
                {
                  LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
                }
                bleDataPtr->inFlight = true;
              }
              else
              {
                indication_queue_t i;
                i.charHandle = gattdb_temperature_measurement;
                i.bufferLength =  sizeof(htm_temperature_buffer);
                i.buffer[0] = htm_temperature_buffer[0];
                i.buffer[1] = htm_temperature_buffer[1];
                i.buffer[2] = htm_temperature_buffer[2];
                i.buffer[3] = htm_temperature_buffer[3];
                i.buffer[4] = htm_temperature_buffer[4];
                enqueue(circularBufferPtr ,i);
              }

              displayPrintf(DISPLAY_ROW_TEMPVALUE, "Temp = %d", temp_in_C);

            }

            nextState = stateIdle;
          }
          break;
        default:
          break;

      } //switch end

    } //if end

  } //if end

} //temperature_state_machine (sl_bt_msg_t *evt)


//The discovery state machine sequences through discovery states using events
//to drive the state machine. Further clarification is given in Client Command
//table in the questions folder
void discovery_state_machine(sl_bt_msg_t *evt)
{
  State_Discovery_t currentState;
  static State_Discovery_t nextState = scanning;
  currentState = nextState;

  ble_data_struct_t *bleDataPtr = getBleDataPtr();
  sl_status_t sc;

  switch (currentState)
  {
    case scanning:
      nextState = scanning;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id )
      {
        sc = sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr->connectionHandle,
                                                     sizeof(distance_violation_state_service),
                                                     (const uint8_t*)distance_violation_state_service);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid(distance) returned != 0 status=0x%04x", (unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Connected" );
        displayPrintf(DISPLAY_ROW_BTADDR2, "%x:%x:%x:%x:%x:%x",
                                         bleDataPtr->serverAddress.addr[5],
                                         bleDataPtr->serverAddress.addr[4],
                                         bleDataPtr->serverAddress.addr[3],
                                         bleDataPtr->serverAddress.addr[2],
                                         bleDataPtr->serverAddress.addr[1],
                                         bleDataPtr->serverAddress.addr[0]);

        nextState = discoverTempService;

        //if close evt occurs then next state is scanning
        if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
        {
          sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
          if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
          }
          displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//          displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//          displayPrintf(DISPLAY_ROW_BTADDR2, "");

          nextState = scanning;
        }
      }
      break;

    case discoverTempService:
      nextState = discoverTempService;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id )
      {

        sc = sl_bt_gatt_discover_primary_services_by_uuid(bleDataPtr->connectionHandle,
                                                       sizeof(motion_violation_state_service),
                                                       (const uint8_t*)motion_violation_state_service);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_discover_primary_services_by_uuid(motion) returned != 0 status=0x%04x", (unsigned int) sc);
        }
        nextState = discoverButtonStateService;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");

        nextState = scanning;
      }
      break;

    case discoverButtonStateService:
      nextState = discoverButtonStateService;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id )
      {

        sc = sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr->connectionHandle,
                                                         bleDataPtr->distance_state_service_handle,
                                                         sizeof(distance_violation_state_char),
                                                         (const uint8_t*)distance_violation_state_char);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid(distance) returned != 0 status=0x%04x", (unsigned int) sc);
        }
        nextState = discoverTempCharacteristic;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");

        nextState = scanning;
      }
      break;

    case discoverTempCharacteristic:
      nextState = discoverTempCharacteristic;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id )
      {

        sc = sl_bt_gatt_discover_characteristics_by_uuid(bleDataPtr->connectionHandle,
                                                         bleDataPtr->motion_state_service_handle,
                                                         sizeof(motion_violation_state_char),
                                                         (const uint8_t*)motion_violation_state_char);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_discover_characteristics_by_uuid(motion) returned != 0 status=0x%04x", (unsigned int) sc);
        }

        nextState = discoverButtonStateCharacteristic;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");

        nextState = scanning;
      }
      break;

    case discoverButtonStateCharacteristic:
      nextState = discoverButtonStateCharacteristic;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id )
      {

        sc = sl_bt_gatt_set_characteristic_notification(bleDataPtr->connectionHandle,
                                                        bleDataPtr->distance_state_characteristic_handle,
                                                        sl_bt_gatt_indication);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_set_characteristic_notification(distance) returned != 0 status=0x%04x", (unsigned int) sc);
        }


        nextState = EnableButtonStateIndication;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");

        nextState = scanning;
      }
      break;


    case EnableButtonStateIndication:
      nextState = EnableButtonStateIndication;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id )
      {

        sc = sl_bt_gatt_set_characteristic_notification(bleDataPtr->connectionHandle,
                                                        bleDataPtr->motion_state_characteristic_handle,
                                                        sl_bt_gatt_indication);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_gatt_set_characteristic_notification() returned != 0 status=0x%04x", (unsigned int) sc);
        }
//        displayPrintf(DISPLAY_ROW_CONNECTION, "Handling Indications" );

        nextState = EnableTempIndication;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");

        nextState = scanning;
      }
      break;

    case EnableTempIndication:
      nextState = EnableTempIndication;
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_procedure_completed_id )
      {

        displayPrintf(DISPLAY_ROW_ACTION, "Press PB1 to bond");

        nextState = waitForTempReading;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");

        nextState = scanning;
      }
      break;

    case waitForTempReading:
      nextState = waitForTempReading;
      //LOG_INFO("In temp wait state");
      //gatt value id is the evt then remain in the state
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_characteristic_value_id  )
      {
        //if char handle and attribute code is of temp
        if ( evt->data.evt_gatt_characteristic_value.characteristic == bleDataPtr->distance_state_characteristic_handle)
        {
          //uint32_t temperature = gattFloat32ToInt(&(evt->data.evt_gatt_characteristic_value.value.data[0]));
          uint8_t distance_violation_count = evt->data.evt_gatt_characteristic_value.value.data[0];
          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Dist Violation: %d", distance_violation_count);
          sc = sl_bt_gatt_send_characteristic_confirmation(bleDataPtr->connectionHandle);
          if (sc != SL_STATUS_OK)
          {
           LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation() returned != 0 status=0x%04x", (unsigned int) sc);
          }
        }
        //if char handle and attribute code is of motion state count
        if ( evt->data.evt_gatt_characteristic_value.characteristic == bleDataPtr->motion_state_characteristic_handle && (sl_bt_gatt_handle_value_indication == evt->data.evt_gatt_characteristic_value.att_opcode))
        {
          uint8_t motion_violation_count = evt->data.evt_gatt_characteristic_value.value.data[0];

          displayPrintf(DISPLAY_ROW_9, "Motion Violation: %d", motion_violation_count);

          sc = sl_bt_gatt_send_characteristic_confirmation(bleDataPtr->connectionHandle);
          if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_gatt_send_characteristic_confirmation(button) returned != 0 status=0x%04x", (unsigned int) sc);
          }
        }
        if (evt->data.evt_gatt_characteristic_value.characteristic == bleDataPtr->motion_state_characteristic_handle && (sl_bt_gatt_read_response  == evt->data.evt_gatt_characteristic_value.att_opcode))
        {
            uint8_t motion_violation_count_t = evt->data.evt_gatt_characteristic_value.value.data[0];

            displayPrintf(DISPLAY_ROW_9, "Motion Violation: %d", motion_violation_count_t);

        }

        nextState = waitForTempReading;
      }

      //if close evt occurs then next state is scanning
      if ( SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_closed_id  )
      {
        sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
        }
        displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
//        displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
//        displayPrintf(DISPLAY_ROW_BTADDR2, "");
//        displayPrintf(DISPLAY_ROW_9, "");

        nextState = scanning;
      }
      break;

    default:
      break;

  } //end switch

} //discovery_state_machine(sl_bt_msg_t *evt)


void MPU6050_state_machine(sl_bt_msg_t *evt)
{
  State_MPU6050_t currentState;
  static State_MPU6050_t nextState = state_Idle;
  currentState = nextState;

  ble_data_struct_t *bleDataPtr = getBleDataPtr();
  sl_status_t sc;
  queue_t *circularBufferPtr = getQueuePtr();

  if (SL_BT_MSG_ID(evt->header) == sl_bt_evt_system_external_signal_id)
  {

    switch (currentState)
    {
      case state_Idle:
        nextState = state_Idle;
        if ( evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF )
        {
          //power the MPU6050
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          timerWaitUs_irq(100000);        //wait 100ms
          i2cInit();

          nextState = state_1;
        }
        break;

      case state_1:
        nextState = state_1;
        if ( evt->data.evt_system_external_signal.extsignals == evtCOMP1 )
        {
          i2cWritePWRValueToMPU6050();
          schedularSetStartUltrasonicEvent();
          nextState = state_2;
        }
        break;

      case state_2:
         nextState = state_2;
         if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
         {
           i2cWriteReadPWRToMPU6050();

           nextState = state_3;
         }
         break;

      case state_3:
        nextState = state_3;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {
          i2cWriteReadToWhoAmIMPU6050();

          nextState = state_4;
        }
        break;

      case state_4:
        nextState = state_4;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {
          i2cWriteSampleRateValueToMPU6050();

          nextState = state_5;
        }
        break;

      case state_5:
        nextState = state_5;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {
          i2cWriteReadSampleRateToMPU6050();

          nextState = state_6;
        }
        break;

      case state_6:
        nextState = state_6;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteConfigValueToMPU6050();

          nextState = state_7;
        }
        break;

      case state_7:
        nextState = state_7;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteAccelConfigValueToMPU6050();

          nextState = state_8;
        }
        break;

      case state_8:
        nextState = state_8;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadConfigToMPU6050();

          nextState = state_9;
        }
        break;

      case state_9:
        nextState = state_9;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadAccelConfigToMPU6050();

          nextState = state_10;
        }
        break;

      case state_10:
        nextState = state_10;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadAccelXHighToMPU6050();

          nextState = state_11;
        }
        break;

      case state_11:
        nextState = state_11;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteInterruptConfigValueToMPU6050();

          nextState = state_12;
        }
        break;

      case state_12:
        nextState = state_12;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadInterruptConfigToMPU6050();

          nextState = state_13;
        }
        break;

      case state_13:
        nextState = state_13;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteUserConfigValueToMPU6050();

          nextState = state_14;
        }
        break;

      case state_14:
        nextState = state_14;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadUserConfigToMPU6050();

          nextState = state_15;
        }
        break;

      case state_15:
        nextState = state_15;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteIntPinConfigValueToMPU6050();

          nextState = state_16;
        }
        break;

      case state_16:
        nextState = state_16;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadIntPinConfigToMPU6050();

          nextState = state_17;
        }
        break;

      case state_17:
        nextState = state_17;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {
          i2cWriteFIFOEnableValueToMPU6050();

          nextState = state_18;
        }
        break;

      case state_18:
        nextState = state_18;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadFIFOEnableToMPU6050();

          nextState = state_19;
        }
        break;

      case state_19:
        nextState = state_19;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {

          i2cWriteReadAccelXYZToMPU6050();

          nextState = state_20;
        }
        break;

      case state_20:
        nextState = state_20;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {



          nextState = state_21;
        }
        break;

      case state_21:
        nextState = state_21;
        if ( evt->data.evt_system_external_signal.extsignals == evtSoftTimer )
        {
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
          writeFIFODataToBuffer();


          nextState = state_22;
        }
        break;

      case state_22:
        nextState = state_22;
        if ( evt->data.evt_system_external_signal.extsignals == evtSoftTimer )
        {
          sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
          i2cWriteReadAccelXYZToMPU6050();
          if (bleDataPtr->count >= 1)
          {
            bleDataPtr->count -= 1;
          }
          if (bleDataPtr->count == 0)
          {
              gpioLed0SetOff();
          }

          nextState = final_state;
        }
        break;

      case final_state:
        nextState = final_state;
        if ( evt->data.evt_system_external_signal.extsignals == evtI2CTransferDone )
        {
          sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);

          writeFIFODataToBuffer();
          sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_motion_violation_count, // handle from gatt_db.h
          0, // offset
          1, // length
          &bleDataPtr->motion_violation_count // pointer to buffer where data is
          );

          if (sc != SL_STATUS_OK) {
            LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
          }

          //send indications if indication is set and connection is open

          if (bleDataPtr->distanceIndicationSet == true  && bleDataPtr->connectionOpen == true && bleDataPtr->bonded == true)
          {
              //LOG_INFO("flags %d\r",bleDataPtr->inFlight);
            if (bleDataPtr->inFlight == false)
            {
              sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionHandle,gattdb_motion_violation_count,
                                                       1,
                                                       &bleDataPtr->motion_violation_count);
              if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
              }
              bleDataPtr->inFlight = true;
            }
            else
            {
              indication_queue_t i;
              i.charHandle = gattdb_motion_violation_count;
              i.bufferLength =  1;
              i.buffer[0] = bleDataPtr->motion_violation_count;
              enqueue(circularBufferPtr ,i);
            }

            //displayPrintf(DISPLAY_ROW_TEMPVALUE, "Distance = %.02f", distance);
            displayPrintf(DISPLAY_ROW_9, "Motion Violation: %d", bleDataPtr->motion_violation_count);

          }



          nextState = state_22;
        }
        break;

      default:
       break;

    } // end of switch()
  }
}  // end of MPU6050_state_machine()



void ultrasonic_state_machine(sl_bt_msg_t*evt)
{

  State_Ultrasonic_t currentstate;
  static State_Ultrasonic_t nextstate  = start;

  ble_data_struct_t *bleDataPtr = getBleDataPtr();
  sl_status_t sc;
  queue_t *circularBufferPtr = getQueuePtr();

  currentstate = nextstate;

  switch(currentstate)
  {
    case start:
      nextstate = start;
      if(( evt->data.evt_system_external_signal.extsignals == evtStartUltrasonic ))  //when UF interrupt flag occurs
      {

        //LOG_INFO("In Start State\r");

        nextstate = enable_trigger;
      }
      break;

    case enable_trigger:
      nextstate = enable_trigger;
      if(( evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF )) //When Comp1 interrupt occurs 10us
      {
        sl_power_manager_add_em_requirement(SL_POWER_MANAGER_EM1);
        ultrasonic_sensor_trigger_enable();
        timerWaitUs_irq(20);     //wait for 10us
        gpioLed1SetOff();
        //LOG_INFO("In Start State\r");
        nextstate = trigger_state;
      }
      break;


    case trigger_state:
      nextstate = trigger_state;
      if(( evt->data.evt_system_external_signal.extsignals == evtCOMP1 )) //When Comp1 interrupt occurs 10us
      {
        ultrasonic_sensor_trigger_disable();
        sl_power_manager_remove_em_requirement(SL_POWER_MANAGER_EM1);
        //LOG_INFO("In Trigger State\r");

        nextstate = echo_state;
      }
      if(( evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF ))
      {

        nextstate = enable_trigger;
      }
      break;

    case echo_state:
      nextstate = echo_state;
      //LOG_INFO("event %d\r", evt->data.evt_system_external_signal.extsignals);
      if(( evt->data.evt_system_external_signal.extsignals == evtEcho)) //When echo pin is high use GPIO irq
      {
        //start counting how long the the pin is high
        echo_start_time = LETIMER_CounterGet(LETIMER0);

        //LOG_INFO("echo start time %d:\r", echo_start_time);
        nextstate = calculate_distance;
      }
      if(( evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF ))
      {

        nextstate = enable_trigger;
      }
      break;

     case calculate_distance:
      nextstate = calculate_distance;
      if(( evt->data.evt_system_external_signal.extsignals == evtEcho)) //When echo pin is high use GPIO irq
      {

        //start counting how long the the pin is high
        echo_stop_time = LETIMER_CounterGet(LETIMER0);
        //LOG_INFO("echo stop time %d:\r", echo_stop_time);
        //LOG_INFO("echo start time %d:\r", echo_start_time);
        echo_time = echo_start_time - echo_stop_time;
        //LOG_INFO("echo diff time %d:\r", echo_time);
        distance = (float)(echo_time * 1000000 ) / 8192;
        distance = distance / 58;
        if (distance < 30)
        {
          bleDataPtr->distance_violation_count += 1 ;
          gpioLed1SetOn();
        }
        // Write our local GATT DB
        sc = sl_bt_gatt_server_write_attribute_value(
        gattdb_distance_violation_count, // handle from gatt_db.h
        0, // offset
        1, // length
        &bleDataPtr->distance_violation_count // pointer to buffer where data is
        );

        if (sc != SL_STATUS_OK) {
          LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
        }

        //send indications if indication is set and connection is open
        //Could remove as state machine only runs when this condition is true
        if (bleDataPtr->distanceIndicationSet == true  && bleDataPtr->connectionOpen == true && bleDataPtr->bonded == true )
        {
          if (bleDataPtr->inFlight == false)
          {
            sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionHandle,gattdb_distance_violation_count,
                                                     1,
                                                     &bleDataPtr->distance_violation_count);
            if (sc != SL_STATUS_OK)
            {
              LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
            }
            bleDataPtr->inFlight = true;
          }
          else
          {
              LOG_INFO("distance queue");
            indication_queue_t i;
            i.charHandle = gattdb_distance_violation_count;
            i.bufferLength =  1;
            i.buffer[0] = bleDataPtr->distance_violation_count;
            enqueue(circularBufferPtr ,i);
          }

          displayPrintf(DISPLAY_ROW_TEMPVALUE, "Distance = %.02f", distance);
          displayPrintf(DISPLAY_ROW_8  , "Dist Violation: %d", bleDataPtr->distance_violation_count);
          if( bleDataPtr->distance_violation_count + bleDataPtr->motion_violation_count  > 5)
            {
              sl_bt_connection_close(bleDataPtr->connectionHandle);
            }
//          LOG_INFO("Distance %.02f cm\r", distance);

        }
        LOG_INFO("Distance Violation count: %d\r", bleDataPtr->distance_violation_count);

        nextstate = enable_trigger;
      }
      if(( evt->data.evt_system_external_signal.extsignals == evtLETIMER0_UF )) //When Comp1 interrupt occurs 10us
      {

        nextstate = enable_trigger;
      }
      break;

    default:
      break;

  } // switch
} //ultrasonic state_machine


void server_state_machine(sl_bt_msg_t*evt)
{

  State_Server_t currentstate;
  static State_Server_t next_state  = Idle;

  ble_data_struct_t *bleDataPtr = getBleDataPtr();
  sl_status_t sc;
  //queue_t *circularBufferPtr = getQueuePtr();

  currentstate = next_state;

  if( bleDataPtr->bonded == true)
    switch(currentstate)
    {
    case Idle:
      next_state = Idle;
      if( (SL_BT_MSG_ID(evt->header) == sl_bt_evt_sm_bonded_id) ||  (SL_BT_MSG_ID(evt->header) == sl_bt_evt_connection_opened_id))
      {
        LOG_INFO("In idle\r");
        if (bleDataPtr->inFlight == false && bleDataPtr->distanceIndicationSet == true )
        {
            LOG_INFO("In idle\r");

          sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionHandle,gattdb_distance_violation_count,
                                                   1,
                                                   &bleDataPtr->distance_violation_count);
          if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_gatt_server_send_indication(distance) returned != 0 status=0x%04x", (unsigned int) sc);
            sc = sl_bt_sm_increase_security(bleDataPtr->connectionHandle);
              if (sc != SL_STATUS_OK)
              {
                LOG_ERROR("sl_bt_sm_increase_security() returned != 0 status=0x%04x", (unsigned int) sc);
              }
              server_connection = true;
          }
          //bleDataPtr->inFlight = true;
        }
        next_state = connection_open;
      }
      break;


    case connection_open:
      next_state = connection_open;
      if( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_server_characteristic_status_id)
      {
          LOG_INFO("In soething again\r");
        if (bleDataPtr->inFlight == false && bleDataPtr->distanceIndicationSet == true )
        {
          sc = sl_bt_gatt_server_send_indication(bleDataPtr->connectionHandle,gattdb_motion_violation_count,
                                                   1,
                                                   &bleDataPtr->motion_violation_count);
          if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_gatt_server_send_indication(motion) returned != 0 status=0x%04x", (unsigned int) sc);

          }

          //bleDataPtr->inFlight = true;
        }
        next_state = send_motion_count;
      }
      break;
    case send_motion_count:
      next_state = send_motion_count;
      if( SL_BT_MSG_ID(evt->header) == sl_bt_evt_gatt_server_characteristic_status_id )
      {
         sl_bt_connection_close(bleDataPtr->connectionHandle);
         LOG_INFO("In connetion close\r");
        next_state = send_distance_count;
      }
      break;
    case send_distance_count:
      next_state = send_distance_count;
      if( evt->data.evt_system_soft_timer.handle == 20 )
      {
        sc = sl_bt_advertiser_start(
                bleDataPtr->advertisingSetHandle,
                sl_bt_advertiser_general_discoverable,
                sl_bt_advertiser_connectable_scannable);
        if (sc != SL_STATUS_OK)
        {
          LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
        }
        next_state = Idle;
      }
      break;
    default:
      break;
    }



}












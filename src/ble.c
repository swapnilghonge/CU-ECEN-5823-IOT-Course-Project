/*******************************************************************************
 * @file ble.c
 * @brief Bluetooth event handler
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  Course Project
 * @date:        1-10-2021
 * @author:      Peter Braganza
 * Description:  Contains bluetooth event handler which will handle all event
 *               from the bluetooth stack
 ******************************************************************************/


#include "ble.h"
#include "scheduler.h"
#include "lcd.h"


#define INCLUDE_LOG_DEBUG 1
#include "src/log.h"

// BLE private data
ble_data_struct_t ble_data;

queue_t circular_buffer;

void queue_init(queue_t *q)
{
  q->head = 0;
  q->tail = 0;
  q->queue_size = 0;

} //queue_init(queue_t *q)

void enqueue(queue_t *q, indication_queue_t i)
{

 // CORE_DECLARE_IRQ_STATE;
 // CORE_ENTER_CRITICAL();

  if(q->queue_size >= MAX_QUEUE_SIZE)
    {
      //handling overflow case of circular buffer

      q->tail++;
      q->queue_size--;
      if (q->tail >= MAX_QUEUE_SIZE)
      {
        q->tail = 0;
      }

    }
  q->indication_queue[q->head] = i;
  q->head++;
  q->queue_size++;
  if (q->head >= MAX_QUEUE_SIZE)
  {
    q->head = 0;
  }
  //CORE_EXIT_CRITICAL();

} // enqueue(queue_t *q, indication_queue_t i)

indication_queue_t* dequeue(queue_t *q)
{

  // CORE_DECLARE_IRQ_STATE;

  // CORE_ENTER_CRITICAL();

  if(q->queue_size == 0)
  {
    return NULL;
  }
  q->queue_size--;


  indication_queue_t* temp_indication = &(q->indication_queue[q->tail]);
  q->tail++;
  if (q->tail >= MAX_QUEUE_SIZE)
  {
    q->tail = 0;
  }


  // CORE_EXIT_CRITICAL();
  return temp_indication;

} //dequeue(queue_t *q)

// function that returns a pointer to the BLE private data
ble_data_struct_t* getBleDataPtr() {
 return (&ble_data);
} // getBleDataPtr()

// function that returns a pointer to the circular buffer
queue_t* getQueuePtr() {
 return (&circular_buffer);
} // getQueuePtr()


//Bluetooth event handler which is continuously called from app_process_action()
void handle_ble_event(sl_bt_msg_t *evt)
{
  //DOS ble_data.advertisingSetHandle = 0xff;

  sl_status_t sc;


#if DEVICE_IS_BLE_SERVER
  uint8_t button_state;
#else
  bool addrIsSame = true;
#endif

  switch (SL_BT_MSG_ID(evt->header))
  {
  // ******************************************************
  // Events common to both Servers and Clients
  // ******************************************************
  // --------------------------------------------------------
  // This event indicates the device has started and the radio is ready.
  // Do not call any stack API commands before receiving this boot event!
  // Including starting BT stack soft timers!
  // --------------------------------------------------------
  case sl_bt_evt_system_boot_id:

#if DEVICE_IS_BLE_SERVER

    sc = sl_bt_sm_delete_bondings();
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
    }
    sc = sl_bt_sm_store_bonding_configuration(2,1);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_store_bonding_configuration() returned != 0 status=0x%04x", (unsigned int) sc);
    }
    // set up the security manager
    sc = sl_bt_sm_configure (
      SM_CONFIG_FLAGS,
      sm_io_capability_displayyesno
    );
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_configure() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
    }

//    LOG_INFO("1: %x 2: %x 3: %x, 4: %x, 5: %x, 6: %x, ",
//                   ble_data.myAddress.addr[5],
//                   ble_data.myAddress.addr[4],
//                   ble_data.myAddress.addr[3],
//                   ble_data.myAddress.addr[2],
//                   ble_data.myAddress.addr[1],
//                   ble_data.myAddress.addr[0]);

    ble_data.advertisingSetHandle = 0xff; // DOS
    ble_data.bonded = false;
    sc = sl_bt_advertiser_create_set(&ble_data.advertisingSetHandle);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_advertiser_create_set() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_advertiser_set_timing(ble_data.advertisingSetHandle, 400, 400, 0, 0);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_advertiser_set_timing() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_advertiser_start(
            ble_data.advertisingSetHandle,
            sl_bt_advertiser_general_discoverable,
            sl_bt_advertiser_connectable_scannable);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_system_set_soft_timer(8198*2, 12, false);  //seting softtimer to about 250ms
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_system_set_soft_timer() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_system_set_soft_timer(1640, 10, false);  //seting softtimer to about 50ms
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_system_set_soft_timer() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_system_set_soft_timer(8198*30, 20, false);  // 6sec
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_system_set_soft_timer() returned != 0 status=0x%04x", (unsigned int) sc);
    }
    queue_init(&circular_buffer);

    displayInit();
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Course Project" );
    displayPrintf(DISPLAY_ROW_NAME, "SERVER" );
    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising" );
    displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x",
                                     ble_data.myAddress.addr[5],
                                     ble_data.myAddress.addr[4],
                                     ble_data.myAddress.addr[3],
                                     ble_data.myAddress.addr[2],
                                     ble_data.myAddress.addr[1],
                                     ble_data.myAddress.addr[0]);

    ble_data.buttonState = false;
    ble_data.passKeyConfirm = false;
    ble_data.distance_violation_count = 0;

#else
    sc = sl_bt_system_get_identity_address(&ble_data.myAddress, &ble_data.myAddressType);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_system_get_identity_address() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    bd_addr server_addr  = SERVER_BT_ADDRESS;
    ble_data.serverAddress = server_addr;
    sc = sl_bt_scanner_set_mode(sl_bt_gap_1m_phy, 0); //2nd arg is 0 for passive scanning change it to a macro later
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_scanner_set_mode() returned != 0 status=0x%04x", (unsigned int) sc);
    }
    sc = sl_bt_scanner_set_timing(sl_bt_gap_1m_phy, 80, 40);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_scanner_set_timing() returned != 0 status=0x%04x",(unsigned int) sc);
    }
    // Set the default connection parameters for subsequent connections
    sc = sl_bt_connection_set_default_parameters(60,
                                                 60,
                                                 4,
                                                 750,
                                                 0,
                                                 0xffff);

    sc = sl_bt_sm_delete_bondings();
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
    }
    // set up the security manager
    sc = sl_bt_sm_configure (
      SM_CONFIG_FLAGS,
      sm_io_capability_displayyesno
    );
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_configure() returned != 0 status=0x%04x", (unsigned int) sc);
    }
    sc = sl_bt_scanner_start(sl_bt_gap_1m_phy, sl_bt_scanner_discover_generic);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_scanner_start() returned != 0 status=0x%04x",(unsigned int) sc);
    }
    //ble_data.PB1Pressed = false;
    ble_data.passKeyConfirm = false;
    ble_data.bonded = false;
    ble_data.buttonState = false;

    displayInit();
    displayPrintf(DISPLAY_ROW_ASSIGNMENT, "Course Project" );
    displayPrintf(DISPLAY_ROW_NAME, "CLIENT" );
    displayPrintf(DISPLAY_ROW_CONNECTION, "Discovering" );
    displayPrintf(DISPLAY_ROW_BTADDR, "%x:%x:%x:%x:%x:%x",
                                     ble_data.myAddress.addr[5],
                                     ble_data.myAddress.addr[4],
                                     ble_data.myAddress.addr[3],
                                     ble_data.myAddress.addr[2],
                                     ble_data.myAddress.addr[1],
                                     ble_data.myAddress.addr[0]);

#endif
    break;


  case sl_bt_evt_gatt_procedure_completed_id:
#if DEVICE_IS_BLE_SERVER == 0

    if (evt->data.evt_gatt_procedure_completed.result == 0x110f)
    {
      sc = sl_bt_sm_increase_security(ble_data.connectionHandle);
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_sm_increase_security() returned != 0 status=0x%04x", (unsigned int) sc);
      }
    }
#endif
    if (evt->data.evt_gatt_procedure_completed.result == 0x110f)
    {
      sc = sl_bt_sm_increase_security(ble_data.connectionHandle);
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_sm_increase_security() returned != 0 status=0x%04x", (unsigned int) sc);
      }
      LOG_INFO("increasing\r");
    }
    break;


  case sl_bt_evt_connection_opened_id:
#if DEVICE_IS_BLE_SERVER

    ble_data.connectionHandle = evt->data.evt_connection_opened.connection;
    ble_data.connectionOpen   = true;

    sc = sl_bt_advertiser_stop(ble_data.advertisingSetHandle);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_advertiser_stop() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    sc = sl_bt_connection_set_parameters(
        ble_data.connectionHandle, // connection handle
        60,   // min_interval, Time = Value x 1.25 ms, Range: 0x0006 to 0x0c80
        60,   // max_interval, Time = Value x 1.25 ms, Range: 0x0006 to 0x0c80
        4,    // slave latency, Range: 0x0000 to 0x01f4
        750,   // supervision timeout, Time = Value x 10 ms
        //                      Range: 0x000a to 0x0c80, 100 ms to 32 s
        //                      The value in milliseconds must be larger than
        //                      (1 + latency) * max_interval * 2, where max_interval
        //                      is given in milliseconds. Set the supervision timeout
        //                      at a value which allows communication attempts over
        //                      at least a few connection intervals.
        0,    // min connection event length, Time = Value x 0.625 ms,
              //                      Range: 0x0000 to 0xffff, not used, set to 0

        0     // max_connection event length, Time = Value x 0.625 ms,
              // Range: 0x0000 to 0xffff, not used, set to 0
        );

    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_connection_set_parameters() returned != 0 status=0x%04x", (unsigned int) sc);
    }

    displayPrintf(DISPLAY_ROW_CONNECTION, "Connected" );
    if(ble_data.bonded == true)
    {


    }
#else
    ble_data.connectionHandle = evt->data.evt_connection_opened.connection;
    ble_data.connectionOpen   = true;
#endif
    break;

  // This event is generated when an advertisement packet or a scan response is received from a responder
  case sl_bt_evt_scanner_scan_report_id:
#if DEVICE_IS_BLE_SERVER == 0

    //checks if sever address in ble_device_type is same as discovered address
    for (int i = 0; i < 5; i++)
    {
      if(evt->data.evt_scanner_scan_report.address.addr[i] != ble_data.serverAddress.addr[i])
        {
          addrIsSame = false;
          break;
        }
    }

    if ( (evt->data.evt_scanner_scan_report.address_type == ble_data.myAddressType)  && addrIsSame)
    {
      sc = sl_bt_scanner_stop();
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_scanner_stop() returned != 0 status=0x%04x", (unsigned int) sc);
      }
      sc = sl_bt_connection_open(evt->data.evt_scanner_scan_report.address,
                                 evt->data.evt_scanner_scan_report.address_type,
                                 sl_bt_gap_1m_phy,
                                 NULL);
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_connection_open() returned != 0 status=0x%04x", (unsigned int) sc);
      }
    }

#endif
    break;

  case sl_bt_evt_connection_parameters_id:
//        LOG_INFO("Connection params: connection=%d, interval=%d, latency=%d, timeout=%d, securitymode=%d",
//             (int) (evt->data.evt_connection_parameters.connection),
//             (int) (evt->data.evt_connection_parameters.interval*1.25),
//             (int) (evt->data.evt_connection_parameters.latency),
//             (int) (evt->data.evt_connection_parameters.timeout*10),
//             (int) (evt->data.evt_connection_parameters.security_mode) );
#if DEVICE_IS_BLE_SERVER ==0
//    LOG_INFO("Connection params: connection=%d, interval=%d, latency=%d, timeout=%d, securitymode=%d",
//         (int) (evt->data.evt_connection_parameters.connection),
//         (int) (evt->data.evt_connection_parameters.interval*1.25),
//         (int) (evt->data.evt_connection_parameters.latency),
//         (int) (evt->data.evt_connection_parameters.timeout*10),
//         (int) (evt->data.evt_connection_parameters.security_mode) );
#endif
    break;

  case sl_bt_evt_connection_closed_id:
#if DEVICE_IS_BLE_SERVER

//    sc = sl_bt_sm_delete_bondings();
//    if (sc != SL_STATUS_OK)
//    {
//      LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
//    }

//    sc = sl_bt_advertiser_start(
//            ble_data.advertisingSetHandle,
//            sl_bt_advertiser_general_discoverable,
//            sl_bt_advertiser_connectable_scannable);
//    if (sc != SL_STATUS_OK)
//    {
//      LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
//    }
    ble_data.connectionOpen = false;
//    ble_data.temperatureIndicationSet = false;
//    ble_data.buttonIndicationSet = false;
//    ble_data.motionIndicationSet = false;
//    ble_data.distanceIndicationSet = false;
    ble_data.bonded = false;
//    queue_init(&circular_buffer);
//
    ble_data.inFlight = false;
    displayPrintf(DISPLAY_ROW_CONNECTION, "Advertising" );
    displayPrintf(DISPLAY_ROW_TEMPVALUE, "Learn to drive");
    displayPrintf(DISPLAY_ROW_8, "");
    displayPrintf(DISPLAY_ROW_9, "");
    displayPrintf(DISPLAY_ROW_PASSKEY, "");
    displayPrintf(DISPLAY_ROW_ACTION, "");
    gpioLed1SetOff();
    gpioLed0SetOff();


#else
//    sc = sl_bt_sm_delete_bondings();
//     if (sc != SL_STATUS_OK)
//     {
//       LOG_ERROR("sl_bt_sm_delete_bondings() returned != 0 status=0x%04x", (unsigned int) sc);
//     }
//     ble_data.bonded = false;
     ble_data.connectionOpen = false;

     displayPrintf(DISPLAY_ROW_TEMPVALUE, "");
     displayPrintf(DISPLAY_ROW_8, "");
     displayPrintf(DISPLAY_ROW_9, "");
     displayPrintf(DISPLAY_ROW_PASSKEY, "");
     displayPrintf(DISPLAY_ROW_ACTION, "");
#endif
    break;

  case sl_bt_evt_system_external_signal_id:

#if DEVICE_IS_BLE_SERVER
    if (evt->data.evt_system_external_signal.extsignals == evtPB0Pressed  && ble_data.passKeyConfirm == true && ble_data.buttonState == true )
    {
      sc = sl_bt_sm_passkey_confirm(ble_data.connectionHandle, 1);
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_gatt_server_send_indication(hello) returned != 0 status=0x%04x", (unsigned int) sc);
      }
      ble_data.passKeyConfirm = false;
    }

    if (evt->data.evt_system_external_signal.extsignals == evtPB0Pressed)
    {
      if (ble_data.buttonState == true)
      {
        ble_data.buttonState = false;
        button_state = 0;
        //displayPrintf(DISPLAY_ROW_9, "Button Released");
      }
      else if (ble_data.buttonState == false)
      {
        ble_data.buttonState = true;
        button_state = 1;
        //displayPrintf(DISPLAY_ROW_9, "Button Pressed");
      }

      sc = sl_bt_gatt_server_write_attribute_value(
          gattdb_button_state, // handle from gatt_db.h
          0,                   // offset
          1,                   // length
          &button_state        // pointer to buffer where data is
          );
      if (sc != SL_STATUS_OK)
      {
       LOG_ERROR("sl_bt_gatt_server_write_attribute_value() returned != 0 status=0x%04x", (unsigned int) sc);
      }

//      if (ble_data.bonded == true && ble_data.connectionOpen == true && ble_data.buttonIndicationSet == true)
//      {
//        if (ble_data.inFlight == false)
//        {
//          sc = sl_bt_gatt_server_send_indication(ble_data.connectionHandle,gattdb_button_state,
//                                                               sizeof(button_state),
//                                                               &button_state);
//          if (sc != SL_STATUS_OK)
//          {
//            LOG_ERROR("sl_bt_gatt_server_send_indication() returned != 0 status=0x%04x", (unsigned int) sc);
//          }
//          ble_data.inFlight = true;
//        }
//        else
//        {
//          indication_queue_t i;
//          i.charHandle = gattdb_button_state;
//          i.bufferLength =  sizeof(button_state);
//          i.buffer[0] = button_state;
//          enqueue(&circular_buffer ,i);
//          //LOG_INFO("enqueue : size: %d, head: %d,tail: %d\r\n", circular_buffer.queue_size,circular_buffer.head, circular_buffer.tail);
//        }
//      }
    }
#else
    if (evt->data.evt_system_external_signal.extsignals == evtPB1Pressed && ble_data.connectionOpen && (ble_data.buttonState == false))
    {
      sc = sl_bt_gatt_read_characteristic_value(ble_data.connectionHandle, ble_data.distance_state_characteristic_handle);
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_gatt_read_characteristic_value() returned != 0 status=0x%04x", (unsigned int) sc);
      }
    }
    if (evt->data.evt_system_external_signal.extsignals == evtPB0Pressed  && ble_data.passKeyConfirm == true)
    {
      sc = sl_bt_sm_passkey_confirm(ble_data.connectionHandle, 1);
      if (sc != SL_STATUS_OK)
      {
        LOG_ERROR("sl_bt_sm_passkey_confirm() returned != 0 status=0x%04x", (unsigned int) sc);
      }
      ble_data.passKeyConfirm = false;
    }
    if (evt->data.evt_system_external_signal.extsignals == evtPB0Pressed)
    {
      if (ble_data.buttonState == true)
      {
       ble_data.buttonState = false;
      }
      else if (ble_data.buttonState == false)
      {
       ble_data.buttonState = true;
      }
    }
//    if ( evt->data.evt_system_external_signal.extsignals == evtPB1Pressed && ble_data.buttonState == true)
//    {
//      if (ble_data.buttonState == true)
//      {
//        sc = sl_bt_gatt_set_characteristic_notification(ble_data.connectionHandle,
//                                                         ble_data.button_state_characteristic_handle,
//                                                         sl_bt_gatt_disable);
//        if (sc != SL_STATUS_OK)
//        {
//         LOG_ERROR("sl_bt_gatt_set_characteristic_notification(PB1) returned != 0 status=0x%04x", (unsigned int) sc);
//        }
//      }
//    }

#endif
    break;

  case sl_bt_evt_sm_confirm_bonding_id:
#if DEVICE_IS_BLE_SERVER
    sc = sl_bt_sm_bonding_confirm(ble_data.connectionHandle,1);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_bonding_confirm() returned != 0 status=0x%04x", (unsigned int) sc);
    }
#else
    sc = sl_bt_sm_bonding_confirm(ble_data.connectionHandle,1);
    if (sc != SL_STATUS_OK)
    {
      LOG_ERROR("sl_bt_sm_bonding_confirm() returned != 0 status=0x%04x", (unsigned int) sc);
    }
#endif
    break;

  //Indicates a request for passkey display and confirmation by the user
  case sl_bt_evt_sm_confirm_passkey_id:
    if(ble_data.bonded == false)
    {
      displayPrintf(DISPLAY_ROW_PASSKEY, "Passkey %06d" , evt->data.evt_sm_passkey_display.passkey);
      displayPrintf(DISPLAY_ROW_ACTION, "Confirm with PB0");
      ble_data.passKeyConfirm = true;
    }

    break;


  //Triggered after the pairing or bonding procedure is successfully completed
  case sl_bt_evt_sm_bonded_id:

    displayPrintf(DISPLAY_ROW_CONNECTION, "Bonded");
    displayPrintf(DISPLAY_ROW_PASSKEY, "");
    displayPrintf(DISPLAY_ROW_ACTION, "");
    ble_data.bonded = true;

    break;


  //event occurs when soft timer expires
  case sl_bt_evt_system_soft_timer_id:
#if DEVICE_IS_BLE_SERVER
    if (evt->data.evt_system_soft_timer.handle == 10 && ble_data.inFlight == false)
    {
      indication_queue_t *i = dequeue(&circular_buffer);

      if (i != NULL)
      {
        if (i->charHandle == gattdb_distance_violation_count)
        {
          sc = sl_bt_gatt_server_send_indication(ble_data.connectionHandle,i->charHandle,
                                                                         i->bufferLength,
                                                                         &(i->buffer[0]));
          if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_gatt_server_send_indication(button) returned != 0 status=0x%04x", (unsigned int) sc);
          }
          ble_data.inFlight = true;
        }
        if (i->charHandle == gattdb_motion_violation_count)
        {
          sc = sl_bt_gatt_server_send_indication(ble_data.connectionHandle,i->charHandle,
                                                                         i->bufferLength,
                                                                         &(i->buffer[0]));
          if (sc != SL_STATUS_OK)
          {
            LOG_ERROR("sl_bt_gatt_server_send_indication(temp) returned != 0 status=0x%04x", (unsigned int) sc);
          }
          ble_data.inFlight = true;
        }
      //LOG_INFO("dequeue : size: %d, head: %d,tail: %d\r", circular_buffer.queue_size,circular_buffer.head, circular_buffer.tail);
      }
    }
    if (evt->data.evt_system_soft_timer.handle == 2)
    {
      displayUpdate();
    }
    if (evt->data.evt_system_soft_timer.handle == 12)
    {
      schedularSetSoftTimerEvent();
    }
//    if (evt->data.evt_system_soft_timer.handle == 20)
//    {
//      sc = sl_bt_advertiser_start(
//              ble_data.advertisingSetHandle,
//              sl_bt_advertiser_general_discoverable,
//              sl_bt_advertiser_connectable_scannable);
//      if (sc != SL_STATUS_OK)
//      {
//        LOG_ERROR("sl_bt_advertiser_start() returned != 0 status=0x%04x", (unsigned int) sc);
//      }
//    }

#else
    if (evt->data.evt_system_soft_timer.handle == 2)
    {
      displayUpdate();
    }

#endif
    break;

  case sl_bt_evt_gatt_service_id:
#if DEVICE_IS_BLE_SERVER == 0

    if (memcmp((const uint8_t*)distance_violation_state_service, (const uint8_t*)evt->data.evt_gatt_service.uuid.data,16) == 0)
    {
      ble_data.distance_state_service_handle = evt->data.evt_gatt_service.service;

    }
    if (memcmp((const uint8_t*)motion_violation_state_service, (const uint8_t*)evt->data.evt_gatt_service.uuid.data,16) == 0)
    {
      ble_data.motion_state_service_handle = evt->data.evt_gatt_service.service;

    }
#endif
    break;

  case sl_bt_evt_gatt_characteristic_id:
#if DEVICE_IS_BLE_SERVER == 0


    if (memcmp((const uint8_t*)distance_violation_state_char, (const uint8_t*)evt->data.evt_gatt_characteristic.uuid.data,16) == 0 )
    {
      ble_data.distance_state_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;

    }
    if (memcmp((const uint8_t*)motion_violation_state_char, (const uint8_t*)evt->data.evt_gatt_characteristic.uuid.data,16) == 0)
    {
      ble_data.motion_state_characteristic_handle = evt->data.evt_gatt_characteristic.characteristic;

    }
#endif
    break;

  case sl_bt_evt_gatt_server_characteristic_status_id:
#if DEVICE_IS_BLE_SERVER
    //implementation taken from sample code of health thermometer


    if (gattdb_distance_violation_count == evt->data.evt_gatt_server_characteristic_status.characteristic)
    {
      if(sl_bt_gatt_disable != (sl_bt_gatt_client_config_flag_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags)
      {
        ble_data.distanceIndicationSet = true;
        //gpioLed0SetOn();
        //LOG_INFO("indication set : %d", (int)ble_data.indicationSet);
      }
      else
      {
        ble_data.distanceIndicationSet = false;
        //gpioLed0SetOff();
        //displayPrintf(DISPLAY_ROW_9, "");
      }

      // confirmation of indication received from remove GATT client
      if (sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags)
      {
        (void)evt->data.evt_gatt_server_characteristic_status.connection;
        ble_data.inFlight = false;
      }
    }
    if (gattdb_motion_violation_count == evt->data.evt_gatt_server_characteristic_status.characteristic)
    {
      if(sl_bt_gatt_disable != (sl_bt_gatt_client_config_flag_t)evt->data.evt_gatt_server_characteristic_status.client_config_flags)
      {
        ble_data.motionIndicationSet = true;
        //gpioLed1SetOn();

      }
      else
      {
        ble_data.motionIndicationSet = false;
        //gpioLed1SetOff();
        //displayPrintf(DISPLAY_ROW_9, "");
      }

      // confirmation of indication received from remove GATT client
      if (sl_bt_gatt_server_confirmation == (sl_bt_gatt_server_characteristic_status_flag_t)evt->data.evt_gatt_server_characteristic_status.status_flags)
      {
        (void)evt->data.evt_gatt_server_characteristic_status.connection;
        ble_data.inFlight = false;
      }
    }
#endif
    break;

  case sl_bt_evt_gatt_server_indication_timeout_id:

    break;

  default:
    break;
  }

} // handle_ble_event()


/*******************************************************************************
 * @file ble.h
 * @brief Bluetooth event handler header file
 *******************************************************************************
 * @institution University of Colorado Boulder (UCB)
 * @course ECEN 5823-001: IoT Embedded Firmware (Fall 2020)
 * @instructor:  David Sluiter
 *
 * @copyright All rights reserved. Distribution allowed only for the
 * use of assignment grading. Use of code excerpts allowed at the
 * discretion of author. Contact for permission.
 * @assignment:  ecen5823-assignment6-PeterBraganza
 * @date:        1-10-2021
 * @author:      Peter Braganza
 * Description:  Contains macros for Int to float and uint to bitstream macros
 *               Contains structure for bluetooth data and other function
 *               prototypes
 ******************************************************************************/


#ifndef SRC_BLE_H_
#define SRC_BLE_H_

#include "sl_bt_api.h"
#include "gatt_db.h"
#include "ble_device_type.h"


#define UINT8_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); }
#define UINT32_TO_BITSTREAM(p, n) { *(p)++ = (uint8_t)(n); *(p)++ = (uint8_t)((n) >> 8); \
 *(p)++ = (uint8_t)((n) >> 16); *(p)++ = (uint8_t)((n) >> 24); }
#define UINT32_TO_FLOAT(m, e) (((uint32_t)(m) & 0x00FFFFFFU) | (uint32_t)((int32_t)(e) << 24))

//line35 to 38 is taken from SL sample thermometer client code
// Health Thermometer service UUID defined by Bluetooth SIG : 1809
static const uint8_t thermo_service[2] = { 0x09, 0x18 };
// Temperature Measurement characteristic UUID defined by Bluetooth SIG
static const uint8_t thermo_char[2] = { 0x1c, 0x2a };


//Button State service UUID 00000001-38c8-433e-87ec-652a2d136289
static const uint8_t button_state_service[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x01, 0x00, 0x00, 0x00 };

//Button State characteristic UUID 00000002-38c8-433e-87ec-652a2d136289
static const uint8_t button_state_char[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x02, 0x00, 0x00, 0x00 };

//Ultrasonic Sensor service distance violation count UUID 00000003-38c8-433e-87ec-652a2d136289
static const uint8_t distance_violation_state_service[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x03, 0x00, 0x00, 0x00 };

//Ultrasonic Sensor characteristic distance violation count UUID 00000004-38c8-433e-87ec-652a2d136289
static const uint8_t distance_violation_state_char[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x04, 0x00, 0x00, 0x00 };

//Motion Violation service distance violation count UUID 00000005-38c8-433e-87ec-652a2d136289
static const uint8_t motion_violation_state_service[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x05, 0x00, 0x00, 0x00 };

//Ultrasonic Sensor characteristic distance violation count UUID 00000006-38c8-433e-87ec-652a2d136289
static const uint8_t motion_violation_state_char[16] = { 0x89, 0x62, 0x13, 0x2d, 0x2a, 0x65, 0xec, 0x87, 0x3e, 0x43, 0xc8, 0x38, 0x06, 0x00, 0x00, 0x00 };

#define MAX_QUEUE_SIZE 18

#define SM_CONFIG_FLAGS 0x0F

// BLE Data Structure, save all of our private BT data in here.
// Modern C (circa 2021 does it this way)
// typedef ble_data_struct_t is referred to as an anonymous struct definition
typedef struct {
 // values that are common to servers and clients
  bd_addr myAddress;
  uint8_t myAddressType;
  bool    buttonState;    //probably change to PB0Pressed
  bool    passKeyConfirm;
  uint8_t distance_violation_count;
  uint8_t motion_violation_count;

  // values unique for server
  uint8_t advertisingSetHandle;
  uint8_t connectionHandle;
  bool    temperatureIndicationSet;
  bool    buttonIndicationSet;
  bool    distanceIndicationSet;
  bool    motionIndicationSet;
  bool    connectionOpen;
  bool    inFlight;
  bool    bonded;

  //need to change this later
  uint8_t count;

  // values unique for client
  bd_addr  serverAddress;
  uint32_t thermometer_service_handle;
  uint16_t thermometer_characteristic_handle;
  uint32_t button_state_service_handle;
  uint16_t button_state_characteristic_handle;
  uint32_t distance_state_service_handle;
  uint16_t distance_state_characteristic_handle;
  uint32_t motion_state_service_handle;
  uint16_t motion_state_characteristic_handle;


} ble_data_struct_t;

//structure for storing indication data in queue
typedef struct {
  uint16_t charHandle;     // char handle from gatt_db.h
  size_t bufferLength;     // length of buffer in bytes to send
  uint8_t buffer[5];       // the actual data buffer for the indication
                           // need space for HTM [5] and button_state [2] indications
} indication_queue_t;

//queue structure
typedef struct{
  indication_queue_t indication_queue[MAX_QUEUE_SIZE];
  int tail;
  int head;
  int queue_size;

} queue_t;



void                queue_init(queue_t *q);

void                enqueue(queue_t *q, indication_queue_t i);
indication_queue_t* dequeue(queue_t *q);


ble_data_struct_t*  getBleDataPtr(void);
queue_t*            getQueuePtr();
void                handle_ble_event(sl_bt_msg_t *evt);

#endif /* SRC_BLE_H_ */

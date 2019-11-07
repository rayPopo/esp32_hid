/*
HCI Reset complete
Write class of device
Local Bluetooth Address: 00:1B:DC:06:A2:E9
Gamepad is connecting
Gamepad found
HID device found
Connecting to HID device
Connected to HID device
Received Key Request
Bluetooth pin is set too: 0000
Pairing successful with HID device
Send HID Control Connection Request
Send HID Control Config Request
Set protocol mode: 00
Send HID Interrupt Connection Request
Send HID Interrupt Config Request
HID Channels Established
Wait For Incoming Connection Request
*/

#define DEBUG_HCI 1
#define DEBUG_ACL 1
#define DEBUG_USB_HOST 1
#define EXTRADEBUG 1

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt.h"

/* Macros for HCI event flag tests */
#define hci_check_flag(flag) (hci_event_flag & (flag))
#define hci_set_flag(flag) (hci_event_flag |= (flag))
#define hci_clear_flag(flag) (hci_event_flag &= ~(flag))

uint8_t classOfDevice[3];
char remote_name[30];

uint8_t hcibuf[BULK_MAXPKTSIZE];

uint8_t hci_state;
uint8_t hci_version = 0;
uint16_t hci_counter = 0;
uint16_t hci_event_flag = 0;
uint16_t hci_num_reset_loops = 100;
uint16_t hci_handle;
uint8_t inquiry_counter = 0;
uint8_t identifier = 0;

bool l2capConnectionClaimed = false;
bool incomingHIDDevice = false;
bool connectToHIDDevice = false;
bool pairWithHIDDevice = true;
bool waitingForConnection = false;
bool connected = false;
bool activeConnection = false;
bool readyToSend = false;

const char *btdName = NULL;
const char *btdPin = "0000";

uint8_t own_bdaddr[6];
uint8_t disc_bdaddr[6];

uint8_t l2cap_state = L2CAP_WAIT;
uint8_t l2cap_event_flag = 0;
uint8_t l2capinbuf[BULK_MAXPKTSIZE];
uint8_t control_scid[2];
uint8_t interrupt_scid[2];

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))

static int HCI_Packet_Task(uint8_t *, uint16_t);
static void HCI_Event_Task(uint8_t *, uint16_t);
static void ACL_Event_Task(uint8_t *, uint16_t);
static void HCI_Task();
static void L2CAP_Task();

static bool checkHciHandle(uint8_t *buf, uint16_t handle) {
  return (buf[0] == (handle & 0xFF)) && (buf[1] == ((handle >> 8) | 0x20));
}

static void controller_send_ready(void) {
  readyToSend = true;
  printf("Controller ready to send\n");
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_send_ready,
    HCI_Packet_Task
};

void HCI_Command(uint8_t *data, uint16_t nbytes) {
  if (!esp_vhci_host_check_send_available()) {
#ifdef DEBUG_HCI
    printf("Unable to send HCI Command\n");
#endif
    return;
  }
  hci_clear_flag(HCI_FLAG_CMD_COMPLETE);

  esp_vhci_host_send_packet(data, nbytes);
#ifdef DEBUG_HCI
  printf("HCI Command: Code 0x%x, Length %d (", data[1], nbytes);

  for (uint8_t i = 0; i < nbytes - 1; i++)
    printf("0x%x ", data[i]);

  printf("0x%x)\n", data[nbytes - 1]);
#endif
}

void hci_reset() {
  hci_event_flag = 0; // Clear all the flags
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x03;
  hcibuf[2] = 0x03 << 2;
  hcibuf[3] = 0x00;

  HCI_Command(hcibuf, 4);
}

void hci_write_class_of_device() { // See http://bluetooth-pentest.narod.ru/software/bluetooth_class_of_device-service_generator.html
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x24; // HCI OCF = 24
  hcibuf[2] = 0x03 << 2; // HCI OGF = 3
  hcibuf[3] = 0x03; // parameter length = 3
  hcibuf[4] = 0x04;
  hcibuf[5] = 0x08;
  hcibuf[6] = 0x00;

  HCI_Command(hcibuf, 7);
}

void hci_write_scan_enable() {
  hci_clear_flag(HCI_FLAG_INCOMING_REQUEST);
  hcibuf[0] = 0x1A; // HCI OCF = 1A
  hcibuf[1] = 0x03 << 2; // HCI OGF = 3
  hcibuf[2] = 0x01; // parameter length = 1

  if (btdName != NULL)
    hcibuf[3] = 0x03; // Inquiry Scan enabled. Page Scan enabled.
  else
    hcibuf[3] = 0x02; // Inquiry Scan disabled. Page Scan enabled.

  HCI_Command(hcibuf, 4);
}

void hci_write_scan_disable() {
  hcibuf[0] = 0x1A; // HCI OCF = 1A
  hcibuf[1] = 0x03 << 2; // HCI OGF = 3
  hcibuf[2] = 0x01; // parameter length = 1
  hcibuf[3] = 0x00; // Inquiry Scan disabled. Page Scan disabled.

  HCI_Command(hcibuf, 4);
}

void hci_read_bdaddr() {
  hci_clear_flag(HCI_FLAG_READ_BDADDR);
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x09; // HCI OCF = 9
  hcibuf[2] = 0x04 << 2; // HCI OGF = 4
  hcibuf[3] = 0x00;

  HCI_Command(hcibuf, 4);
}

void hci_read_local_version_information() {
  hci_clear_flag(HCI_FLAG_READ_VERSION);
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x01; // HCI OCF = 1
  hcibuf[2] = 0x04 << 2; // HCI OGF = 4
  hcibuf[3] = 0x00;

  HCI_Command(hcibuf, 4);
}

void hci_accept_connection() {
  hci_clear_flag(HCI_FLAG_CONNECT_COMPLETE);
  hcibuf[0] = 0x09; // HCI OCF = 9
  hcibuf[1] = 0x01 << 2; // HCI OGF = 1
  hcibuf[2] = 0x07; // parameter length 7
  hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
  hcibuf[4] = disc_bdaddr[1];
  hcibuf[5] = disc_bdaddr[2];
  hcibuf[6] = disc_bdaddr[3];
  hcibuf[7] = disc_bdaddr[4];
  hcibuf[8] = disc_bdaddr[5];
  hcibuf[9] = 0x00; // Switch role to master

  HCI_Command(hcibuf, 10);
}

void hci_remote_name() {
  hci_clear_flag(HCI_FLAG_REMOTE_NAME_COMPLETE);
  hcibuf[0] = 0x19; // HCI OCF = 19
  hcibuf[1] = 0x01 << 2; // HCI OGF = 1
  hcibuf[2] = 0x0A; // parameter length = 10
  hcibuf[3] = disc_bdaddr[0]; // 6 octet bdaddr
  hcibuf[4] = disc_bdaddr[1];
  hcibuf[5] = disc_bdaddr[2];
  hcibuf[6] = disc_bdaddr[3];
  hcibuf[7] = disc_bdaddr[4];
  hcibuf[8] = disc_bdaddr[5];
  hcibuf[9] = 0x01; // Page Scan Repetition Mode
  hcibuf[10] = 0x00; // Reserved
  hcibuf[11] = 0x00; // Clock offset - low byte
  hcibuf[12] = 0x00; // Clock offset - high byte

  HCI_Command(hcibuf, 13);
}

void hci_set_local_name(const char* name) {
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x13; // HCI OCF = 13
  hcibuf[2] = 0x03 << 2; // HCI OGF = 3
  hcibuf[3] = strlen(name) + 1; // parameter length = the length of the string + end byte

  uint8_t i;
  for (i = 0; i < strlen(name); i++)
    hcibuf[i + 4] = name[i];

  hcibuf[i + 4] = 0x00; // End of string

  HCI_Command(hcibuf, 5 + strlen(name));
}

void hci_inquiry() {
  hci_clear_flag(HCI_FLAG_DEVICE_FOUND);
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x01;
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x05; // Parameter Total Length = 5
  hcibuf[4] = 0x33; // LAP: Genera/Unlimited Inquiry Access Code (GIAC = 0x9E8B33) - see https://www.bluetooth.org/Technical/AssignedNumbers/baseband.htm
  hcibuf[5] = 0x8B;
  hcibuf[6] = 0x9E;
  hcibuf[7] = 0x30; // Inquiry time = 61.44 sec (maximum)
  hcibuf[8] = 0x0A; // 10 number of responses

  HCI_Command(hcibuf, 9);
}

void hci_inquiry_cancel() {
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x02;
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x00; // Parameter Total Length = 0

  HCI_Command(hcibuf, 4);
}

/*
static void hci_connect() {
  hci_connect(disc_bdaddr); // Use last discovered device
}
*/

void hci_connect(uint8_t *bdaddr) {
  hci_clear_flag(HCI_FLAG_CONNECT_COMPLETE | HCI_FLAG_CONNECT_EVENT);
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x05;
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x0D; // parameter Total Length = 13
  hcibuf[4] = disc_bdaddr[0]; // 6 octet bdaddr (LSB)
  hcibuf[5] = disc_bdaddr[1];
  hcibuf[6] = disc_bdaddr[2];
  hcibuf[7] = disc_bdaddr[3];
  hcibuf[8] = disc_bdaddr[4];
  hcibuf[9] = disc_bdaddr[5];
  hcibuf[10] = 0x18; // DM1 or DH1 may be used
  hcibuf[11] = 0xCC; // DM3, DH3, DM5, DH5 may be used
  hcibuf[12] = 0x01; // Page repetition mode R1
  hcibuf[13] = 0x00; // Reserved
  hcibuf[14] = 0x00; // Clock offset
  hcibuf[15] = 0x00; // Invalid clock offset
  hcibuf[16] = 0x01; // Do not allow role switch

  HCI_Command(hcibuf, 17);
}

void hci_pin_code_request_reply() {
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x0D; // HCI OCF = 0D
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x17; // parameter length 23
  hcibuf[4] = disc_bdaddr[0]; // 6 octet bdaddr
  hcibuf[5] = disc_bdaddr[1];
  hcibuf[6] = disc_bdaddr[2];
  hcibuf[7] = disc_bdaddr[3];
  hcibuf[8] = disc_bdaddr[4];
  hcibuf[9] = disc_bdaddr[5];

  hcibuf[10] = strlen(btdPin); // Length of pin
  uint8_t i;

  for (i = 0; i < strlen(btdPin); i++) // The maximum size of the pin is 16
    hcibuf[i + 11] = btdPin[i];

  for (; i < 16; i++)
    hcibuf[i + 11] = 0x00; // The rest should be 0

  HCI_Command(hcibuf, 27);
}

void hci_pin_code_negative_request_reply() {
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x0E; // HCI OCF = 0E
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x06; // parameter length 6
  hcibuf[4] = disc_bdaddr[0]; // 6 octet bdaddr
  hcibuf[5] = disc_bdaddr[1];
  hcibuf[6] = disc_bdaddr[2];
  hcibuf[7] = disc_bdaddr[3];
  hcibuf[8] = disc_bdaddr[4];
  hcibuf[9] = disc_bdaddr[5];

  HCI_Command(hcibuf, 10);
}

void hci_link_key_request_negative_reply() {
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x0C; // HCI OCF = 0C
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x06; // parameter length 6
  hcibuf[4] = disc_bdaddr[0]; // 6 octet bdaddr
  hcibuf[5] = disc_bdaddr[1];
  hcibuf[6] = disc_bdaddr[2];
  hcibuf[7] = disc_bdaddr[3];
  hcibuf[8] = disc_bdaddr[4];
  hcibuf[9] = disc_bdaddr[5];

  HCI_Command(hcibuf, 10);
}

void hci_authentication_request() {
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x11; // HCI OCF = 11
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x02; // parameter length = 2
  hcibuf[4] = (uint8_t)(hci_handle & 0xFF); //connection handle - low byte
  hcibuf[5] = (uint8_t)((hci_handle >> 8) & 0x0F); //connection handle - high byte

  HCI_Command(hcibuf, 6);
}

void hci_disconnect(uint16_t handle) { // This is called by the different services
  hci_clear_flag(HCI_FLAG_DISCONNECT_COMPLETE);
  hcibuf[0] = HCIT_TYPE_COMMAND;
  hcibuf[1] = 0x06; // HCI OCF = 6
  hcibuf[2] = 0x01 << 2; // HCI OGF = 1
  hcibuf[3] = 0x03; // parameter length = 3
  hcibuf[4] = (uint8_t)(handle & 0xFF); //connection handle - low byte
  hcibuf[5] = (uint8_t)((handle >> 8) & 0x0F); //connection handle - high byte
  hcibuf[6] = 0x13; // reason

  HCI_Command(hcibuf, 7);
}

void l2cap_connection_request() {
  hcibuf[0] = HCIT_TYPE_ACL_DATA;
  hcibuf[1] = (uint8_t)(hci_handle & 0xFF); // HCI handle with PB,BC flag
  hcibuf[2] = (uint8_t)(((hci_handle >> 8) & 0x0f) | 0x20);
  hcibuf[3] = (uint8_t)(12 & 0xFF); // HCI ACL total data length
  hcibuf[4] = (uint8_t)(12 >> 8);
  hcibuf[5] = (uint8_t)(8 & 0xFF); // L2CAP header: Length
  hcibuf[6] = (uint8_t)(8 >> 8);
  hcibuf[7] = (uint8_t)(1 & 0xFF);
  hcibuf[8] = (uint8_t)(1 >> 8);
  hcibuf[9] = L2CAP_CMD_CONNECTION_REQUEST; // Code
  hcibuf[10] = 0x01; // Identifier
  hcibuf[11] = 0x04; // Length
  hcibuf[12] = 0x00;
  hcibuf[13] = (uint8_t)(0x11 & 0xFF); // PSM
  hcibuf[14] = (uint8_t)(0x11 >> 8);
  hcibuf[15] = (uint8_t)(0x40 & 0xFF); // Source CID
  hcibuf[16] = (uint8_t)(0x40 >> 8);

  HCI_Command(hcibuf, 17);
}

void l2cap_config_request() {
  hcibuf[0] = HCIT_TYPE_ACL_DATA;
  hcibuf[1] = (uint8_t)(hci_handle & 0xFF); // HCI handle with PB,BC flag
  hcibuf[2] = (uint8_t)(((hci_handle >> 8) & 0x0f) | 0x20);
  hcibuf[3] = (uint8_t)(16 & 0xFF); // HCI ACL total data length
  hcibuf[4] = (uint8_t)(16 >> 8);
  hcibuf[5] = (uint8_t)(12 & 0xFF); // L2CAP header: Length
  hcibuf[6] = (uint8_t)(12 >> 8);
  hcibuf[7] = (uint8_t)(1 & 0xFF);
  hcibuf[8] = (uint8_t)(1 >> 8);
  hcibuf[9] = L2CAP_CMD_CONFIG_REQUEST; // Code
  hcibuf[10] = 0x01; // Identifier
  hcibuf[11] = 0x08; // Length
  hcibuf[12] = 0x00;
  hcibuf[13] = 0x70;
  hcibuf[14] = 0x00;
  hcibuf[15] = 0x00; // Flags
  hcibuf[16] = 0x00;
  hcibuf[17] = 0x01; // Config Opt: type = MTU (Maximum Transmission Unit) - Hint
  hcibuf[18] = 0x02; // Config Opt: length
  hcibuf[19] = 0xFF; // MTU
  hcibuf[20] = 0xFF;

  HCI_Command(hcibuf, 21);
}

void l2cap_connection_response(uint8_t result) {
  hcibuf[0] = HCIT_TYPE_ACL_DATA;
  hcibuf[1] = (uint8_t)(hci_handle & 0xFF); // HCI handle with PB,BC flag
  hcibuf[2] = (uint8_t)(((hci_handle >> 8) & 0x0f) | 0x20);
  hcibuf[3] = (uint8_t)(16 & 0xFF); // HCI ACL total data length
  hcibuf[4] = (uint8_t)(16 >> 8);
  hcibuf[5] = (uint8_t)(12 & 0xFF); // L2CAP header: Length
  hcibuf[6] = (uint8_t)(12 >> 8);
  hcibuf[7] = (uint8_t)(1 & 0xFF);
  hcibuf[8] = (uint8_t)(1 >> 8);
  hcibuf[9] = L2CAP_CMD_CONNECTION_RESPONSE; // Code
  hcibuf[10] = 0x01; // Identifier
  hcibuf[11] = 0x08; // Length
  hcibuf[12] = 0x00;
  hcibuf[13] = 0x40; // Destination CID
  hcibuf[14] = 0x00;
  hcibuf[13] = (uint8_t)(control_scid[0] & 0xFF); // Source CID
  hcibuf[14] = (uint8_t)(control_scid[1] >> 8);
  hcibuf[17] = result; // Result: Pending or Success
  hcibuf[18] = 0x00;
  hcibuf[19] = 0x00; // No further information
  hcibuf[20] = 0x00;

  HCI_Command(hcibuf, 21);
}

void l2cap_config_response() {
  hcibuf[0] = HCIT_TYPE_ACL_DATA;
  hcibuf[1] = (uint8_t)(hci_handle & 0xFF); // HCI handle with PB,BC flag
  hcibuf[2] = (uint8_t)(((hci_handle >> 8) & 0x0f) | 0x20);
  hcibuf[3] = (uint8_t)(18 & 0xFF); // HCI ACL total data length
  hcibuf[4] = (uint8_t)(18 >> 8);
  hcibuf[5] = (uint8_t)(14 & 0xFF); // L2CAP header: Length
  hcibuf[6] = (uint8_t)(14 >> 8);
  hcibuf[7] = (uint8_t)(1 & 0xFF);
  hcibuf[8] = (uint8_t)(1 >> 8);
  hcibuf[9] = L2CAP_CMD_CONFIG_RESPONSE; // Code
  hcibuf[10] = 0x01; // Identifier
  hcibuf[11] = 0x0A; // Length
  hcibuf[12] = 0x00;
  hcibuf[13] = (uint8_t)(control_scid[0] & 0xFF); // Source CID
  hcibuf[14] = (uint8_t)(control_scid[1] >> 8);
  hcibuf[15] = 0x00; // Flag
  hcibuf[16] = 0x00;
  hcibuf[17] = 0x00; // Result
  hcibuf[18] = 0x00;
  hcibuf[19] = 0x01; // Config
  hcibuf[20] = 0x02;
  hcibuf[21] = 0xA0;
  hcibuf[22] = 0x02;

  HCI_Command(hcibuf, 23);
}

void l2cap_disconnection_request() {
  hcibuf[0] = HCIT_TYPE_ACL_DATA;
  hcibuf[1] = (uint8_t)(hci_handle & 0xFF); // HCI handle with PB,BC flag
  hcibuf[2] = (uint8_t)(((hci_handle >> 8) & 0x0f) | 0x20);
  hcibuf[3] = (uint8_t)(12 & 0xFF); // HCI ACL total data length
  hcibuf[4] = (uint8_t)(12 >> 8);
  hcibuf[5] = (uint8_t)(8 & 0xFF); // L2CAP header: Length
  hcibuf[6] = (uint8_t)(8 >> 8);
  hcibuf[7] = (uint8_t)(1 & 0xFF);
  hcibuf[8] = (uint8_t)(1 >> 8);
  hcibuf[9] = L2CAP_CMD_DISCONNECT_REQUEST; // Code
  hcibuf[10] = 0x01; // Identifier
  hcibuf[11] = 0x04; // Length
  hcibuf[12] = 0x00;
  hcibuf[13] = (uint8_t)(0x11 & 0xFF); // PSM
  hcibuf[14] = (uint8_t)(0x11 >> 8);
  hcibuf[15] = (uint8_t)(0x40 & 0xFF); // Source CID
  hcibuf[16] = (uint8_t)(0x40 >> 8);

  HCI_Command(hcibuf, 17);
}

void l2cap_disconnection_response() {
  hcibuf[0] = HCIT_TYPE_ACL_DATA;
  hcibuf[1] = (uint8_t)(hci_handle & 0xFF); // HCI handle with PB,BC flag
  hcibuf[2] = (uint8_t)(((hci_handle >> 8) & 0x0f) | 0x20);
  hcibuf[3] = (uint8_t)(12 & 0xFF); // HCI ACL total data length
  hcibuf[4] = (uint8_t)(12 >> 8);
  hcibuf[5] = (uint8_t)(8 & 0xFF); // L2CAP header: Length
  hcibuf[6] = (uint8_t)(8 >> 8);
  hcibuf[7] = (uint8_t)(1 & 0xFF);
  hcibuf[8] = (uint8_t)(1 >> 8);
  hcibuf[9] = L2CAP_CMD_DISCONNECT_RESPONSE; // Code
  hcibuf[10] = 0x01; // Identifier
  hcibuf[11] = 0x04; // Length
  hcibuf[12] = 0x00;
  hcibuf[13] = (uint8_t)(0x11 & 0xFF); // PSM
  hcibuf[14] = (uint8_t)(0x11 >> 8);
  hcibuf[15] = (uint8_t)(0x40 & 0xFF); // Source CID
  hcibuf[16] = (uint8_t)(0x40 >> 8);

  HCI_Command(hcibuf, 17);
}

static void l2cap_reset() {
  connected = false;
  activeConnection = false;
  l2cap_event_flag = 0; // Reset flags
  l2cap_state = L2CAP_WAIT;
}

/* Incoming HCI Packet */
static int HCI_Packet_Task(uint8_t *buf, uint16_t length) {
  switch (buf[0]) {
    case HCIT_TYPE_EVENT:
      HCI_Event_Task(++buf, length - 1);
      break;

    case HCIT_TYPE_ACL_DATA:
      //ACL_Event_Task(++buf, length - 1);
      break;

    default:
#ifdef DEBUG_HCI
      printf("Unmanaged HCI Packet Type: 0x%x\n", buf[0]);
#endif
      break;
  }

  return 0;
}

static void ACL_Event_Task(uint8_t *buf, uint16_t length) {
#ifdef DEBUG_ACL
  printf("ACL Event Code 0x%x Data: ", buf[8]);

  for (uint8_t i = 0; i < length; i++)
    printf("0x%x ", buf[i]);

  printf("\n");
#endif

  if (!l2capConnectionClaimed && incomingHIDDevice && !connected && !activeConnection) {
    if (buf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
#ifdef DEBUG_HCI
      printf("Incoming cmd connection request: 0x%x\n", buf[12]);
#endif
      if ((buf[12] | (buf[13] << 8)) == 0x11) {
        incomingHIDDevice = false;
        l2capConnectionClaimed = true; // Claim that the incoming connection belongs to this service
        activeConnection = true;
        l2cap_state = L2CAP_WAIT;
#ifdef EXTRADEBUG
        printf("L2CAP Connection claimed\n");
#endif
      }
    }
  }

  if (checkHciHandle(buf, hci_handle)) { // acl_handle_ok
    if ((buf[6] | (buf[7] << 8)) == 0x0001U) { // l2cap_control - Channel ID for ACL-U
      if (buf[8] == L2CAP_CMD_COMMAND_REJECT) {
#ifdef DEBUG_USB_HOST
        printf("L2CAP Command Rejected - Reason: 0x%x 0x%x 0x%x 0x%x 0x%x 0x%x\n", buf[13], buf[12], buf[17], buf[16], buf[15], buf[14]);
#endif
      } else if (buf[8] == L2CAP_CMD_CONNECTION_RESPONSE) {
        if (((buf[16] | (buf[17] << 8)) == 0x0000) && ((buf[18] | (buf[19] << 8)) == SUCCESSFUL)) { // Success
          if (buf[14] == 0x40 && buf[15] == 0x00) {
            printf("HID Control Connection Complete\n");
            identifier = buf[9];
            control_scid[0] = buf[12];
            control_scid[1] = buf[13];
            l2cap_set_flag(L2CAP_FLAG_CONTROL_CONNECTED);
          } else if (buf[14] == 0x41 && buf[15] == 0x00) {
            printf("HID Interrupt Connection Complete\n");
            identifier = buf[9];
            interrupt_scid[0] = buf[12];
            interrupt_scid[1] = buf[13];
            l2cap_set_flag(L2CAP_FLAG_INTERRUPT_CONNECTED);
          }
        }
      } else if (buf[8] == L2CAP_CMD_CONNECTION_REQUEST) {
#ifdef EXTRADEBUG
        printf("L2CAP Connection Request - PSM: 0x%x 0x%x SCID: 0x%x 0x%x Identidifier: 0x%x\n", buf[13], buf[12], buf[15], buf[14], buf[9]);
#endif
        if ((buf[12] | (buf[13] << 8)) == 0x11) {
          identifier = buf[9];
          control_scid[0] = buf[14];
          control_scid[1] = buf[15];
          l2cap_set_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST);
        } else if ((buf[12] | (buf[13] << 8)) == 0x13) {
          identifier = buf[9];
          interrupt_scid[0] = buf[14];
          interrupt_scid[1] = buf[15];
          l2cap_set_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST);
        }
      } else if (buf[8] == L2CAP_CMD_CONFIG_RESPONSE) {
        if ((buf[16] | (buf[17] << 8)) == 0x0000) { // Success
          if(buf[12] == 0x40 && buf[13] == 0x00) {
            printf("HID Control Configuration Complete\n");
            identifier = buf[9];
            l2cap_set_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS);
          } else if (buf[12] == 0x41 && buf[13] == 0x00) {
            printf("HID Interrupt Configuration Complete\n");
            identifier = buf[9];
            l2cap_set_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS);
          }
        }
      } else if (buf[8] == L2CAP_CMD_CONFIG_REQUEST) {
        if (buf[12] == 0x40 && buf[13] == 0x00) {
          printf("HID Control Configuration Request\n");
          l2cap_config_response();
        } else if (buf[12] == 0x41 && buf[13] == 0x00) {
          printf("HID Interrupt Configuration Request\n");
          l2cap_config_response();
        }
      } else if (buf[8] == L2CAP_CMD_DISCONNECT_REQUEST) {
        if (buf[12] == 0x40 && l2capinbuf[13] == 0x00) {
#ifdef DEBUG_USB_HOST
          printf("Disconnect Request: Control Channel\n");
#endif
          identifier = buf[9];
          l2cap_disconnection_response();
          l2cap_reset();
        } else if (buf[12] == 0x41 && buf[13] == 0x00) {
#ifdef DEBUG_USB_HOST
          printf("Disconnect Request: Interrupt Channel\n");
#endif
          identifier = buf[9];
          l2cap_disconnection_response();
          l2cap_reset();
        }
      } else if (buf[8] == L2CAP_CMD_DISCONNECT_RESPONSE) {
        if (buf[12] == 0x40 && buf[13] == 0x00) {
          printf("Disconnect Response: Control Channel\n");
          identifier = buf[9];
          l2cap_set_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE);
        } else if(l2capinbuf[12] == 0x41 && l2capinbuf[13] == 0x00) {
          printf("Disconnect Response: Interrupt Channel\n");
          identifier = buf[9];
          l2cap_set_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE);
        }
      } else {
#ifdef EXTRADEBUG

        identifier = buf[9];
        printf("L2CAP Unknown Signaling Command: 0x%x\n", buf[8]);

#endif
      }
    } else if (buf[6] == 0x41 && buf[7] == 0x00) { // l2cap_interrupt
#ifdef PRINTREPORT
      printf("L2CAP Interrupt: ");

      for (uint16_t i = 0; i < ((uint16_t)buf[5] << 8 | buf[4]); i++) {
              printf("0x%x ", buf[i + 8]);
      }

      printf("\n");
#endif
      if (buf[8] == 0xA1) { // HID_THDR_DATA_INPUT
        uint16_t length = ((uint16_t)buf[5] << 8 | buf[4]);
        //ParseBTHIDData((uint8_t)(length - 1), &buf[9]);
      }
    }
  } else if (buf[6] == 0X40 && buf[7] == 0X00) { // l2cap_control
#ifdef PRINTREPORT
    printf("L2CAP Control: ");

    for (uint16_t i = 0; i < ((uint16_t)buf[5] << 8 | buf[4]); i++) {
      printf("0x%x ", buf[i + 8]);
    }

    printf("\n");
#endif
  }
#ifdef EXTRADEBUG
  else {
    printf("Unsupported L2CAP Data - Channel ID: 0x%x 0x%x Data: ", buf[7], buf[6]);

    for (uint16_t i = 0; i < ((uint16_t)buf[5] << 8 | buf[4]); i++) {
            printf("0x%x ", buf[i + 8]);
    }

    printf("\n");
  }
#endif

  L2CAP_Task();

  switch (l2cap_state) {
    case L2CAP_WAIT:
#ifdef EXTRADEBUG
      printf("L2CAP_WAIT: connectToHIDDevice (%d), l2capConnectionClaimed (%d), connected (%d), activeConnection (%d)", connectToHIDDevice, l2capConnectionClaimed, connected, activeConnection);
#endif
      if (connectToHIDDevice && !l2capConnectionClaimed && !connected && !activeConnection) {
        l2capConnectionClaimed = true;
        activeConnection = true;
#ifdef DEBUG_USB_HOST
        printf("Send HID Control Connection Request\n");
#endif
        l2cap_event_flag = 0; // Reset flags
        identifier = 0;
        l2cap_connection_request();
        l2cap_state = L2CAP_CONTROL_CONNECT_REQUEST;
      } else if (l2cap_check_flag(L2CAP_FLAG_CONNECTION_CONTROL_REQUEST)) {
#ifdef DEBUG_USB_HOST
        printf("HID Control Incoming Connection Request\n");
#endif
        l2cap_connection_response(PENDING);
        vTaskDelay(1 / portTICK_PERIOD_MS);
        l2cap_connection_response(SUCCESSFUL);
        identifier++;
        vTaskDelay(1 / portTICK_PERIOD_MS);
        l2cap_config_request();
        l2cap_state = L2CAP_CONTROL_SUCCESS;
      }
      break;
  }
}

static void L2CAP_Task() {
  switch (l2cap_state) {
    /* These states are used if the HID device is the host */
    case L2CAP_CONTROL_SUCCESS:
      if (l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {
#ifdef DEBUG_USB_HOST
      printf("HID Control Successfully Configured\n");
#endif
      //setProtocol(); // Set protocol before establishing HID interrupt channel
      l2cap_state = L2CAP_INTERRUPT_SETUP;
    }
    break;

  case L2CAP_INTERRUPT_SETUP:
    if (l2cap_check_flag(L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST)) {
#ifdef DEBUG_USB_HOST
      printf("HID Interrupt Incoming Connection Request\n");
#endif
      l2cap_connection_response(PENDING);
      vTaskDelay(1 / portTICK_PERIOD_MS);
      l2cap_connection_response(SUCCESSFUL);
      identifier++;
      vTaskDelay(1 / portTICK_PERIOD_MS);
      l2cap_config_request();

      l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
    }
    break;

  /* These states are used if the Arduino is the host */
  case L2CAP_CONTROL_CONNECT_REQUEST:
    if (l2cap_check_flag(L2CAP_FLAG_CONTROL_CONNECTED)) {
#ifdef DEBUG_USB_HOST
      printf("Send HID Control Config Request\n");
#endif
      identifier++;
      l2cap_config_request();
      l2cap_state = L2CAP_CONTROL_CONFIG_REQUEST;
    }
    break;

  case L2CAP_CONTROL_CONFIG_REQUEST:
    if (l2cap_check_flag(L2CAP_FLAG_CONFIG_CONTROL_SUCCESS)) {
      //setProtocol(); // Set protocol before establishing HID interrupt channel
      vTaskDelay(1 / portTICK_PERIOD_MS); // Short delay between commands - just to be sure
#ifdef DEBUG_USB_HOST
    printf("Send HID Interrupt Connection Request\n");
#endif
      identifier++;
      l2cap_connection_request();
      l2cap_state = L2CAP_INTERRUPT_CONNECT_REQUEST;
    }
    break;

    case L2CAP_INTERRUPT_CONNECT_REQUEST:
      if (l2cap_check_flag(L2CAP_FLAG_INTERRUPT_CONNECTED)) {
#ifdef DEBUG_USB_HOST
        printf("Send HID Interrupt Config Request\n");
#endif
        identifier++;
        l2cap_config_request();
        l2cap_state = L2CAP_INTERRUPT_CONFIG_REQUEST;
      }
      break;

    case L2CAP_INTERRUPT_CONFIG_REQUEST:
      if (l2cap_check_flag(L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS)) { // Now the HID channels is established
#ifdef DEBUG_USB_HOST
        printf("HID Channels Established\n");
#endif
        connectToHIDDevice = false;
        pairWithHIDDevice = false;
        connected = true;
        //onInit();
        l2cap_state = L2CAP_DONE;
      }
      break;

    case L2CAP_DONE:
      break;

    case L2CAP_INTERRUPT_DISCONNECT:
      if (l2cap_check_flag(L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE)) {
#ifdef DEBUG_USB_HOST
        printf("Disconnected Interrupt Channel\n");
#endif
        identifier++;
        l2cap_disconnection_request();
        l2cap_state = L2CAP_CONTROL_DISCONNECT;
      }
      break;

    case L2CAP_CONTROL_DISCONNECT:
      if (l2cap_check_flag(L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE)) {
#ifdef DEBUG_USB_HOST
        printf("Disconnected Control Channel\n");
#endif
        hci_disconnect(hci_handle);
        hci_handle = -1; // Reset handle
        l2cap_event_flag = 0; // Reset flags
        l2cap_state = L2CAP_WAIT;
      }
      break;
  }
}

/* HCI Event Task */
static void HCI_Event_Task(uint8_t *buf, uint16_t length) {
#ifdef DEBUG_HCI
  printf("HCI Event Code 0x%x, Length: %d, Content: ", buf[0], length);

  for (uint8_t i = 0; i < length - 1; i++)
    printf("0x%x ", buf[i]);

  printf("0x%x\n", buf[length - 1]);
#endif

  switch (buf[0]) { // Switch on event type
    case EV_COMMAND_COMPLETE:
#ifdef EXTRADEBUG
      printf("HCI Command Complete Status 0x%x\n", buf[5]);
#endif
      if (!buf[5]) { // Check if command succeeded
        hci_set_flag(HCI_FLAG_CMD_COMPLETE); // Set command complete flag

        if ((buf[3] == 0x09) && (buf[4] == 0x10)) { // Parameters from read local bluetooth address
          for (uint8_t i = 0; i < 6; i++)
            own_bdaddr[i] = buf[6 + i];

          hci_set_flag(HCI_FLAG_READ_BDADDR);
        } else if ((buf[3] == 0x01) && (buf[4] == 0x10)) { // Parameters from read local version information
#ifdef EXTRADEBUG
          printf("Setting hci_version: 0x%x\n", buf[6]);
#endif
          hci_version = buf[6]; // Used to check if it supports 2.0+EDR - see http://www.bluetooth.org/Technical/AssignedNumbers/hci.htm
          hci_set_flag(HCI_FLAG_READ_VERSION);
        }
      }
      break;

    case EV_COMMAND_STATUS:
      if (buf[2]) { // Show status on serial if not OK
#ifdef DEBUG_USB_HOST
        printf("HCI Command Failed: 0x%x\n", buf[2]);
#endif
      }
      break;

    case EV_INQUIRY_COMPLETE:
      if (inquiry_counter >= 5) {
        inquiry_counter = 0;
#ifdef DEBUG_USB_HOST
        printf("Couldn't find HID device\n");
#endif

        hci_state = HCI_SCANNING_STATE;
      }

      inquiry_counter++;
      break;

    case EV_INQUIRY_RESULT:
      if (buf[2]) { // Check that there is more than zero responses

#ifdef EXTRADEBUG
        printf("Number of responses: %d\n", buf[2]);
#endif
        for (uint8_t i = 0; i < buf[2]; i++) {
          uint8_t offset = 8 * buf[2] + 3 * i;

          for (uint8_t j = 0; j < 3; j++)
            classOfDevice[j] = buf[j + 4 + offset];

#ifdef EXTRADEBUG
          printf("Class of device: 0x%x 0x%x 0x%x\n", classOfDevice[2], classOfDevice[1], classOfDevice[0]);
#endif

          if ((classOfDevice[1] & 0x05) && (classOfDevice[0] & 0xC8)) {
#ifdef DEBUG_USB_HOST
            if (classOfDevice[0] & 0x80)
              printf("Mouse found: ");
            if (classOfDevice[0] & 0x40)
              printf("Keyboard found:");
            if (classOfDevice[0] & 0x08)
              printf("Gamepad found: ");
#endif

            for (uint8_t j = 0; j < 6; j++)
              disc_bdaddr[j] = buf[j + 3 + 6 * i];

#ifdef DEBUG_USB_HOST
            for (uint8_t i = 0; i < 5; i++)
              printf("%x:", disc_bdaddr[i]);

            printf("%x\n", disc_bdaddr[5]);
#endif

            hci_set_flag(HCI_FLAG_DEVICE_FOUND);
            break;
          }
        }
      }
      break;

    case EV_CONNECT_COMPLETE:
      hci_set_flag(HCI_FLAG_CONNECT_EVENT);

      if (!buf[2]) { // Check if connected OK
#ifdef EXTRADEBUG
        printf("Connection established\n");
#endif
        hci_handle = buf[3] | ((buf[4] & 0x0F) << 8); // Store the handle for the ACL connection
        hci_set_flag(HCI_FLAG_CONNECT_COMPLETE); // Set connection complete flag
      } else {
        hci_state = HCI_CHECK_DEVICE_SERVICE;
#ifdef DEBUG_USB_HOST
        printf("Connection Failed: 0x%x\n", buf[2]);
#endif
      }
      break;

    case EV_DISCONNECT_COMPLETE:
      if (!buf[2]) { // Check if disconnected OK
        hci_set_flag(HCI_FLAG_DISCONNECT_COMPLETE); // Set disconnect command complete flag
        hci_clear_flag(HCI_FLAG_CONNECT_COMPLETE); // Clear connection complete flag
      }
      break;

    case EV_REMOTE_NAME_COMPLETE:
      if (!buf[2]) { // Check if reading is OK
        for(uint8_t i = 0; i < min(sizeof(remote_name), sizeof(buf) - 9); i++) {
          remote_name[i] = buf[9 + i];

          if (remote_name[i] == '\0') // End of string
            break;
        }

        // TODO: Altid sæt '\0' i remote name!
        hci_set_flag(HCI_FLAG_REMOTE_NAME_COMPLETE);
      }
      break;

    case EV_INCOMING_CONNECT:
      for (uint8_t i = 0; i < 6; i++)
        disc_bdaddr[i] = buf[i + 2];

      for (uint8_t i = 0; i < 3; i++)
        classOfDevice[i] = buf[i + 8];

      if ((classOfDevice[1] & 0x05) && (classOfDevice[0] & 0xC8)) { // Check if it is a mouse, keyboard or a gamepad
#ifdef DEBUG_USB_HOST
        if (classOfDevice[0] & 0x80)
          printf("Mouse is connecting\n");
        if (classOfDevice[0] & 0x40)
          printf("Keyboard is connecting\n");
        if (classOfDevice[0] & 0x08)
          printf("Gamepad is connecting\n");
#endif

        incomingHIDDevice = true;
      }

#ifdef EXTRADEBUG
      printf("Class of device: 0x%x 0x%x 0x%x\n", classOfDevice[2], classOfDevice[1], classOfDevice[0]);
#endif
      hci_set_flag(HCI_FLAG_INCOMING_REQUEST);
      break;

    case EV_PIN_CODE_REQUEST:
      if (btdPin != NULL) {
#ifdef DEBUG_USB_HOST
        printf("Bluetooth pin is set to: %s\n", btdPin);
#endif
        hci_pin_code_request_reply();
      } else {
#ifdef DEBUG_USB_HOST
        printf("No pin was set\n");
#endif
        hci_pin_code_negative_request_reply();
      }
      break;

    case EV_LINK_KEY_REQUEST:
#ifdef DEBUG_USB_HOST
      printf("Received Key Request\n");
#endif
      hci_link_key_request_negative_reply();
      break;

    case EV_AUTHENTICATION_COMPLETE:
      if (!connectToHIDDevice) {
#ifdef DEBUG_USB_HOST
        printf("Pairing successful with HID device\n");
#endif
        connectToHIDDevice = true; // Used to indicate to the BTHID service, that it should connect to this device
      } else {
#ifdef DEBUG_USB_HOST
        printf("Pairing Failed: 0x%x\n", buf[2]);
#endif
        hci_disconnect(hci_handle);
        hci_state = HCI_DISCONNECT_STATE;
      }
      break;

    case EV_NUM_COMPLETE_PKT:
    case EV_ROLE_CHANGED:
    case EV_PAGE_SCAN_REP_MODE:
    case EV_LOOPBACK_COMMAND:
    case EV_DATA_BUFFER_OVERFLOW:
    case EV_CHANGE_CONNECTION_LINK:
    case EV_MAX_SLOTS_CHANGE:
    case EV_QOS_SETUP_COMPLETE:
    case EV_LINK_KEY_NOTIFICATION:
    case EV_ENCRYPTION_CHANGE:
    case EV_READ_REMOTE_VERSION_INFORMATION_COMPLETE:
      break;

#ifdef EXTRADEBUG
    default:
      if (buf[0] != 0x00) {
        printf("Unmanaged HCI Event: 0x%x", hcibuf[0]);
      }
      break;
#endif
  }
}

void mainTask(void *pvParameters) {
  while (1) {
    HCI_Task();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  return;
}

static void HCI_Task() {
  esp_vhci_host_register_callback(&vhci_host_cb);

  switch (hci_state) {
    case HCI_INIT_STATE:
      hci_counter++;

      if (hci_counter > hci_num_reset_loops) { // wait until we have looped x times to clear any old events
#ifdef DEBUG_USB_HOST
        printf("Resetting HCI State\n");
#endif
        hci_reset();
        hci_state = HCI_RESET_STATE;
        hci_counter = 0;
      }
      break;

    case HCI_RESET_STATE:
      hci_counter++;

      if (hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
        hci_counter = 0;

#ifdef DEBUG_USB_HOST
        printf("HCI Reset complete\n");
#endif

        hci_state = HCI_CLASS_STATE;
        hci_write_class_of_device();
      } else if (hci_counter > hci_num_reset_loops) {
        hci_num_reset_loops *= 10;

        if (hci_num_reset_loops > 2000)
          hci_num_reset_loops = 2000;

#ifdef DEBUG_USB_HOST
        printf("No response to HCI Reset\n");
#endif
        hci_state = HCI_INIT_STATE;
        hci_counter = 0;
      }
      break;

    case HCI_CLASS_STATE:
      if (hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
#ifdef DEBUG_USB_HOST
        printf("Write class of device\n");
#endif
        hci_state = HCI_BDADDR_STATE;
        hci_read_bdaddr();
      }
      break;

    case HCI_BDADDR_STATE:
      if (hci_check_flag(HCI_FLAG_READ_BDADDR)) {
#ifdef DEBUG_USB_HOST
        printf("Local Bluetooth Address: ");

        for (int8_t i = 5; i > 0; i--) {
          printf("%x:", own_bdaddr[i]);
        }

        printf("%x\n", own_bdaddr[0]);
#endif
        hci_read_local_version_information();
        hci_state = HCI_LOCAL_VERSION_STATE;
      }
      break;

    case HCI_LOCAL_VERSION_STATE: // The local version is used by the PS3BT class
      if (hci_check_flag(HCI_FLAG_READ_VERSION)) {
        if (btdName != NULL) {
          hci_set_local_name(btdName);
          hci_state = HCI_SET_NAME_STATE;
        } else {
          hci_state = HCI_CHECK_DEVICE_SERVICE;
        }
      }
      break;

    case HCI_SET_NAME_STATE:
      if (hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
#ifdef DEBUG_USB_HOST
        printf("The name is set to: %s\n", btdName);
#endif
        hci_state = HCI_CHECK_DEVICE_SERVICE;
      }
      break;

    case HCI_CHECK_DEVICE_SERVICE:
#ifdef DEBUG_USB_HOST
      printf("Please enable discovery of your device\n");
#endif
      hci_inquiry();
      hci_state = HCI_INQUIRY_STATE;
      break;

    case HCI_INQUIRY_STATE:
      if (hci_check_flag(HCI_FLAG_DEVICE_FOUND)) {
        hci_inquiry_cancel(); // Stop inquiry

#ifdef DEBUG_USB_HOST
        printf("HID device found\n");
#endif
        hci_state = HCI_CONNECT_DEVICE_STATE;
      }
      break;

    case HCI_CONNECT_DEVICE_STATE:
      if (hci_check_flag(HCI_FLAG_CMD_COMPLETE)) {
#ifdef DEBUG_USB_HOST
        printf("Connecting to HID device\n");
#endif

        hci_connect(disc_bdaddr);
        hci_state = HCI_CONNECTED_DEVICE_STATE;
      }
      break;

    case HCI_CONNECTED_DEVICE_STATE:
      if (hci_check_flag(HCI_FLAG_CONNECT_EVENT)) {
        if (hci_check_flag(HCI_FLAG_CONNECT_COMPLETE)) {
          if (!readyToSend)
            break;

#ifdef DEBUG_USB_HOST
          printf("Connected to HID device\n");
#endif

          hci_authentication_request(); // This will start the pairing with the Wiimote
          //l2cap_connection_request();
          hci_state = HCI_SCANNING_STATE;
        } else {
#ifdef DEBUG_USB_HOST
          printf("Trying to connect one more time...\n");
#endif
          hci_connect(disc_bdaddr); // Try to connect one more time
        }
      }
      break;

    case HCI_SCANNING_STATE:
      if (!connectToHIDDevice && !pairWithHIDDevice) {
#ifdef DEBUG_USB_HOST
        printf("Wait For Incoming Connection Request\n");
#endif
        hci_write_scan_enable();
        waitingForConnection = true;
        hci_state = HCI_CONNECT_IN_STATE;
      }
      break;

    case HCI_CONNECT_IN_STATE:
      if (hci_check_flag(HCI_FLAG_INCOMING_REQUEST)) {
        waitingForConnection = false;
#ifdef DEBUG_USB_HOST
        printf("Incoming Connection Request\n");
#endif
        hci_remote_name();
        hci_state = HCI_REMOTE_NAME_STATE;
      } else if (hci_check_flag(HCI_FLAG_DISCONNECT_COMPLETE))
        hci_state = HCI_DISCONNECT_STATE;
      break;

    case HCI_REMOTE_NAME_STATE:
      if (hci_check_flag(HCI_FLAG_REMOTE_NAME_COMPLETE)) {
#ifdef DEBUG_USB_HOST
        printf("Remote Name: %s\n", remote_name);
#endif
        hci_accept_connection();
        hci_state = HCI_CONNECTED_STATE;
      }
      break;

    case HCI_CONNECTED_STATE:
      if (hci_check_flag(HCI_FLAG_CONNECT_COMPLETE)) {
#ifdef DEBUG_USB_HOST
        printf("Connected to Device: ");

        for (int8_t i = 5; i > 0; i--) {
          printf("%x:", disc_bdaddr[i]);
        }

        printf("%x\n", disc_bdaddr[0]);
#endif

        // Clear these flags for a new connection
        l2capConnectionClaimed = false;

        hci_event_flag = 0;
        hci_state = HCI_DONE_STATE;
      }
      break;

    case HCI_DONE_STATE:
      hci_counter++;

      if (hci_counter > 1000) { // Wait until we have looped 1000 times to make sure that the L2CAP connection has been started
        hci_counter = 0;
        hci_state = HCI_SCANNING_STATE;
      }
      break;

    case HCI_DISCONNECT_STATE:
      if (hci_check_flag(HCI_FLAG_DISCONNECT_COMPLETE)) {
#ifdef DEBUG_USB_HOST
        printf("HCI Disconnected from Device\n");
#endif
        hci_event_flag = 0; // Clear all flags

        // Reset all buffers
        memset(hcibuf, 0, BULK_MAXPKTSIZE);
        memset(l2capinbuf, 0, BULK_MAXPKTSIZE);

        hci_state = HCI_SCANNING_STATE;
      }
      break;

    default:
      break;
  }
}

void app_main() {
    nvs_flash_init();
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

    // Initialize Bluetooth
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        printf("Bluetooth controller initialize failed\n");
        return;
    }

    // Setup dual mode
    if (esp_bt_controller_enable(ESP_BT_MODE_BTDM) != ESP_OK) {
        printf("Bluetooth controller enable failed\n");
        return;
    }

    hci_state = HCI_INIT_STATE;
    hci_num_reset_loops = 100;

    xTaskCreatePinnedToCore(&mainTask, "mainTask", 2048, NULL, 5, NULL, 0);
}

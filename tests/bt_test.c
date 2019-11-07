#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_bt.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "bt_types.h"
#include "hcidefs.h"

static bool pin_code_request = false;

static uint8_t hci_cmd_buf[128];

static DEV_CLASS dev_class = {0, 0x08, 0x04};

static BD_ADDR own_addr = {0};

static BD_ADDR peer_addr = {0x28, 0x9A, 0x4B, 0x0A, 0x1D, 0x9A};

static uint16_t conn_handle = 0xFFFF;

#define PACKET_TYPES ( HCI_PKT_TYPES_MASK_DM1 | HCI_PKT_TYPES_MASK_DH1 \
                     | HCI_PKT_TYPES_MASK_DM3 | HCI_PKT_TYPES_MASK_DH3 \
                     | HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 )

#define HCI_PARAM_SIZE_L2CAP_CONNECTION_REQUEST_CONTROL (12)

#define HCI_ACL_PB_HLM_CONTINUE (1 << 12)
#define HCI_ACL_PB_HLM_FIRST    (2 << 12)

#define HCI_ACL_BC_POINT_TO_POINT    (0)
#define HCI_ACL_BC_ACTIVE_BROADCAST  (1 << 14)
#define HCI_ACL_BC_PICONET_BROADCAST (2 << 14)

static void controller_rcv_pkt_ready(void)
{
    printf("controller rcv pkt ready\n");
}

/* Incoming Bluetooth packet interrupt, parses events */
static int host_rcv_pkt(uint8_t *data, uint16_t len)
{
    uint16_t opcode;
    if (data[0] == HCIT_TYPE_EVENT) {
        printf("HCI EVENT: ");
        switch (data[1]) {
        case HCI_COMMAND_COMPLETE_EVT:
            opcode = data[4] | (data[5] << 8);
            printf("Command Complete (opcode: 0x%x): ", opcode);

            switch (opcode) {
            case HCI_RESET:
                printf("Reset: ");
                if (!data[6]) {
                    printf("Success");
                } else {
                    printf("ERROR!");
                }
                break;
            case HCI_READ_BD_ADDR:
                printf("Read BD ADDR: ");
                if (!data[6]) {
                    for(uint8_t i = 0; i < 6; i++)
				        own_addr[i] = data[7 + i];
                    printf("%x:%x:%x:%x:%x:%x", own_addr[0], own_addr[1], own_addr[2], own_addr[3], own_addr[4], own_addr[5]);
                } else {
                    printf("ERROR!");
                }
                break;
            case HCI_WRITE_CLASS_OF_DEVICE:
                printf("Write Class of Device: ");
                if (!data[6]) {
                    printf("Success");
                } else {
                    printf("ERROR!");
                }
                break;
            case HCI_CHANGE_LOCAL_NAME:
                printf("Write Local Name ");
                if (!data[6]) {
                    printf("Success");
                } else {
                    printf("ERROR!");
                }
                break;
            default:
                printf("Unknown: ");
                for (uint16_t i = 2; i < len; i++) {
                    printf("%02x", data[i]);
                }
            }
            break;

        case HCI_PIN_CODE_REQUEST_EVT:
            printf("PIN code request\n");
            pin_code_request = true;
            break;

        case HCI_COMMAND_STATUS_EVT:
            opcode = data[5] | (data[6] << 8);
            printf("Command Status (opcode: 0x%x): ", opcode);
            if (!data[3]) {
                printf("Pending");
            } else {
                printf("ERROR!");
            }
            break;
        case HCI_CONNECTION_COMP_EVT:
            conn_handle = data[4] | ((data[5] & 0x0F) << 8);
            printf("Connection Complete (handle: 0x%x)", conn_handle);
            break;
        case HCI_MAX_SLOTS_CHANGED_EVT:
            conn_handle = data[4] | ((data[5] & 0x0F) << 8);
            printf("Max Slots Changed (handle: 0x%x)", conn_handle);
            break;
        case HCI_ROLE_CHANGE_EVT:
            printf("Role Change ");
            if (!data[3]) {
                printf("Success: %s for the BD ADDR: ", (data[10] == 1 ? "Slave" : "Master"));
                for(uint8_t i = 0; i < 6; i++)
                    printf("%x:", data[4 + i]);
            } else {
                printf("ERROR!");
            }
            break;
        case HCI_DISCONNECTION_COMP_EVT:
            conn_handle = data[4] | ((data[5] & 0x0F) << 8);
            printf("Disconnection Complete (handle: 0x%x) ", conn_handle);
            if (!data[3]) {
                printf("Success, reason: 0x%x", data[6]);
                conn_handle = 0xFFFF;
            } else {
                printf("ERROR!");
            }
            break;
        default:
            printf("Unknown (0x%x): ", data[1]);
            for (uint16_t i = 1; i < len; i++) {
                printf("%02x", data[i]);
            }
        }
        printf("\n");
    } else if (data[0] == HCIT_TYPE_ACL_DATA) {
        printf("HCI ACL DATA: ");
        for (uint16_t i = 1; i < len; i++) {
            printf("%02x", data[i]);
        }
        printf("\n");
    } else {
        printf("HCI Unknown packet received (type 0x%d): ", data[0]);
        for (uint16_t i = 1; i < len; i++) {
            printf("%02x", data[i]);
        }
        printf("\n");
    }
    return 0;
}

static esp_vhci_host_callback_t vhci_host_cb = {
    controller_rcv_pkt_ready,
    host_rcv_pkt
};

/* HCI Control commands setup */

static uint16_t make_cmd_reset(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, HCIT_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_RESET);
    UINT8_TO_STREAM (buf, 0);

    return HCIC_PREAMBLE_SIZE;
}

static uint16_t make_cmd_read_bdaddr(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, HCIT_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_READ_BD_ADDR);
    UINT8_TO_STREAM (buf, 0);

    return HCIC_PREAMBLE_SIZE;
}

static uint16_t make_cmd_class_of_device(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, HCIT_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_WRITE_CLASS_OF_DEVICE);
    UINT8_TO_STREAM (buf, 3);

    DEVCLASS_TO_STREAM (buf, dev_class);

    return HCIC_PREAMBLE_SIZE + 3;
}

static uint16_t make_cmd_create_connection(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, HCIT_TYPE_COMMAND);
    UINT16_TO_STREAM (buf, HCI_CREATE_CONNECTION);
    UINT8_TO_STREAM (buf, 13);

    BDADDR_TO_STREAM (buf, peer_addr);
    UINT16_TO_STREAM (buf, PACKET_TYPES);
    UINT8_TO_STREAM (buf, HCI_PAGE_SCAN_REP_MODE_R1);
    UINT8_TO_STREAM (buf, HCI_MANDATARY_PAGE_SCAN_MODE);
    UINT16_TO_STREAM (buf, 0); // Ignore clock offset
    UINT8_TO_STREAM (buf, HCI_CR_CONN_ALLOW_SWITCH);

    return HCIC_PREAMBLE_SIZE + 13;
}

static uint16_t make_cmd_l2cap_send_connection_request_control(uint8_t *buf)
{
    UINT8_TO_STREAM (buf, HCIT_TYPE_ACL_DATA);
/*    UINT16_TO_STREAM (buf, HCI_CREATE_CONNECTION);
    UINT8_TO_STREAM (buf, HCIC_PARAM_SIZE_L2CAP_CONNECTION_REQUEST_CONTROL);*/

    UINT16_TO_STREAM (buf, (conn_handle & 0x0FFF) | (HCI_ACL_PB_HLM_FIRST) | (HCI_ACL_BC_POINT_TO_POINT));
    UINT16_TO_STREAM (buf, HCI_PARAM_SIZE_L2CAP_CONNECTION_REQUEST_CONTROL);

    UINT16_TO_STREAM (buf, 8); // Length
    UINT16_TO_STREAM (buf, 1); // L2CAP Signaling Channel
    UINT8_TO_STREAM (buf, 2);  // Connection Request
    UINT8_TO_STREAM (buf, 1);  // Command Identifier
    UINT16_TO_STREAM (buf, 4); // Command Length
    UINT16_TO_STREAM (buf, 0x0011); // PSM: HID-Control
    UINT16_TO_STREAM (buf, 0x0040); // Source CID: Dynamically Allocated Channel

    return HCI_DATA_PREAMBLE_SIZE + HCI_PARAM_SIZE_L2CAP_CONNECTION_REQUEST_CONTROL;
}

static uint16_t make_cmd_pin_code_reply(uint8_t *buf) {
  //memset(buf, 0, 128);
  UINT8_TO_STREAM(buf, HCIT_TYPE_COMMAND);
  UINT16_TO_STREAM(buf, HCI_PIN_CODE_REQUEST_REPLY);
  UINT8_TO_STREAM(buf, 0x17);
  BDADDR_TO_STREAM(buf, peer_addr);
  UINT8_TO_STREAM(buf, 4);
  UINT8_TO_STREAM(buf, 0x48);
  UINT8_TO_STREAM(buf, 0x48);
  UINT8_TO_STREAM(buf, 0x48);
  UINT8_TO_STREAM(buf, 0x48);

  return 26;
}

/* HCI Control commands send functions */
static void hci_send_pin_code_reply(void) {
  uint16_t sz = make_cmd_pin_code_reply(hci_cmd_buf);
  esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}

static void hci_send_reset(void)
{
    uint16_t sz = make_cmd_reset (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}

static void hci_send_read_bdaddr(void)
{
    uint16_t sz = make_cmd_read_bdaddr (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}

static void hci_send_class_of_device(void)
{
    uint16_t sz = make_cmd_class_of_device (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}

/*
static void hci_send_change_local_name(void)
{
    uint16_t sz = make_cmd_change_local_name (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}
*/

static void hci_send_create_connection(void)
{
    uint16_t sz = make_cmd_create_connection (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}

static void l2cap_send_connection_request_control(void)
{
    uint16_t sz = make_cmd_l2cap_send_connection_request_control (hci_cmd_buf);
    esp_vhci_host_send_packet(hci_cmd_buf, sz + 1);
}

/* Main Bluetooth task. Should use a state machine instead of just blindly sending out data. */

void mainTask(void *pvParameters)
{
    int cmd_cnt = 0;
    bool send_avail = false;

    esp_vhci_host_register_callback(&vhci_host_cb);

    printf("Connecting to host.\n");

    while (cmd_cnt < 4) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        send_avail = esp_vhci_host_check_send_available();
        if (send_avail) {
            switch (cmd_cnt) {
            case 0: hci_send_reset(); ++cmd_cnt; break;
            case 1: hci_send_read_bdaddr(); ++cmd_cnt; break;
            case 2: hci_send_class_of_device(); ++cmd_cnt; break;
            //case 3: hci_send_change_local_name(); ++cmd_cnt; break;
            case 3: hci_send_create_connection(); ++cmd_cnt; break;
            }
        }
    }

    printf("Sending L2CAP commands.\n");

    cmd_cnt = 0;

    while (cmd_cnt < 2) {
        vTaskDelay(10 / portTICK_PERIOD_MS);
        send_avail = esp_vhci_host_check_send_available();
        if (send_avail) {
            switch (cmd_cnt) {
            case 0:
                if (conn_handle != 0xFFFF) {
                    printf("Sending L2CAP\n");
                    l2cap_send_connection_request_control();
                    ++cmd_cnt;
                }
                break;

            case 1:
              if (pin_code_request) {
                printf("Sending PIN code\n");
                hci_send_pin_code_reply();
                ++cmd_cnt;
              }
              break;
            }
        }
    }

    printf("Successfully started.\n");

    while (1) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
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

    xTaskCreatePinnedToCore(&mainTask, "mainTask", 2048, NULL, 5, NULL, 0);
}

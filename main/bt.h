#define BULK_MAXPKTSIZE 64

#define UINT32_TO_STREAM(p, u32) {*(p)++ = (UINT8)(u32); *(p)++ = (UINT8)((u32) >> 8); *(p)++ = (UINT8)((u32) >> 16); *(p)++ = (UINT8)((u32) >> 24);}
#define UINT24_TO_STREAM(p, u24) {*(p)++ = (UINT8)(u24); *(p)++ = (UINT8)((u24) >> 8); *(p)++ = (UINT8)((u24) >> 16);}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT8_TO_STREAM(p, u8)   {*(p)++ = (uint8_t)(u8);}
#define INT8_TO_STREAM(p, u8)    {*(p)++ = (INT8)(u8);}
#define ARRAY32_TO_STREAM(p, a)  {register int ijk; for (ijk = 0; ijk < 32;           ijk++) *(p)++ = (UINT8) a[31 - ijk];}
#define ARRAY16_TO_STREAM(p, a)  {register int ijk; for (ijk = 0; ijk < 16;           ijk++) *(p)++ = (UINT8) a[15 - ijk];}
#define ARRAY8_TO_STREAM(p, a)   {register int ijk; for (ijk = 0; ijk < 8;            ijk++) *(p)++ = (UINT8) a[7 - ijk];}
#define BDADDR_TO_STREAM(p, a)   {register int ijk; for (ijk = 0; ijk < BD_ADDR_LEN;  ijk++) *(p)++ = (UINT8) a[BD_ADDR_LEN - 1 - ijk];}
#define LAP_TO_STREAM(p, a)      {register int ijk; for (ijk = 0; ijk < LAP_LEN;      ijk++) *(p)++ = (UINT8) a[LAP_LEN - 1 - ijk];}
#define DEVCLASS_TO_STREAM(p, a) {register int ijk; for (ijk = 0; ijk < DEV_CLASS_LEN;ijk++) *(p)++ = (UINT8) a[DEV_CLASS_LEN - 1 - ijk];}
#define ARRAY_TO_STREAM(p, a, len) {register int ijk; for (ijk = 0; ijk < len;        ijk++) *(p)++ = (UINT8) a[ijk];}
#define REVERSE_ARRAY_TO_STREAM(p, a, len)  {register int ijk; for (ijk = 0; ijk < len; ijk++) *(p)++ = (UINT8) a[len - 1 - ijk];}

#define PACKET_TYPES ( HCI_PKT_TYPES_MASK_DM1 | HCI_PKT_TYPES_MASK_DH1 \
                     | HCI_PKT_TYPES_MASK_DM3 | HCI_PKT_TYPES_MASK_DH3 \
                     | HCI_PKT_TYPES_MASK_DM5 | HCI_PKT_TYPES_MASK_DH5 )

#define HCI_PARAM_SIZE_L2CAP_CONNECTION_REQUEST_CONTROL (12)

#define HCI_ACL_PB_HLM_CONTINUE (1 << 12)
#define HCI_ACL_PB_HLM_FIRST    (2 << 12)

#define HCI_ACL_BC_POINT_TO_POINT    (0)
#define HCI_ACL_BC_ACTIVE_BROADCAST  (1 << 14)
#define HCI_ACL_BC_PICONET_BROADCAST (2 << 14)

/* HCI message type definitions (for H4 messages) */
#define HCIT_TYPE_COMMAND   1
#define HCIT_TYPE_ACL_DATA  2
#define HCIT_TYPE_SCO_DATA  3
#define HCIT_TYPE_EVENT     4
#define HCIT_TYPE_LM_DIAG   7
#define HCIT_TYPE_NFC       16

/* Bluetooth HCI states for hci_task() */
#define HCI_INIT_STATE                  0
#define HCI_RESET_STATE                 1
#define HCI_CLASS_STATE                 2
#define HCI_BDADDR_STATE                3
#define HCI_LOCAL_VERSION_STATE         4
#define HCI_SET_NAME_STATE              5
#define HCI_CHECK_DEVICE_SERVICE        6

#define HCI_INQUIRY_STATE               7 // These three states are only used if it should pair and connect to a device
#define HCI_CONNECT_DEVICE_STATE        8
#define HCI_CONNECTED_DEVICE_STATE      9

#define HCI_SCANNING_STATE              10
#define HCI_CONNECT_IN_STATE            11
#define HCI_REMOTE_NAME_STATE           12
#define HCI_CONNECTED_STATE             13
#define HCI_DISABLE_SCAN_STATE          14
#define HCI_DONE_STATE                  15
#define HCI_DISCONNECT_STATE            16

/* HCI event flags*/
#define HCI_FLAG_CMD_COMPLETE           (1UL << 0)
#define HCI_FLAG_CONNECT_COMPLETE       (1UL << 1)
#define HCI_FLAG_DISCONNECT_COMPLETE    (1UL << 2)
#define HCI_FLAG_REMOTE_NAME_COMPLETE   (1UL << 3)
#define HCI_FLAG_INCOMING_REQUEST       (1UL << 4)
#define HCI_FLAG_READ_BDADDR            (1UL << 5)
#define HCI_FLAG_READ_VERSION           (1UL << 6)
#define HCI_FLAG_DEVICE_FOUND           (1UL << 7)
#define HCI_FLAG_CONNECT_EVENT          (1UL << 8)

/* HCI Events managed */
#define EV_INQUIRY_COMPLETE                             0x01
#define EV_INQUIRY_RESULT                               0x02
#define EV_CONNECT_COMPLETE                             0x03
#define EV_INCOMING_CONNECT                             0x04
#define EV_DISCONNECT_COMPLETE                          0x05
#define EV_AUTHENTICATION_COMPLETE                      0x06
#define EV_REMOTE_NAME_COMPLETE                         0x07
#define EV_ENCRYPTION_CHANGE                            0x08
#define EV_CHANGE_CONNECTION_LINK                       0x09
#define EV_ROLE_CHANGED                                 0x12
#define EV_NUM_COMPLETE_PKT                             0x13
#define EV_PIN_CODE_REQUEST                             0x16
#define EV_LINK_KEY_REQUEST                             0x17
#define EV_LINK_KEY_NOTIFICATION                        0x18
#define EV_DATA_BUFFER_OVERFLOW                         0x1A
#define EV_MAX_SLOTS_CHANGE                             0x1B
#define EV_READ_REMOTE_VERSION_INFORMATION_COMPLETE     0x0C
#define EV_QOS_SETUP_COMPLETE                           0x0D
#define EV_COMMAND_COMPLETE                             0x0E
#define EV_COMMAND_STATUS                               0x0F
#define EV_LOOPBACK_COMMAND                             0x19
#define EV_PAGE_SCAN_REP_MODE                           0x20

/* Bluetooth states for the different Bluetooth drivers */
#define L2CAP_WAIT                      0
#define L2CAP_DONE                      1

/* Used for HID Control channel */
#define L2CAP_CONTROL_CONNECT_REQUEST   2
#define L2CAP_CONTROL_CONFIG_REQUEST    3
#define L2CAP_CONTROL_SUCCESS           4
#define L2CAP_CONTROL_DISCONNECT        5

/* Used for HID Interrupt channel */
#define L2CAP_INTERRUPT_SETUP           6
#define L2CAP_INTERRUPT_CONNECT_REQUEST 7
#define L2CAP_INTERRUPT_CONFIG_REQUEST  8
#define L2CAP_INTERRUPT_DISCONNECT      9

/* Used for SDP channel */
#define L2CAP_SDP_WAIT                  10
#define L2CAP_SDP_SUCCESS               11

/* Used for RFCOMM channel */
#define L2CAP_RFCOMM_WAIT               12
#define L2CAP_RFCOMM_SUCCESS            13

#define L2CAP_DISCONNECT_RESPONSE       14 // Used for both SDP and RFCOMM channel

/* Bluetooth states used by some drivers */
#define TURN_ON_LED                     17
#define PS3_ENABLE_SIXAXIS              18
#define WII_CHECK_MOTION_PLUS_STATE     19
#define WII_CHECK_EXTENSION_STATE       20
#define WII_INIT_MOTION_PLUS_STATE      21

/* L2CAP event flags for HID Control channel */
#define L2CAP_FLAG_CONNECTION_CONTROL_REQUEST           (1UL << 0)
#define L2CAP_FLAG_CONFIG_CONTROL_SUCCESS               (1UL << 1)
#define L2CAP_FLAG_CONTROL_CONNECTED                    (1UL << 2)
#define L2CAP_FLAG_DISCONNECT_CONTROL_RESPONSE          (1UL << 3)

/* L2CAP event flags for HID Interrupt channel */
#define L2CAP_FLAG_CONNECTION_INTERRUPT_REQUEST         (1UL << 4)
#define L2CAP_FLAG_CONFIG_INTERRUPT_SUCCESS             (1UL << 5)
#define L2CAP_FLAG_INTERRUPT_CONNECTED                  (1UL << 6)
#define L2CAP_FLAG_DISCONNECT_INTERRUPT_RESPONSE        (1UL << 7)

/* L2CAP event flags for SDP channel */
#define L2CAP_FLAG_CONNECTION_SDP_REQUEST               (1UL << 8)
#define L2CAP_FLAG_CONFIG_SDP_SUCCESS                   (1UL << 9)
#define L2CAP_FLAG_DISCONNECT_SDP_REQUEST               (1UL << 10)

/* L2CAP event flags for RFCOMM channel */
#define L2CAP_FLAG_CONNECTION_RFCOMM_REQUEST            (1UL << 11)
#define L2CAP_FLAG_CONFIG_RFCOMM_SUCCESS                (1UL << 12)
#define L2CAP_FLAG_DISCONNECT_RFCOMM_REQUEST            (1UL << 13)

#define L2CAP_FLAG_DISCONNECT_RESPONSE                  (1UL << 14)

/* Macros for L2CAP event flag tests */
#define l2cap_check_flag(flag) (l2cap_event_flag & (flag))
#define l2cap_set_flag(flag) (l2cap_event_flag |= (flag))
#define l2cap_clear_flag(flag) (l2cap_event_flag &= ~(flag))

/* L2CAP signaling commands */
#define L2CAP_CMD_COMMAND_REJECT        0x01
#define L2CAP_CMD_CONNECTION_REQUEST    0x02
#define L2CAP_CMD_CONNECTION_RESPONSE   0x03
#define L2CAP_CMD_CONFIG_REQUEST        0x04
#define L2CAP_CMD_CONFIG_RESPONSE       0x05
#define L2CAP_CMD_DISCONNECT_REQUEST    0x06
#define L2CAP_CMD_DISCONNECT_RESPONSE   0x07
#define L2CAP_CMD_INFORMATION_REQUEST   0x0A
#define L2CAP_CMD_INFORMATION_RESPONSE  0x0B

// Used For Connection Response - Remember to Include High Byte
#define PENDING     0x01
#define SUCCESSFUL  0x00

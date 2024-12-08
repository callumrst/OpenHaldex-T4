#define haldexGen 4  // Generation for Haldex (1, 4) - used in addresses

// ***** Generation 1 Haldex Addresses *****
#if (haldexGen == 1)
#define MOTOR1_ID 0x280
#define MOTOR2_ID 0x288
#define MOTOR3_ID 0x380
#define MOTOR5_ID 0x480
#define MOTOR6_ID 0x488
#define MOTOR7_ID 0x588
#define MOTORBREMS_ID 0x284
#define MOTOR_FLEX_ID 0x580

#define BRAKES1_ID 0x1A0
#define BRAKES2_ID 0x2A0
#define BRAKES3_ID 0x4A0
#define BRAKES5_ID 0x5A0

#define GRA_ID 0x38A
#define HALDEX_ID 0x2C0
#endif

// ***** Generation 2 Haldex Addresses *****
#if (haldexGen == 2)
#define MOTOR1_ID 0x280      // MED9.1
#define MOTOR2_ID 0x288      // MED9.1
#define MOTOR3_ID 0x380      // MED9.1
#define MOTOR5_ID 0x480      // MED9.1
#define MOTOR6_ID 0x488      // MED9.1
#define MOTOR7_ID 0x588      // MED9.1
#define MOTORBREMS_ID 0x284  // MED9.1 6 byte, brake booster etc
#define MOTOR_FLEX_ID 0x580  // MED9.1

#define BRAKES1_ID 0x1A0  // MED9.1
#define BRAKES2_ID 0x5A0
#define BRAKES3_ID 0x4A0
#define BRAKES5_ID 0x5A0

#define GRA_ID 0x38A  // MED9.1
#define HALDEX_ID 0x2C0
#endif

// ***** Generation 4 Haldex Addresses *****
#if (haldexGen == 4)
#define MOTOR1_ID 0x280
#define MOTOR2_ID 0x288
#define MOTOR3_ID 0x380
#define MOTOR5_ID 0x480
#define MOTOR6_ID 0x488
#define MOTOR7_ID 0x588
#define MOTORBREMS_ID 0x284
#define MOTOR_FLEX_ID 0x580

#define BRAKES1_ID 0x1A0
#define BRAKES2_ID 0x5A0
#define BRAKES3_ID 0x4A0
#define BRAKES5_ID 0x5A0

#define GRA_ID 0x38A
#define HALDEX_ID 0x2C0
#endif

#define fisCuntrol_ID 0x7B0
#define openHaldex_ID 0x7C0

#define NUM_LOCK_POINTS 10

#define DATA_CTRL_CHECK_LOCKPOINTS 0
#define DATA_CTRL_CLEAR 1
#define DATA_CTRL_CHECK_MODE 2
#define DATA_CTRL_RECONNECT_BT 3

#define APP_MSG_MODE 0
#define APP_MSG_STATUS 1
#define APP_MSG_CUSTOM_DATA 2
#define APP_MSG_CUSTOM_CTRL 3
#define APP_MSG_IS_SCREEN 4

#define serialPacketEnd 0xff

#define arraySize(array) (sizeof((array)) / sizeof((array)[0]))

typedef struct can_s {
  byte status;
  bool inited;
  const char* name;
} can_s;

typedef enum openhaldex_mode_id {
  MODE_STOCK,
  MODE_FWD,
  MODE_5050,
  MODE_CUSTOM
} openhaldex_mode_id;

typedef struct lockpoint {
  byte speed;
  byte lock;
  byte intensity;
} lockpoint;

typedef struct openhaldex_custom_mode {
  lockpoint lockpoints[NUM_LOCK_POINTS];
  byte lockpoint_rx_h;
  byte lockpoint_rx_l;
  byte lockpoint_count;
} openhaldex_custom_mode;

typedef struct openhaldexState {
  openhaldex_mode_id mode;
  openhaldex_custom_mode custom_mode;
  byte ped_threshold;
  bool mode_override;
} openhaldexState;

typedef struct bt_packet {
  byte len;
  byte data[12];
} bt_packet;

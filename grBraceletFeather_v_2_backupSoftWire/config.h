/*pins

*/
#define LED 13 //let pin 
#define BUTTON_PIN 0 //main button 
#define LED_MARKERS A0 //led markers pin 
//imu addres change triger pins
int8_t PALM = 1; //5
int8_t THUMB = 6; //6
int8_t INDEX = 5; //1
int8_t MIDDLE  = 11; //10
int8_t RING  = 12; //11
int8_t PINKY  = 10; //12
#define VBAT A7

int8_t sa0[6] = { PINKY, RING, MIDDLE, INDEX, THUMB, PALM};
/*bluetooth

*/
#define BUFSIZE 160 //size of reading buffer 
#define VERBOSE_MODE true //need for debuging
//wifi defs
#define SECRET_SSID "gr-ap"
#define SECRET_PASS "agbdlcid"

#define FACTORYRESET_ENABLE 1 //enable factory reset
#define SET_ATTRIBUTES 1  //sett attributes line name and other

#define SOH 1
#define EOT 4
#define BAT 6

//vars
#define SERIAL_VERBOSE_MODE 1
String dev_name = "AT+GAPDEVNAME=GR[DEV]";
//#define ACTIVE_ADDR 0x6B
byte ACTIVE_ADDR = 0x6B;

//LIS3MDL


template <typename T> struct imuvector
{
  T x, y, z;
};

#define LIS3MDL_SA1_HIGH_ADDRESS  0b0011110
#define LIS3MDL_SA1_LOW_ADDRESS   0b0011100

#define LIS3MDL_TEST_REG_ERROR -1

#define LIS3MDL_WHO_ID  0x3D

enum lis3mdldeviceType { device_LIS3MDL, lis3mdldevice_auto };
enum sa1State { sa1_low, sa1_high, sa1_auto };

// register addresses
enum lis3mdlregAddr
{
  LIS3MDL_WHO_AM_I    = 0x0F,

  CTRL_REG1   = 0x20,
  CTRL_REG2   = 0x21,
  CTRL_REG3   = 0x22,
  CTRL_REG4   = 0x23,
  CTRL_REG5   = 0x24,

  STATUS_REG  = 0x27,
  OUT_X_L     = 0x28,
  OUT_X_H     = 0x29,
  OUT_Y_L     = 0x2A,
  OUT_Y_H     = 0x2B,
  OUT_Z_L     = 0x2C,
  OUT_Z_H     = 0x2D,
  TEMP_OUT_L  = 0x2E,
  TEMP_OUT_H  = 0x2F,
  INT_CFG     = 0x30,
  INT_SRC     = 0x31,
  INT_THS_L   = 0x32,
  INT_THS_H   = 0x33,
};


//LSM6

// The Arduino two-wire interface uses a 7-bit number for the address,
// and sets the last bit correctly based on reads and writes
#define DS33_SA0_HIGH_ADDRESS 0b1101011
#define DS33_SA0_LOW_ADDRESS  0b1101010

#define LSM6_TEST_REG_ERROR -1

#define DS33_WHO_ID    0x69

enum lsm6deviceType { device_DS33, lsm6device_auto };
enum sa0State { sa0_low, sa0_high, sa0_auto };

// register addresses
enum lsm6regAddr
{
  FUNC_CFG_ACCESS   = 0x01,

  FIFO_CTRL1        = 0x06,
  FIFO_CTRL2        = 0x07,
  FIFO_CTRL3        = 0x08,
  FIFO_CTRL4        = 0x09,
  FIFO_CTRL5        = 0x0A,
  ORIENT_CFG_G      = 0x0B,

  INT1_CTRL         = 0x0D,
  INT2_CTRL         = 0x0E,
  LSM6_WHO_AM_I     = 0x0F,
  CTRL1_XL          = 0x10,
  CTRL2_G           = 0x11,
  CTRL3_C           = 0x12,
  CTRL4_C           = 0x13,
  CTRL5_C           = 0x14,
  CTRL6_C           = 0x15,
  CTRL7_G           = 0x16,
  CTRL8_XL          = 0x17,
  CTRL9_XL          = 0x18,
  CTRL10_C          = 0x19,

  WAKE_UP_SRC       = 0x1B,
  TAP_SRC           = 0x1C,
  D6D_SRC           = 0x1D,
  LSM6_STATUS_REG   = 0x1E,

  OUT_TEMP_L        = 0x20,
  OUT_TEMP_H        = 0x21,
  OUTX_L_G          = 0x22,
  OUTX_H_G          = 0x23,
  OUTY_L_G          = 0x24,
  OUTY_H_G          = 0x25,
  OUTZ_L_G          = 0x26,
  OUTZ_H_G          = 0x27,
  OUTX_L_XL         = 0x28,
  OUTX_H_XL         = 0x29,
  OUTY_L_XL         = 0x2A,
  OUTY_H_XL         = 0x2B,
  OUTZ_L_XL         = 0x2C,
  OUTZ_H_XL         = 0x2D,

  FIFO_STATUS1      = 0x3A,
  FIFO_STATUS2      = 0x3B,
  FIFO_STATUS3      = 0x3C,
  FIFO_STATUS4      = 0x3D,
  FIFO_DATA_OUT_L   = 0x3E,
  FIFO_DATA_OUT_H   = 0x3F,
  TIMESTAMP0_REG    = 0x40,
  TIMESTAMP1_REG    = 0x41,
  TIMESTAMP2_REG    = 0x42,

  STEP_TIMESTAMP_L  = 0x49,
  STEP_TIMESTAMP_H  = 0x4A,
  STEP_COUNTER_L    = 0x4B,
  STEP_COUNTER_H    = 0x4C,

  FUNC_SRC          = 0x53,

  TAP_CFG           = 0x58,
  TAP_THS_6D        = 0x59,
  INT_DUR2          = 0x5A,
  WAKE_UP_THS       = 0x5B,
  WAKE_UP_DUR       = 0x5C,
  FREE_FALL         = 0x5D,
  MD1_CFG           = 0x5E,
  MD2_CFG           = 0x5F,
};

imuvector<int16_t> a; // accelerometer readings
imuvector<int16_t> g; // gyro readings

uint8_t lsm6last_status; // status of last I2C transmission

/*pins

*/
#define CPU_HZ 48000000000
#define TIMER_PRESCALER_DIV 1024

uint32_t sampleRate = 20; //sample rate of the sine wave in Hertz, how many times per second the TC5_Handler() function gets called per second basically

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
#define SERIAL_VERBOSE_MODE 0
String dev_name = "AT+GAPDEVNAME=GR[DEV]";
//#define ACTIVE_ADDR 0x6B
byte ACTIVE_ADDR = 0x6B;

/*pins

*/
#define LED 13 //let pin 
#define BUTTON_PIN 0 //main button 
#define LED_MARKERS 1 //led markers pin 
//imu addres change triger pins
int8_t PALM = 5; //5
int8_t THUMB = 6; //6
int8_t INDEX = 1; //1
int8_t MIDDLE  = 10; //10
int8_t RING  = 11; //11
int8_t PINKY  = 12; //12
#define VBAT A7

int8_t sa0[6] = {PALM, PINKY, RING, MIDDLE, INDEX, THUMB};
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
#define ACTIVE_ADDR 0x6B

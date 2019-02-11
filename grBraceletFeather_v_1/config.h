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

#define BLUEFRUIT_UART_MODE_PIN        -1 // default is 12 set to -1 if unused

//SPI settings
#define BLUEFRUIT_SPI_CS  8
#define BLUEFRUIT_SPI_IRQ 7
#define BLUEFRUIT_SPI_RST 4    // Optional but recommended, set to -1 if unused

#define FACTORYRESET_ENABLE 1 //enable factory reset
#define SET_ATTRIBUTES 1  //sett attributes line name and other
#define MINIMUM_FIRMWARE_VERSION  "0.7.7"
#define MODE_LED_BEHAVIOUR  "MODE"

#define SOH 1
#define EOT 4
#define BAT 6

//vars
#define SERIAL_VERBOSE_MODE 1
String dev_name = "AT+GAPDEVNAME=GR[L]";
#define ACTIVE_ADDR 0x6B

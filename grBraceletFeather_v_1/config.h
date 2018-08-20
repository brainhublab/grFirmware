/*pins

*/
#define LED 13 //let pin 
#define BUTTON 0 //main button 
#define LED_MARKERS 1 //led markers pin 
//imu addres change triger pins
#define PALM 5
#define THUMB 6
#define INDEX 9
#define MIDDLE 10
#define RING 11
#define PINKY 12
#define VBAT A7

byte fingers[6] = {PINKY, RING, MIDDLE, INDEX, THUMB, PALM};
/*bluetooth

*/
#define BUFSIZE 128 //size of reading buffer 
#define VERBOSE_MODE true //need for debuging

#define BLUEFRUIT_UART_MODE_PIN        -1 // default is 12 set to -1 if unused

//SPI settings
#define BLUEFRUIT_SPI_CS  8
#define BLUEFRUIT_SPI_IRQ 7
#define BLUEFRUIT_SPI_RST 4    // Optional but recommended, set to -1 if unused

#define FACTORYRESET_ENABLE 0 //enable factory reset
#define SET_ATTRIBUTES 1  //sett attributes line name and other
#define MINIMUM_FIRMWARE_VERSION  "0.7.7"
#define MODE_LED_BEHAVIOUR  "MODE"



//vars
#define SERIAL_VERBOSE_MODE 1
String devName = "AT+GAPDEVNAME=GR[L]";
#define ACTIVE_ADDR 0x6B

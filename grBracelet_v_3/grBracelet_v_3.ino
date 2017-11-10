
#include <Wire.h>
#include <SoftwareSerial.h>

#include "sensor.h"  //include sensor.h header with IMU data struct

extern "C" {
#include "utility/twi.h"  // from Wire library, so we can do bus scanning
}


#define STATUS_LED 13 //status led just for verifying of calibration and working of device
#define IR_LEDS 5 // IR leds diadem needet for position recognition
#define LEDS_ON 6 // emulate hold button so if its holded IR diadem is on 
#define IMUS_N 6 //number of IMUS
#define IMUS_INIT_PORT 0 //Initial port fo I2C multiplexer 
#define TCAADDR 0x70 //addres of I2C multiplexer needet from wire.h for connection to it
#define SENDING_DATA_TIMER_BOUND 6

#define PALM_INDEX 5

unsigned long current_timer, last_timer, temporary_timer, buffer_timer; //timer needed for IR leds blinkin

bool finger_is_connected = false; //flag to verify if current finger IMU is connected
bool connected_fingers_id[IMUS_N] = {0, 0, 0, 0, 0, 0}; //boolean array conains values of connected IMU's

const int IMU_SIGN[9] = {1, -1, -1, 1, -1, -1, 1, -1, -1};// IMU sign sets the palm IMU coordinate system to right coordinate system https://www.evl.uic.edu/ralph/508S98/coordinates.html

const int FINGER_IMU_SIGN[9] = { -1, 1, 1, 1, -1, -1, -1, 1, -1}; //IMU sign for fingers IMU's
IMU IMUS[IMUS_N]; //array from IMU for iterative call and read data

SoftwareSerial bSerial(4, 3); //software serial for serial bluetooth communication on digital pins
/*setup of IMUs and pins*/
void setup()
{
  bSerial.begin(115200); //serial bluetooth data sending bandwidth
  Serial.begin(115200);

  Serial.println("Begin...");

  /*setting the led pins */
  pinMode(STATUS_LED, OUTPUT);
  pinMode(IR_LEDS, OUTPUT);
  /*setting hold button pin */
  pinMode(LEDS_ON, INPUT);

  /*initialize I2C communication (implemented on I2C source file)*/
  I2C_Init();

  /*just blinking*/
  digitalWrite(STATUS_LED, LOW);
  delay(200);

  /*iterative initialization and simple callibration of  IMUs*/
  for (uint8_t port_id = 0; port_id < IMUS_N; port_id++)
  {
    TCA_Select(port_id); //select port on I2C multiplexer (implemented in I2C source file)

    Accel_Init(); //initialization of accelerometer (implemented in I2C source file)
    Compass_Init(); //initialization of magnetometer (implemented in I2C source file)
    Gyro_Init(); //initialization of gyroscope (implemented in I2C source file)

    digitalWrite(STATUS_LED, HIGH);
    delay(200);
    digitalWrite(STATUS_LED, LOW);

    /*iteration of all I2C addreces on each multiplexer port and check if connected*/
    for (uint8_t addr = 0; addr <= 127; addr++)
    {
      if (addr == TCAADDR) continue; //except addres of multiplexer

      uint8_t data;

      /*check if it's posible to get addres of port*/
      if (! twi_writeTo(addr, &data, 0, 1, 1)) //for reference "utility/twi.h"
      {
        Serial.print("Found I2C 0x");  Serial.println(addr, HEX); //Serial.println(addr);
        finger_is_connected = true; //so if it is possible to retrieve addres from curent port we have a finger IMU attached
      }

    }
    /*if finger IMU is connected make calibration of gyroscope*/
    if (finger_is_connected)
    {
      /* make 32 readings and store values*/
      for (uint8_t calib_iterator = 0; calib_iterator < 32; calib_iterator++)
      {
        Read_Gyro(port_id);
        for (uint8_t axsis_id = 0; axsis_id < 3; axsis_id ++)
        {
          IMUS[port_id].G_AN_OFFSET[axsis_id] += IMUS[port_id].G_AN[axsis_id];
        }
        delay(20);
      }
      /* extract average from acumulated values for generating offsets*/
      for (uint8_t axsis_id = 0; axsis_id < 3; axsis_id++)
      {
        IMUS[port_id].G_AN_OFFSET[axsis_id] = IMUS[port_id].G_AN_OFFSET[axsis_id] / 32;
      }
      connected_fingers_id[port_id] = 1; //push true flag in boolean array
      Serial.print(port_id);
    }

    finger_is_connected = false;
  }

  digitalWrite(IR_LEDS, HIGH);

  delay(200);
  digitalWrite(STATUS_LED, HIGH);
  delay(20);

}
void loop()
{

  //current_time = millis();
  // Serial.println(current_time - last_time);

  /*if hold button emulator is give HIGH enable IR LEDS*/
  // if (TIMER <= (current_time - last_time))
  // {
  //  digitalWrite(IR_LEDS, HIGH);
  //  last_time = current_time;
  //}
  /*iterative read of each IMU*/
  for (uint8_t i = 0; i < IMUS_N; i++)
  {
    if (connected_fingers_id[i])
    {
      TCA_Select(i); //select port on multiplexer
      Read_Gyro(i); //make readings gyro
      Read_Accel(i); //make readings accelerometer
      Read_Compass(i); // make readings compas

      current_timer = millis();
      temporary_timer = current_timer - last_timer;
      last_timer = current_timer;
      
      if(temporary_timer < SENDING_DATA_TIMER_BOUND)
      {
        buffer_timer = (SENDING_DATA_TIMER_BOUND - temporary_timer);
        delay(buffer_timer);
      }
      IMUS[i].time_stamp = buffer_timer + temporary_timer;//timestamp needed for integration
      printdata(i); // send data via Software serial via bluetooth module
    }
  }
  //digitalWrite(IR_LEDS, LOW);

}

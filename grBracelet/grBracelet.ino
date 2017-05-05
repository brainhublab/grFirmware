#include "sensor.h"
#include <Wire.h>
#include <SoftwareSerial.h>
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the right
   // and Z axis pointing down.
// Positive pitch : nose up
// Positive roll : right wing down
// Positive yaw : clockwise
int SENSOR_SIGN[9] = {1,1,-1,-1,-1,1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer
int FINGER_SENSOR_SIGN[9] = {-1,-1,1,1,1,-1,-1,-1,-1};
// Uncomment the below line to use this axis definition:
   // X axis pointing forward
   // Y axis pointing to the left
   // and Z axis pointing up.
// Positive pitch : nose down
// Positive roll : right wing down
// Positive yaw : counterclockwise
//int SENSOR_SIGN[9] = {1,-1,-1,-1,1,1,1,-1,-1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// tested with Arduino Uno with ATmega328 and Arduino Duemilanove with ATMega168

// accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256 //this equivalent to 1G in the raw data coming from the accelerometer

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

#define STATUS_LED 13

#define SENSORS_N 6
#define SENSORS_INIT_PORT 0
#define TCAADDR 0x70

SoftwareSerial bSerial(9, 10);

float G_Dt=0.02;    // Integration time (DCM algorithm)  We will run the integration loop at 50Hz if possible

sensor SENSORS[SENSORS_N];

void setup()
{
  Serial.begin(115200);
  bSerial.begin(115200);
  pinMode (STATUS_LED,OUTPUT);  // Status LED

  I2C_Init();

  //Serial.println("Pololu MinIMU-9 + Arduino AHRS");

  digitalWrite(STATUS_LED,LOW);
  delay(1500);

  //Serial.println("Initing init :D");
  for(uint8_t i=0; i<SENSORS_N; i++) {
    TCA_Select(i);

    //Serial.println("acc");
    Accel_Init();
    //Serial.println("mag");
    Compass_Init();
    //Serial.println("gyro");
    Gyro_Init();

    delay(200);

    //Serial.println("We will take some readings for a moment ;P");
    for(uint8_t j=0; j<32; j++) 
    {  // We take some readings...
      Read_Gyro(i);
      Read_Accel(i);

      for(uint8_t y=0; y<6; y++) 
      {  // Cumulate values
        SENSORS[i].AN_OFFSET[y] += SENSORS[i].AN[y];
      }
      delay(40);
    }

    for(uint8_t y=0; y<6; y++)
    {
      SENSORS[i].AN_OFFSET[y] = SENSORS[i].AN_OFFSET[y] / 32;
      Serial.print(SENSORS[i].AN_OFFSET[y]);
      Serial.print(" ");
      
    }
    Serial.println();
    /*SENSORS[i].AN_OFFSET[0] = 67;
    SENSORS[i].AN_OFFSET[1] = -73;
    SENSORS[i].AN_OFFSET[2] = -60;
    SENSORS[i].AN_OFFSET[3] = 3910;
    SENSORS[i].AN_OFFSET[4] = -570;
    SENSORS[i].AN_OFFSET[5] = 955;
    
  */
  if(i==5)
  {
    SENSORS[i].AN_OFFSET[5] -= GRAVITY * SENSOR_SIGN[5];
  }
  else
  {
    SENSORS[i].AN_OFFSET[5] -= GRAVITY * FINGER_SENSOR_SIGN[5];
 
    }
  }

  delay(2000);
  digitalWrite(STATUS_LED,HIGH);

  Serial.println("Setting timers, almost there ;)");
  for(uint8_t i=SENSORS_INIT_PORT; i<SENSORS_N + SENSORS_INIT_PORT; i++) {
    SENSORS[i].timer=millis();
    SENSORS[i].counter=0;
  }

  delay(20);
}

void loop() //Main Loop
{
  for(uint8_t i=0; i<SENSORS_N; i++) {
    if((millis()-SENSORS[i].timer)>=20) {  // Main loop runs at 50Hz
      TCA_Select(i);

      SENSORS[i].counter++;
      SENSORS[i].timer_old = SENSORS[i].timer;
      SENSORS[i].timer=millis();

      if (SENSORS[i].timer>SENSORS[i].timer_old) {
        G_Dt = (SENSORS[i].timer-SENSORS[i].timer_old)/1000.0;    // Real time of loop run. We use this on the DCM algorithm (gyro integration time)

        if (G_Dt > 0.2) {
          G_Dt = 0; // ignore integration times over 200 ms
        }
      } else {
        G_Dt = 0;
      }

      // Data adquisition
      Read_Gyro(i);   // This read gyro data
      Read_Accel(i);     // Read I2C accelerometer

      if (SENSORS[i].counter > 5)  // Read compass data at 10Hz... (5 loop runs)
      {
        SENSORS[i].counter=0;
        Read_Compass(i);    // Read I2C magnetometer
      }

      printdata(i);
     // delay(100);
      
    }
  }
}

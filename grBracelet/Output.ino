void printdata(uint8_t i)
{
      //bSerial.print("!");

      if(i==5)
      {
        bSerial.print(i);
       
  
        bSerial.print(" ");
        bSerial.print(SENSORS[i].gyro_z);
        bSerial.print(" ");
        bSerial.print(SENSORS[i].gyro_y);
        bSerial.print(" ");
        bSerial.print(SENSORS[i].gyro_x);
        bSerial.print(" ");
        bSerial.print(SENSORS[i].accel_z);
        bSerial.print (" ");
        bSerial.print(SENSORS[i].accel_y);
        bSerial.print (" ");
        bSerial.print(SENSORS[i].accel_x);
        bSerial.print(" ");
        bSerial.print(SENSORS[i].magnetom_z);
        bSerial.print (" ");
        bSerial.print(SENSORS[i].magnetom_y);
        bSerial.print (" ");
        bSerial.print(SENSORS[i].magnetom_x);
        //bSerial.println();
        bSerial.print('\n');
      }
      else
      {
        
        bSerial.print(i);
       
  
        bSerial.print(" ");
        bSerial.print(SENSORS[i].gyro_z); // y
        bSerial.print(" ");
        bSerial.print(SENSORS[i].gyro_x); // z
        bSerial.print(" ");
        bSerial.print(-SENSORS[i].gyro_y);
        bSerial.print(" ");
        bSerial.print(SENSORS[i].accel_z);
        bSerial.print (" ");
        bSerial.print(SENSORS[i].accel_x);
        bSerial.print (" ");
        bSerial.print(-SENSORS[i].accel_y);
        bSerial.print(" ");
        bSerial.print(SENSORS[i].magnetom_z);
        bSerial.print (" ");
        bSerial.print(SENSORS[i].magnetom_x);
        bSerial.print (" ");
        bSerial.print(-SENSORS[i].magnetom_y);
        //bSerial.println();
        bSerial.print('\n');
        }
      
}

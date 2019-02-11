import processing.serial.*;
Serial myPort;

float yaw = 0.0;
float pitch = 0.0;
float roll = 0.0;

float yaw1 = 0.0;
float pitch1 = 0.0;
float roll1 = 0.0;

void setup()
{
 // size(600, 500, P3D);
  size(1000, 800, P3D);

  // if you have only ONE serial port active
  //myPort = new Serial(this, Serial.list()[0], 9600); // if you have only ONE serial port active

  // if you know the serial port name
  //myPort = new Serial(this, "COM5:", 9600);                    // Windows
  myPort = new Serial(this, "/dev/ttyACM0", 115200);             // Linux
  //myPort = new Serial(this, "/dev/cu.usbmodem1217321", 9600);  // Mac

  textSize(16); // set text size
  textMode(SHAPE); // set text mode to shape
}

void draw()
{
  serialEvent();  // read and parse incoming serial message
  background(255); // set background to white
  lights();
  tryDraw( 5, 2, roll1, pitch1, yaw1);
  tryDraw( 2, 150, roll, pitch, yaw);
  //tryDraw( 5, 2, roll, pitch, yaw);
}

void tryDraw(float trans1, float trans2, float r, float p, float y)
{
  

  translate(width/trans1, height/trans2); // set position to centre

  pushMatrix(); // begin object

  float c1 = cos(radians(r));
  float s1 = sin(radians(r));
  float c2 = cos(radians(p));
  float s2 = sin(radians(p));
  float c3 = cos(radians(y));
  float s3 = sin(radians(y));
  applyMatrix( c2*c3, s1*s3+c1*c3*s2, c3*s1*s2-c1*s3, 0,
               -s2, c1*c2, c2*s1, 0,
               c2*s3, c1*s2*s3-c3*s1, c1*c3+s1*s2*s3, 0,
               0, 0, 0, 1);

  drawArduino();


  popMatrix(); // end of object

  // Print values to console
  print(roll);
  print("\t");
  print(pitch);
  print("\t");
  print(yaw);
  println();

}

void serialEvent()
{
  int newLine = 13; // new line character in ASCII
  String message;
  do {
    message = myPort.readStringUntil(newLine); // read from port until new line
    if (message != null) {
      String[] list = split(trim(message), " ");
      if (list.length >= 7 && list[0].equals("Orientation:")) {
        yaw = float(list[1]); // convert to float yaw
        pitch = float(list[2]); // convert to float pitch
        roll = float(list[3]); // convert to float roll
        
        yaw1 = float(list[4]); // convert to float pyaw
        pitch1 = float(list[5]); // convert to float ppitch
        roll1 = float(list[6]); // convert to float proll
      }
    }
  } while (message != null);
}

void drawArduino()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 200); // draw Arduino board base shape

  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, -10, 90); // set position to edge of Arduino box
  box(170, 20, 10); // draw pin header as box

  translate(-20, 0, -180); // set position to other edge of Arduino box
  box(210, 20, 10); // draw other pin header as box
}
void drawArduino1()
{
  /* function contains shape(s) that are rotated with the IMU */
  stroke(0, 90, 90); // set outline colour to darker teal
  fill(0, 130, 130); // set fill colour to lighter teal
  box(300, 10, 200); // draw Arduino board base shape
/*
  stroke(0); // set outline colour to black
  fill(80); // set fill colour to dark grey

  translate(60, -10, 90); // set position to edge of Arduino box
  box(170, 20, 10); // draw pin header as box

  translate(-20, 0, -180); // set position to other edge of Arduino box
  box(210, 20, 10); // draw other pin header as box
  */
}

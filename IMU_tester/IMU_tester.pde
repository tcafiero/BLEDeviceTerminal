/*
This code is made for processing https://processing.org/
 */

import processing.serial.*;     // import the Processing serial library
Serial myPort;                  // The serial port
String my_port = "COM11";      //choose your port
float roll, pitch, yaw;

void setup() {
  size(1024, 800, P3D);

  myPort = new Serial(this, my_port, 115200);
  myPort.bufferUntil('\n');

  smooth();
}

void draw() {
  background(0);
  noStroke();
  translate(width/2, height/2);
  pushMatrix();
  //rotateX(radians(roll));
  //rotateY(radians(pitch));
  //rotateZ(radians(yaw));
  rotateX(roll);
  rotateY(pitch);
  rotateZ(yaw);
  box(100, 50, 600);
  popMatrix();
}


void serialEvent(Serial myPort) {
  try {
    String myString = myPort.readStringUntil('\n');
    myString = trim(myString);
    float sensors[] = float(split(myString, ':'));

    roll = sensors[0];
    pitch = sensors[1];
    yaw = sensors[2];
    //print(sensors[0]);

    println("roll: " + roll + " pitch: " + pitch + " yaw: " + yaw + "\n"); //debug
  }
  catch (Exception e) {
    println("error");
  }
}

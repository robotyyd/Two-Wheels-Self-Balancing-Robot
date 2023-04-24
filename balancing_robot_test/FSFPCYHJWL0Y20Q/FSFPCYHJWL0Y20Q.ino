#include "GY6050.h"           //library for GYRO 
#include <Wire.h>
#include <Servo.h>

Servo myservo;  // create servo object to control a servo


int X = 0;
int Y = 0;
GY6050 gyro(0x68);              //to save GYRO data


void setup() {

  Wire.begin();            //initializing GYRO
  gyro.initialisation();
  delay(100);
  myservo.attach(9);
}

void loop() {
  X = map(gyro.refresh('A', 'X'), -90, 90, 0, 180);                //mapping the gyro data according to angle limitation of servo motor 
  Y = map(gyro.refresh('A', 'Y'), -90, 90, 0, 180);
  myservo.write(X);                                               //movement of Y axis will control servo
  delay(15);

}

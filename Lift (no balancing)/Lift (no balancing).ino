#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <Servo.h>
#include "LMotorController.h"
#include <IRremote.h>

#define CE_PIN 9
#define CSN_PIN 10
#define single_red 31
#define double_green 30
#define IR_RECEIVER_PIN 37

double motorSpeedFactorLeft = 1;
double motorSpeedFactorRight = 1;
int speed = 0;
int direction = 0;
int speedlimit = 255;
int turnspeedlimit = 50;
int leftSpeed = 0;
int rightSpeed = 0;

// L298N
int ENA = 3;
int IN1 = 5;
int IN2 = 4;
int IN3 = 7;
int IN4 = 6;
int ENB = 8;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

// RF receiver
double data[6];
RF24 radio(CE_PIN, CSN_PIN);
const byte address[6] = "00001";

Servo myservo1;
Servo myservo2;

IRrecv irrecv(IR_RECEIVER_PIN);
decode_results results;

void setup()
{
  Serial.begin(9600);

  pinMode(single_red, OUTPUT);
  pinMode(double_green, OUTPUT);

  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  myservo1.attach(40);
  myservo2.attach(41);

  irrecv.enableIRIn(); // Start the IR receiver
}

void loop() {

  if (irrecv.decode(&results)) {
    digitalWrite(single_red, HIGH);
    delay(50); // Change this value to adjust the LED blink duration
    digitalWrite(single_red, LOW);
    irrecv.resume(); // Receive the next value
  }

  if (radio.available()) {
    radio.read(&data, sizeof(data));

    if (data[4] == 0) {
      myservo1.write(0);
      myservo2.write(180);
    } 
    else if (data[5] == 0){
      myservo1.write(180);
      myservo2.write(0);
    }
    else if (data[4] == 1 && data[5] == 1) {
      myservo1.write(90);
      myservo2.write(90);
      if (data[2] >= 507 && data[2] <= 1023) {
        speed = map(data[2], 507, 1023, 0, speedlimit);
        leftSpeed = speed;
        rightSpeed = speed;
      } else if (data[2] >= 0 && data[2] <= 497) {
        speed = map(data[2], 0, 497, speedlimit, 0);
        leftSpeed = -speed;
        rightSpeed = -speed;
      } else if (data[3] >= 498 && data[3] <= 1023) {
        direction = map(data[3], 498, 1023, 0, turnspeedlimit);
        leftSpeed += direction;
        rightSpeed -= direction;
      } else if (data[3] >= 0 && data[3] <= 488) {
        direction = map(data[3], 0, 488, turnspeedlimit, 0);
        leftSpeed -= direction;
        rightSpeed += direction;
      } else {
        leftSpeed = 0;
        rightSpeed = 0;
      }
      motorController.move(leftSpeed, rightSpeed, 0);
    }
  }
}

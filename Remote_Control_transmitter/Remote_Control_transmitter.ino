#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10

int joyX = A0;
int joyY = A1;
int yjoyX = A3;
int yjoyY = A2;
int dataX;
int dataY;
int ydataX;
int ydataY;
double data[4];

RF24 radio(CE_PIN, CSN_PIN);                                 
const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MAX);
  radio.stopListening();
}



void loop() {
  dataX = analogRead(joyX);
  dataY = analogRead(joyY);
  ydataX = analogRead(yjoyX);
  ydataY = analogRead(yjoyY);
  data[0] = dataX;
  data[1] = dataY;
  data[2] = ydataX;
  data[3] = ydataY;

  Serial.print("Data X:"); Serial.println(dataX);
  Serial.print("Data Y:"); Serial.println(dataY);
  Serial.print("yData X:"); Serial.println(ydataX);
  Serial.print("yData Y:"); Serial.println(ydataY);
  radio.write(&data, sizeof(data));
  
  delay(100);
}
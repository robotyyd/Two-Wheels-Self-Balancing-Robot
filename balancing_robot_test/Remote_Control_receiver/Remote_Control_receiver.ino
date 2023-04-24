#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

#define CE_PIN 9
#define CSN_PIN 10

RF24 radio(CE_PIN, CSN_PIN);

const byte address[6] = "00001";

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  if (radio.available()) {
    int data[2];
    radio.read(&data, sizeof(data));
    
    Serial.print("A0 Value: ");
    Serial.print(data[0]);
    Serial.print(" A1 Value: ");
    Serial.println(data[1]);
  }
}
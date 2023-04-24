void setup() {
  Serial.begin(9600);
}

void loop() {
  long duration[2] = {0, 0};
  RCTime(24, 25, duration);
  Serial.println(duration[0]);	    // Display result of sensor1
  //Serial.println(duration[1]);	    // Display result of sensor2
  delay(250);			    // Wait 250ms
}

void RCTime(int sensor1, int sensor2, long* duration){
   pinMode(sensor1, OUTPUT);     // Make pin OUTPUT
   pinMode(sensor2, OUTPUT);     // Make pin OUTPUT
   digitalWrite(sensor1, HIGH);  // Pin HIGH (discharge capacitor)
   digitalWrite(sensor2, HIGH);  // Pin HIGH (discharge capacitor)
   delay(1);                      // Wait 1ms
   pinMode(sensor1, INPUT);      // Make pin INPUT
   pinMode(sensor2, INPUT);      // Make pin INPUT
   digitalWrite(sensor1, LOW);   // Turn off internal pullups
   digitalWrite(sensor2, LOW);   // Turn off internal pullups
   while(digitalRead(sensor1)){  // Wait for pin1 to go LOW
      duration[0]++;
   }
   while(digitalRead(sensor2)){  // Wait for pin2 to go LOW
      duration[1]++;
   }
}

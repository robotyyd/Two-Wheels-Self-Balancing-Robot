#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

MPU6050 mpu;

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// If you havent calibrated your Gyro you will want to do that NOW!!!!
// How to calibrate your gyro is easy
// position your balancing bot on a level surface and prop it up at its balancing point The MPU6050 should be really close to level also.
// install and run the MPU6050_calibration
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                       XA      YA      ZA      XG      YG      ZG
int MPUOffsets[6] = { -371,  -470,   1233,     -162,    29,     -23}; //<< put your Calibration numbers here
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


#define LED_PIN 12 // 
// MPU control/status vars
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double Input;
double YawMPU;// Actual reading from the MPU


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}
// ================================================================
// ===                      MPU DMP SETUP                       ===
// ================================================================
void MPU6050Connect() {
  static int MPUInitCntr = 0;
  // initialize device
  mpu.initialize(); // same
  // load and configure the DMP
  devStatus = mpu.dmpInitialize();// same

  if (devStatus != 0) {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)

    char * StatStr[5] { "No Error", "initial memory load failed", "DMP configuration updates failed", "3", "4"};

    MPUInitCntr++;

    Serial.print("MPU connection Try #");
    Serial.println(MPUInitCntr);
    Serial.print("DMP Initialization failed (code ");
    Serial.print(StatStr[devStatus]);
    Serial.println(")");

    if (MPUInitCntr >= 10) return; //only try 10 times
    delay(1000);
    MPU6050Connect(); // Lets try again
    return;
  }


  mpu.setXAccelOffset(MPUOffsets[0]);
  mpu.setYAccelOffset(MPUOffsets[1]);
  mpu.setZAccelOffset(MPUOffsets[2]);
  mpu.setXGyroOffset(MPUOffsets[3]);
  mpu.setYGyroOffset(MPUOffsets[4]);
  mpu.setZGyroOffset(MPUOffsets[5]);

  Serial.println(F("Enabling DMP..."));
  mpu.setDMPEnabled(true);
  // enable Arduino interrupt detection
  //Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 2 on the Uno)..."));
  //attachInterrupt(0, dmpDataReady, FALLING); //pin 2 on the Uno
  // Serial.println(F("Enabling interrupt detection (Arduino external interrupt pin 19 int4 on mega2560)..."));
  attachInterrupt(4, dmpDataReady, RISING); //pin 19 int4 on mega2560 ******************************
  mpuIntStatus = mpu.getIntStatus(); // Same
  // get expected DMP packet size for later comparison
  packetSize = mpu.dmpGetFIFOPacketSize();
  delay(1000); // Let it Stabalize
  mpu.resetFIFO(); // Clear fifo buffer
  mpu.getIntStatus();
  mpuInterrupt = false; // wait for next interrupt

}


void setup() {
  Serial.begin(9600); //115200
  while (!Serial) {
  }
    // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
#endif
MPU6050Connect();
  }

  void loop() {
    if (mpuInterrupt ) { // wait for MPU interrupt or extra packet(s) available
      // I did not alter GetDMP
      GetDMP(0); // if StartUP is true then GetDMP skipps the MPUMath()  and does not call the DirectDrive() preventing motors from engaging.
    }
    SerialPrint(); // this has a spam timer to only send to serial port once every 100ms
  }

  void MPUMath() {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    Input = (ypr[1] * 180 / M_PI) ;
    YawMPU = (ypr[0] * 180 / M_PI) ;
  }
  
  void GetDMP(bool Startup) { // Best version I have made so far
    mpuInterrupt = false;
    fifoCount = mpu.getFIFOCount();
    /*
    fifoCount is a 16-bit unsigned value. Indicates the number of bytes stored in the FIFO buffer.
    This number is in turn the number of bytes that can be read from the FIFO buffer and it is
    directly proportional to the number of samples available given the set of sensor data bound
    to be stored in the FIFO
    */

    // PacketSize = 42; refference in MPU6050_6Axis_MotionApps20.h Line 527
    // FIFO Buffer Size = 1024;
    uint16_t MaxPackets = 20;// 20*42=840 leaving us with  2 Packets (out of a total of 24 packets) left before we overflow.
    // If we overflow the entire FIFO buffer will be corrupt and we must discard it!

    // At this point in the code FIFO Packets should be at 1 99% of the time if not we need to look to see where we are skipping samples.
    if ((fifoCount % packetSize) || (fifoCount > (packetSize * MaxPackets)) || (fifoCount < packetSize)) { // we have failed Reset and wait till next time!
      digitalWrite(LED_PIN, LOW); // lets turn off the blinking light so we can see we are failing.
      Serial.println("Reset FIFO");
      if (fifoCount % packetSize) Serial.print("\t Packet corruption"); // fifoCount / packetSize returns a remainder... Not good! This should never happen if all is well.
      Serial.print("\tfifoCount "); Serial.print(fifoCount);
      Serial.print("\tpacketSize "); Serial.print(packetSize);

      mpuIntStatus = mpu.getIntStatus(); // reads MPU6050_RA_INT_STATUS       0x3A
      Serial.print("\tMPU Int Status "); Serial.print(mpuIntStatus , BIN);
      // MPU6050_RA_INT_STATUS       0x3A
      //
      // Bit7, Bit6, Bit5, Bit4          , Bit3       , Bit2, Bit1, Bit0
      // ----, ----, ----, FIFO_OFLOW_INT, I2C_MST_INT, ----, ----, DATA_RDY_INT

      /*
      Bit4 FIFO_OFLOW_INT: This bit automatically sets to 1 when a FIFO buffer overflow interrupt has been generated.
      Bit3 I2C_MST_INT: This bit automatically sets to 1 when an I2C Master interrupt has been generated. For a list of I2C Master interrupts, please refer to Register 54.
      Bit1 DATA_RDY_INT This bit automatically sets to 1 when a Data Ready interrupt is generated.
      */
      if (mpuIntStatus & B10000) { //FIFO_OFLOW_INT
        Serial.print("\tFIFO buffer overflow interrupt ");
      }
      if (mpuIntStatus & B1000) { //I2C_MST_INT
        Serial.print("\tSlave I2c Device Status Int ");
      }
      if (mpuIntStatus & B1) { //DATA_RDY_INT
        Serial.print("\tData Ready interrupt ");
      }
      Serial.println();
      //I2C_MST_STATUS
      //PASS_THROUGH, I2C_SLV4_DONE,I2C_LOST_ARB,I2C_SLV4_NACK,I2C_SLV3_NACK,I2C_SLV2_NACK,I2C_SLV1_NACK,I2C_SLV0_NACK,
      mpu.resetFIFO();// clear the buffer and start over
      mpu.getIntStatus(); // make sure status is cleared we will read it again.
    } else {
      while (fifoCount  >= packetSize) { // Get the packets until we have the latest!
        if (fifoCount < packetSize) break; // Something is left over and we don't want it!!!
        mpu.getFIFOBytes(fifoBuffer, packetSize); // lets do the magic and get the data
        fifoCount -= packetSize;
      }
      if (!Startup) MPUMath(); // <<<<<<<<<<<<<<<<<<<<<<<<<<<< On success MPUMath() <<<<<<<<<<<<<<<<<<<
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink the Light
      if (fifoCount > 0) mpu.resetFIFO(); // clean up any leftovers Should never happen! but lets start fresh if we need to. this should never happen.
    }
  }



  void SerialPrint() {
    static int sx = 0;
    char s[4] = {'/', '-', '\\', '|'};
    for (static long QTimer = millis(); (long)( millis() - QTimer ) >= 100; QTimer = millis() ) {
      if (sx > 3)sx = 0;
      Serial.print(s[sx++]);
      Serial.print("\t Pitch "); Serial.print(Input);
      Serial.print("\t Yaw "); Serial.print(YawMPU);
      Serial.println();
    }

  }

//MOTOR SETUP


/*motor 1 top motor
  pin1 LOW pin2 HIGH = counterclockwise
  pin1 HIGH pin2 LOW = clockwise
  pin1=pin2 FAST STOP
  pwm = 0 slow rolling stop
*/
int motor1pin1 = 3;
int motor1pin2 = 4;
int motor1pwm = 5;

double angle =0;
double constAng = 5;

//PID SETUP

#include <PID_v1.h>


//Define Variables we'll be connecting to
double Setpoint, Input, Output;
double Speed =0;
//Specify the links and initial tuning parameters
double divisor = 1;
double divisor2 = 2;
double aggKp = .2 / divisor, aggKi = .4 / divisor, aggKd = 0 / divisor;
double consKp = .2 / divisor2, consKi = .4 / divisor2, consKd = 0 / divisor2;
PID myPID(&Input, &Output, &Setpoint, consKp, consKi, consKd, DIRECT);
int latch = 0;
String inputString;
// tuning was around 80 250 .4 to get good results or maybe 80 180 .4 p was 80 i was ? d was .4
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps_V6_12.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */



#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

double dividor =1;

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================
unsigned long timer = 0;
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  Wire.setWireTimeout(3000, true);
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(400, true);
#endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(-1668);
  mpu.setYGyroOffset(-389);
  mpu.setZGyroOffset(-4);
  mpu.setXAccelOffset(172);
  mpu.setYAccelOffset(-1168);
  mpu.setZAccelOffset(1276);
  mpu.PrintActiveOffsets();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
   // mpu.CalibrateAccel(6);
   // mpu.CalibrateGyro(6);
    //Serial.println();
    //mpu.PrintActiveOffsets();
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);



  printPID();

  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  digitalWrite(motor1pwm, LOW);
  Setpoint = 88.5;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-255, 255);
  myPID.SetSampleTime(20);
}



// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO

  if (Serial.available()) {
    inputString = "";
    while (Serial.available()) {
      // get the new byte:
      char inChar = (char)Serial.read();
      // add it to the inputString:
      inputString += inChar;
      // if the incoming character is a newline, set a flag
      // so the main loop can do something about it:
      delay(1);
    }
    if (inputString.indexOf("new")>-1) {
      while (!Serial.available()) {

      }
      delay(5);
      double kp = Serial.parseFloat();
      while (!Serial.available()) {

      }
      delay(5);
      double ki = Serial.parseFloat();
      while (!Serial.available()) {

      }
      delay(5);
      double kd = Serial.parseFloat();
      aggKp = kp;
      aggKi = ki;
      aggKd = kd;
      consKp = kp / dividor;
      consKi = ki / dividor;
      consKd = kd / dividor;
      printPID();
    }
    if (inputString.indexOf("SampleTime")>-1) {
      while (!Serial.available()) {

      }
      delay(5);
      myPID.SetSampleTime(Serial.parseInt());
      Serial.print("Sample Time: ");
      Serial.println(myPID.GetSampleTime());

    }
    if (inputString.indexOf("div")>-1) {
      //Serial.clear();
      while (!Serial.available()) {
      }
      delay(5);
      dividor=Serial.parseFloat();
      Serial.print("Dividor: ");
      Serial.println(dividor);

    }
    if (inputString.indexOf("Angle")>-1) {
      //Serial.clear();
      while (!Serial.available()) {
      }
      delay(5);
      constAng=Serial.parseFloat();
      Serial.print("Constant Angle: ");
      Serial.println(constAng);

    }
    if (inputString.indexOf("Setpoint")>-1) {
      while (!Serial.available()) {
      }
      delay(5);
      Setpoint=Serial.parseFloat();
      Serial.print("Setpoint: ");
      Serial.println(Setpoint);

    }
  }
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    angle = ypr[2]*180 / M_PI;
   Serial.print("Data:");
   Serial.println(angle);
    Input = angle;
    double gap = abs(Setpoint - Input); //distance away from setpoint
    if (gap > 25) {
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor1pwm, HIGH);
     Serial.println("Output:STOP");
      latch = 1;
      myPID.SetMode(MANUAL);
      return;
    }
    if (latch == 1) {
      if (gap > 5) {
      Serial.println("Output:STOP");
        return;
      } else {
        latch = 0;
        digitalWrite(motor1pwm, LOW);
        Output = 0;
        myPID.SetMode(AUTOMATIC);
      }
    }
    if (gap < constAng)
    { //we're close to setpoint, use conservative tuning parameters
      myPID.SetTunings(consKp, consKi, consKd);
    }
    else
    {
      //we're far from setpoint, use aggressive tuning parameters
      myPID.SetTunings(aggKp, aggKi, aggKd);
    }
    myPID.Compute();
    if((Speed>128&&Output<100&&Output>-100)||(Speed<-128&&Output>-100&&Output<100)){
      //digitalWrite(motor1pin1, HIGH);
      //digitalWrite(motor1pin2, HIGH);
      //digitalWrite(motor1pwm, HIGH);
      //delay(25);
    }
    Speed=Output;
    if (Output < 0) {
      digitalWrite(motor1pin1, HIGH);
      digitalWrite(motor1pin2, LOW);
    } else {
      digitalWrite(motor1pin2, HIGH);
      digitalWrite(motor1pin1, LOW);
    }

    analogWrite(motor1pwm, abs(Output));
    Serial.print("Output:");
   Serial.println(Output);
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}


void printPID() {
  Serial.print("Current Settings|KP: ");
  Serial.print(aggKp);
  Serial.print(" KI: ");
  Serial.print(aggKi);
  Serial.print(" KD: ");
  Serial.println(aggKd);
}

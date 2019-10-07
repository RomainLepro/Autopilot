//Aircraft Stabilizer
//Servo 
#include <Servo.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "I2Cdev.h"
#include <PinChangeInt.h>

#define SERIAL_PORT_SPEED 115200

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

Servo myservoRoll; // Roll
Servo myservoPitch; // Pitch
Servo myservoYaw; 
Servo myservoThrotle; 

//list of pwm pin
#define roll 8
#define pitch 9
#define throtle 10
#define yaw 11

//var used by receiver

#define RC_NUM_CHANNELS  5

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4

#define RC_CH1_INPUT 3
#define RC_CH2_INPUT 4
#define RC_CH3_INPUT 5
#define RC_CH4_INPUT 6
#define RC_CH5_INPUT 7

int rc_values[RC_NUM_CHANNELS];
uint32_t rc_start[RC_NUM_CHANNELS];
volatile uint16_t rc_shared[RC_NUM_CHANNELS];

#define env 100

MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

#define OUTPUT_READABLE_YAWPITCHROLL 

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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// ================================================================
// ===               INTERRUPT DETECTION FOR RECIEVER           ===
// ================================================================

void rc_read_values() {
  noInterrupts();
  memcpy(rc_values, (const void *)rc_shared, sizeof(rc_shared));
  interrupts();
}

void calc_input(uint8_t channel, uint8_t input_pin) {
  if (digitalRead(input_pin) == HIGH) {
    rc_start[channel] = micros();
  } else {
    uint16_t rc_compare = (uint16_t)(micros() - rc_start[channel]);
    rc_shared[channel] = rc_compare;
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

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
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

//Attach servo

  myservoPitch.attach(pitch); // Attach Y servo to pin 9
  myservoRoll.attach(roll);// Attach X servo to pin 10
  myservoYaw.attach(yaw);
  myservoThrotle.attach(throtle);

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);

  PCintPort::attachInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  PCintPort::attachInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  PCintPort::attachInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  PCintPort::attachInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  PCintPort::attachInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);

    Serial.begin(SERIAL_PORT_SPEED);

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(-1115);
    mpu.setYGyroOffset(-81);
    mpu.setZGyroOffset(-2);
    mpu.setZAccelOffset(516);
    //           X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
    //OFFSETS    -3678,     343,     516,   -1115,     -81,     -2   100T

    
    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
      //calculate the ofset of the gyro/axxelero
        mpu.CalibrateAccel(8);
        mpu.CalibrateGyro(8);
        mpu.PrintActiveOffsets();
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
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
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  int K1 = -4;
  int K2 = -2;
  int K3 = -1;
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
      rc_read_values();

      if (abs(rc_values[RC_CH5]-1500)<env || rc_values[RC_CH5]==0){
        // 2 = roll  1 = pitch  0 = yaw
        // ATTENTION ! l'angle de comande yaw devrait etre remis a 0
          myservoRoll.write(int(K1*int(ypr[2] * -180/M_PI)+90 + K3 *int(ypr[0] * -180/M_PI))); 
          myservoPitch.write(K2*int(ypr[1] * 180/M_PI)+90); 
          myservoThrotle.write(int((rc_values[RC_CH3]-1000)*18/100)); 
          myservoYaw.write(K3*int(ypr[0] * -180/M_PI)+90); 
          
          Serial.print(int(K1*int(ypr[2] * -180/M_PI)+90 + K3 *int(ypr[0] * -180/M_PI)));Serial.print("\t");
          Serial.print(ypr[1] * 180/M_PI);Serial.print("\t");
          Serial.print(ypr[2] * 180/M_PI);Serial.println("\t");
        
          }
      else{
         myservoRoll.write(int((rc_values[RC_CH1]-1000)*18/100)); 
         myservoPitch.write(int((rc_values[RC_CH2]-1000)*18/100)); 
         myservoThrotle.write(int((rc_values[RC_CH3]-1000)*18/100)); 
         myservoYaw.write(int((rc_values[RC_CH4]-1000)*18/100)); 
         
         Serial.print(rc_values[RC_CH1]);Serial.print("\t");
         Serial.print(rc_values[RC_CH2]);Serial.print("\t");
         Serial.print(rc_values[RC_CH3]);Serial.print("\t");
         Serial.print(rc_values[RC_CH4]);Serial.print("\t");
         Serial.print(rc_values[RC_CH5]);Serial.println("\t");
      }
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // calculate euler angle
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
        #endif

        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    } 
}

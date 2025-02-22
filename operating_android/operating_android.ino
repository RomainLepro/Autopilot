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
#define roll 9
#define pitch 10
#define throtle 11
#define yaw 12

//var used by receiver

#define RC_NUM_CHANNELS  6

#define RC_CH1  0
#define RC_CH2  1
#define RC_CH3  2
#define RC_CH4  3
#define RC_CH5  4
#define RC_CH6  5

#define RC_CH1_INPUT 3
#define RC_CH2_INPUT 4
#define RC_CH3_INPUT 5
#define RC_CH4_INPUT 6
#define RC_CH5_INPUT 7
#define RC_CH6_INPUT 8

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

//O of the plane
int Ox = 0,Oy = 0,Oz = 0; 
//O of the radio
int Oradiox = 0,Oradioy = 0,Oradioz = 0;
int Xthrotle = 0;
int OVRA = 0;
//Ocomande
int Ocz1 = 0;
float Ocomandex,Ocomandey,Ocomandez;
//Oservo
float Oservox,Oservoy,Oservoz;


int DOx = 0, DOy = 0, DOz = 0;
int IDOx = 0,IDOy = 0,IDOz = 0;
float Kx1 = 2.0,Ky1 = 2.0,Kz1 = 1.0;
float Kx2 = 0.2,Ky2 = 0.2,Kz2 = 0.2;



uint8_t dt;
uint16_t t;


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
    rc_shared[channel] = (uint16_t)(micros() - rc_start[channel]);
  }
}

void calc_ch1() { calc_input(RC_CH1, RC_CH1_INPUT); }
void calc_ch2() { calc_input(RC_CH2, RC_CH2_INPUT); }
void calc_ch3() { calc_input(RC_CH3, RC_CH3_INPUT); }
void calc_ch4() { calc_input(RC_CH4, RC_CH4_INPUT); }
void calc_ch5() { calc_input(RC_CH5, RC_CH5_INPUT); }
void calc_ch6() { calc_input(RC_CH6, RC_CH6_INPUT); }


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                         ms to deg                        ===
// ================================================================

int degres(int t){
  return (t-1500)*18/100; // return a falue in degres centerded a 90 degres
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
  
//initialisation for interupts

  pinMode(RC_CH1_INPUT, INPUT);
  pinMode(RC_CH2_INPUT, INPUT);
  pinMode(RC_CH3_INPUT, INPUT);
  pinMode(RC_CH4_INPUT, INPUT);
  pinMode(RC_CH5_INPUT, INPUT);
  pinMode(RC_CH6_INPUT, INPUT);

  PCintPort::attachInterrupt(RC_CH1_INPUT, calc_ch1, CHANGE);
  PCintPort::attachInterrupt(RC_CH2_INPUT, calc_ch2, CHANGE);
  PCintPort::attachInterrupt(RC_CH3_INPUT, calc_ch3, CHANGE);
  PCintPort::attachInterrupt(RC_CH4_INPUT, calc_ch4, CHANGE);
  PCintPort::attachInterrupt(RC_CH5_INPUT, calc_ch5, CHANGE);
  PCintPort::attachInterrupt(RC_CH6_INPUT, calc_ch6, CHANGE);

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
  
    // if programming failed, don't try to do anything
    if (!dmpReady) return;
    // wait for MPU interrupt or extra packet(s) available
     while (!mpuInterrupt && fifoCount < packetSize) {
      rc_read_values();
      //O of the plane
      Ox = ypr[2] * -180/M_PI; 
      Oy = ypr[1] * 180/M_PI;
      Oz = ypr[0] * -180/M_PI;    
      //O of the radio
      Oradiox = degres(rc_values[RC_CH1]);
      Oradioy = degres(rc_values[RC_CH2]);
      Oradioz = degres (rc_values[RC_CH4]);
      Xthrotle = degres(rc_values[RC_CH3]);
      OVRA = degres(rc_values[RC_CH6]);

      dt = t-millis();
      t += dt;
    
      if (abs(rc_values[RC_CH5]-2000)<env ){//switch high
        // full radio control 
        Ocomandex += 0.05 * Oradiox * dt * 0.001; 
        Ocomandey += 0.05 * Oradioy * dt * 0.001; 
        Ocomandez += 0.05 * Oradioz * dt * 0.001;
        // 0.1 => max 90 degs/s
          }
          
      else if(abs(rc_values[RC_CH5]-1000)<env){//switch low
        //mixed control
        Ocomandex = 0.5*Oradiox;
        Ocomandey = 0.5*Oradioy;
        Ocomandez = 0.5*Oradioz;
        // 0.5=> max angle = 45 deg
          } 
          
      else{//switch mid or no signal
        //full control from the gyro    
        Ocomandex = 0;
        Ocomandey = 0;
 
        Ocz1 = float(analogRead(A3))*180.0/1024.0-90;
        Ocomandez += 0.01*(Ocz1-Ocomandez);
      }

      
      DOx = Ocomandex-Ox;
      IDOx +=  DOx * dt/1000 ;
      if (IDOx> 30.0/Kx2 ){//limit integral to 30 degs
        IDOx = 30.0/Kx2;
      }
      else if(IDOx< -30.0/Kx2 ){
        IDOx = -30.0/Kx2;
      }
      Oservox =  Kx1 * DOx +  Kx2 * IDOx;
      if (Oservox>90){Oservox = 90;}//limit servo range
      else if (Oservox<-90){Oservox = -90;}

      
      DOy = Ocomandey-Oy;
      IDOy +=  DOy * dt/1000 ;
      if (IDOy> 30.0/Ky2 ){//limit integral to 30 degs
        IDOy = 30.0/Ky2;
      }
      else if(IDOy< -30.0/Ky2 ){
        IDOy = -30.0/Ky2;
      }
      Oservoy =  Ky1 * DOy +  Ky2 * IDOy;

      
      if (Oservoy>90){Oservoy = 90;}//limit servo range
      else if (Oservoy<-90){Oservoy = -90;}
      DOz = Ocomandez-Oz;
      IDOz +=  DOz * dt/1000 ;
      if (IDOz> 30.0/Kz2 ){//limit integral to 30 degs
        IDOz = 30.0/Kz2;
      }
      else if(IDOz< -30.0/Kz2 ){
        IDOz = -30.0/Kz2;
      }
      Oservoz =  Kz1 * DOz +  Kz2 * IDOz;
      if (Oservoz>90){Oservoz = 90;}//limit servo range
      else if (Oservoz<-90){Oservoz = -90;}


      //Serial.print("OX : ");Serial.print(Ox);Serial.print("    ");
      //Serial.print("OservoX : ");Serial.print(Oservox,0);Serial.println("    "); 
      //Serial.print("OY : ");Serial.print(Oy);Serial.print("    ");
      //Serial.print("OservoY : ");Serial.print(Oservoy,0);Serial.println("    "); 
      //Serial.print("OZ : ");Serial.print(Oz);Serial.print("    ");
      Serial.print("OZ co: ");Serial.print(Ocomandez);Serial.print("    ");   
      //Serial.print("OservoZ : ");Serial.print(Oservoz,0);Serial.println("    "); 
      Serial.println("");
      
      myservoYaw.write(Oservoz+90); 
      myservoPitch.write(Oservoy+90); 
      myservoRoll.write(Oservox+90); 

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

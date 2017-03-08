#include <SoftwareSerial.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

float yaw;
float pitch;
float roll;

uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}


double P_IMU=30;
double I_IMU=5;
double D_IMU=0.25;

double setpoint_IMU,input_IMU,output_IMU;  // variables for imu input
PID pid_IMU(&input_IMU,&output_IMU,&setpoint_IMU,P_IMU,I_IMU,D_IMU,DIRECT);// pid for imu input

double P_upper_A_side_side=10;
double I_upper_A_side_side=0;
double D_upper_A_side_side=0;

double setpoint_upper_A_side_side, input_upper_A_side_side, output_upper_A_side_side;
PID pid_upper_A_side_side(&setpoint_upper_A_side_side, &input_upper_A_side_side, &output_upper_A_side_side, P_upper_A_side_side, I_upper_A_side_side, D_upper_A_side_side, DIRECT); // pid for upper pot whose setpoint is output for pid_IMU

double P_lower_A_side_side=10;
double I_lower_A_side_side=0;
double D_lower_A_side_side=0;

double setpoint_lower_A_side_side, input_lower_A_side_side, output_lower_A_side_side;
PID pid_lower_A_side_side(&setpoint_lower_A_side_side, &input_lower_A_side_side, &output_lower_A_side_side, P_lower_A_side_side, I_lower_A_side_side, D_lower_A_side_side, DIRECT); // pid for lower pot whose setpoint is output for pid_IMU

double P_upper_A_front_back=10;
double I_upper_A_front_back=0;
double D_upper_A_front_back=0;

double setpoint_upper_A_front_back, input_upper_A_front_back, output_upper_A_front_back;
PID pid_upper_A_front_back(&setpoint_upper_A_front_back, &input_upper_A_front_back, &output_upper_A_front_back, P_upper_A_front_back, I_upper_A_front_back, D_upper_A_front_back, DIRECT); //pid for upper pot (front-back motion)

double P_lower_A_front_back=10;
double I_lower_A_front_back=0;
double D_lower_A_front_back=0;

double setpoint_lower_A_front_back, input_lower_A_front_back, output_lower_A_front_back;
PID pid_lower_A_front_back(&setpoint_lower_A_front_back, &input_lower_A_front_back, &output_lower_A_front_back, P_lower_A_front_back, I_lower_A_front_back, D_lower_A_front_back, DIRECT); //pid for lower pot (front-back motion)

double P_upper_B_side_side=10;
double I_upper_B_side_side=0;
double D_upper_B_side_side=0;

double setpoint_upper_B_side_side, input_upper_B_side_side, output_upper_B_side_side;
PID pid_upper_B_side_side(&setpoint_upper_B_side_side, &input_upper_B_side_side, &output_upper_B_side_side, P_upper_B_side_side, I_upper_B_side_side, D_upper_B_side_side, DIRECT); // pid for upper pot whose setpoint is output for pid_IMU

double P_lower_B_side_side=10;
double I_lower_B_side_side=0;
double D_lower_B_side_side=0;

double setpoint_lower_B_side_side, input_lower_B_side_side, output_lower_B_side_side;
PID pid_lower_B_side_side(&setpoint_lower_B_side_side, &input_lower_B_side_side, &output_lower_B_side_side, P_lower_B_side_side, I_lower_B_side_side, D_lower_B_side_side, DIRECT); // pid for lower pot whose setpoint is output for pid_IMU

double P_upper_B_front_back=10;
double I_upper_B_front_back=0;
double D_upper_B_front_back=0;

double setpoint_upper_B_front_back, input_upper_B_front_back, output_upper_B_front_back;
PID pid_upper_B_front_back(&setpoint_upper_B_front_back, &input_upper_B_front_back, &output_upper_B_front_back, P_upper_B_front_back, I_upper_B_front_back, D_upper_B_front_back, DIRECT); //pid for upper pot (front-back motion)

double P_lower_B_front_back=10;
double I_lower_B_front_back=0;
double D_lower_B_front_back=0;

double setpoint_lower_B_front_back, input_lower_B_front_back, output_lower_B_front_back;
PID pid_lower_B_front_back(&setpoint_lower_B_front_back, &input_lower_B_front_back, &output_lower_B_front_back, P_lower_B_front_back, I_lower_B_front_back, D_lower_B_front_back, DIRECT); //pid for lower pot (front-back motion)

char motorControl='m';
char upDownTest='u';
char walkControl='w';
char right='r';
char left='l';
char ident;

int motorAUpper1speed=0;
int motorAUpper2speed=0;
int motorALower1speed=0;
int motorALower2speed=0;
int motorBUpper1speed=0;
int motorBUpper2speed=0;
int motorBLower1speed=0;
int motorBLower2speed=0;

int setpoint_up_down=0;

  int Tx=11;//BLuetooth Rx
  int Rx=10;//Bluetooth TX

 SoftwareSerial bluetooth(Rx,Tx) ;

 
void setup() {

   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

 mpu.initialize();

 devStatus = mpu.dmpInitialize();

// supply your own gyro offsets here, scaled for min sensitivity
mpu.setXGyroOffset(220);
mpu.setYGyroOffset(76);
mpu.setZGyroOffset(-85);
mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        //Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    }  
 
 bluetooth.begin(9600);
 Serial.begin(115200);

 pid_IMU.SetMode(AUTOMATIC);
 pid_IMU.SetOutputLimits(-255,255);
 pid_IMU.SetSampleTime(20);

 pid_upper_A_side_side.SetMode(AUTOMATIC);
 pid_upper_A_side_side.SetOutputLimits(-255,255);
 pid_upper_A_side_side.SetSampleTime(20);

 pid_lower_A_side_side.SetMode(AUTOMATIC);
 pid_lower_A_side_side.SetOutputLimits(-255,255);
 pid_lower_A_side_side.SetSampleTime(20);

 pid_upper_A_front_back.SetMode(AUTOMATIC);
 pid_upper_A_front_back.SetOutputLimits(-255,255);
 pid_upper_A_front_back.SetSampleTime(20);

 pid_lower_A_front_back.SetMode(AUTOMATIC);
 pid_lower_A_front_back.SetOutputLimits(-255,255);
 pid_lower_A_front_back.SetSampleTime(20);

 pid_upper_B_side_side.SetMode(AUTOMATIC);
 pid_upper_B_side_side.SetOutputLimits(-255,255);
 pid_upper_B_side_side.SetSampleTime(20);

 pid_lower_B_side_side.SetMode(AUTOMATIC);
 pid_lower_B_side_side.SetOutputLimits(-255,255);
 pid_lower_B_side_side.SetSampleTime(20);

 pid_upper_B_front_back.SetMode(AUTOMATIC);
 pid_upper_B_front_back.SetOutputLimits(-255,255);
 pid_upper_B_front_back.SetSampleTime(20);

 pid_lower_B_front_back.SetMode(AUTOMATIC);
 pid_lower_B_front_back.SetOutputLimits(-255,255);
 pid_lower_B_front_back.SetSampleTime(20);
 
}



void loop() { 
  pid_Imu();
 
    if(bluetooth.available()>0){
      ident=bluetooth.read();

      switch(ident){
        
        case 'm':
        
        motorAUpper1speed=bluetooth.parseInt();
        motorAUpper2speed=bluetooth.parseInt();
        motorALower1speed=bluetooth.parseInt();
        motorALower2speed=bluetooth.parseInt();
        motorBUpper1speed=bluetooth.parseInt();
        motorBUpper2speed=bluetooth.parseInt();
        motorBLower1speed=bluetooth.parseInt();
        motorBLower2speed=bluetooth.parseInt();
        
          Serial.println(motorControl);
          Serial.println(motorAUpper1speed);
          Serial.println(motorAUpper2speed);
          Serial.println(motorALower1speed);
          Serial.println(motorALower2speed);
          Serial.println(motorBUpper1speed);
          Serial.println(motorBUpper2speed);
          Serial.println(motorBLower1speed);
          Serial.println(motorBLower2speed);
          Serial.println(" ");
          break;

        case 'u':
        
        setpoint_up_down=bluetooth.parseInt();
        setpoint_upper_A_front_back=setpoint_up_down;
        setpoint_lower_A_front_back=setpoint_up_down;
        setpoint_upper_B_front_back=setpoint_up_down;
        setpoint_lower_B_front_back=setpoint_up_down;

          Serial.println(upDownTest);
          Serial.println(setpoint_up_down);
          Serial.println(" ");
          
        break;

        case 'w':
        
        setpoint_upper_A_front_back=bluetooth.parseInt();
        setpoint_lower_A_front_back=bluetooth.parseInt();
        setpoint_upper_B_front_back=bluetooth.parseInt();
        setpoint_lower_B_front_back=bluetooth.parseInt();

          Serial.println(walkControl);
          Serial.println(setpoint_upper_A_front_back);
          Serial.println(setpoint_lower_A_front_back);
          Serial.println(setpoint_upper_B_front_back);
          Serial.println(setpoint_lower_B_front_back);
          Serial.println(" ");

          pid_Imu();
          
        break;

        case 'r':

        setpoint_upper_A_front_back=bluetooth.parseInt();
        setpoint_lower_A_front_back=setpoint_upper_A_front_back;
        setpoint_upper_B_front_back=0;
        setpoint_lower_B_front_back=0;

          Serial.println(right);
          Serial.println(setpoint_upper_A_front_back);
          Serial.println(" ");

          pid_Imu();

        break;

        case 'l':

        setpoint_upper_B_front_back=bluetooth.parseInt();
        setpoint_lower_B_front_back=setpoint_upper_A_front_back;
        setpoint_upper_A_front_back=0;
        setpoint_lower_A_front_back=0;

          Serial.println(left);
          Serial.println(setpoint_upper_B_front_back);
          Serial.println(" ");

          pid_Imu();

        break;
          
        }
     
      }
      delay(10);

}

void pid_Imu(){

  // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
        // other program behavior stuff here
        // .
        // .
        // .
        // if you are really paranoid you can frequently test in between other
        // stuff to see if mpuInterrupt is true, and if so, "break;" from the
        // while() loop to immediately process the MPU data
        // .
        // .
        // .
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
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            yaw=ypr[0]*180/M_PI;
            pitch = ypr[1]* 180/M_PI;
            roll = ypr[2]* 180/M_PI;
        #endif

        /*Serial.print('y');   
        Serial.print (",");
        Serial.print(pitch,4);
        Serial.print (",");
        Serial.print(roll,4);
        Serial.print('y'); */  

    }
  }

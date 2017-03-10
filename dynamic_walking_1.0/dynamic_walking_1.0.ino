#include <SoftwareSerial.h>
#include <I2Cdev.h>
#include <MPU6050_6Axis_MotionApps20.h>
#include <PID_v1.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL
#define OUTPUT_TEAPOT

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

int motorAUpper1Speed=0;
int motorAUpper2Speed=0;
int motorALower1Speed=0;
int motorALower2Speed=0;
int motorBUpper1Speed=0;
int motorBUpper2Speed=0;
int motorBLower1Speed=0;
int motorBLower2Speed=0;

int motorAUpper1_pwm=2;
int motorAUpper2_pwm=3;
int motorALower1_pwm=4;
int motorALower2_pwm=5;
int motorBUpper1_pwm=6;
int motorBUpper2_pwm=7;
int motorBLower1_pwm=8;
int motorBLower2_pwm=9;

int motorAUpper1_cloc=30;
int motorAUpper1_anticloc=31;
int motorAUpper2_cloc=32;
int motorAUpper2_anticloc=33;
int motorALower1_cloc=34;
int motorALower1_anticloc=35;
int motorALower2_cloc=36;
int motorALower2_anticloc=37;
int motorBUpper1_cloc=38;
int motorBUpper1_anticloc=39;
int motorBUpper2_cloc=40;
int motorBUpper2_anticloc=41;
int motorBLower1_cloc=42;
int motorBLower1_anticloc=43;
int motorBLower2_cloc=44;
int motorBLower2_anticloc=45;


int pot_A_upper_side_side;
int pot_A_lower_side_side;
int pot_A_upper_front_back;
int pot_A_lower_front_back;
int pot_B_upper_side_side;
int pot_B_lower_side_side;
int pot_B_upper_front_back;
int pot_B_lower_front_back;

int pot_A_upper_side_side_pin= A0 ;
int pot_A_lower_side_side_pin=A1;
int pot_A_upper_front_back_pin=A2;
int pot_A_lower_front_back_pin=A3;
int pot_B_upper_side_side_pin=A4;
int pot_B_lower_side_side_pin=A5;
int pot_B_upper_front_back_pin=A6;
int pot_B_lower_front_back_pin=A7;

int INTERRUPT_PIN=2;

int previousMillis=0;
int currentMillis=0;

int pot_A_upper_side_side_offset=0;
int pot_A_lower_side_side_offset=0;
int pot_A_upper_front_back_offset=0;
int pot_A_lower_front_back_offset=0;
int pot_B_upper_side_side_offset=0;
int pot_B_lower_side_side_offset=0;
int pot_B_upper_front_back_offset=0;
int pot_B_lower_front_back_offset=0;

int setpoint_up_down=0;

  int Tx=11;//BLuetooth Rx
  int Rx=10;//Bluetooth TX

 SoftwareSerial bluetooth(Rx,Tx) ;

 
void setup() {

   #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz). Comment this line if having compilation difficulties with TWBR.
        Wire.setClock(400000);
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

Serial.begin(115200);
while(!Serial);

 mpu.initialize();
 pinMode(INTERRUPT_PIN, INPUT);
 while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read());

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

pinMode(motorAUpper1_pwm,OUTPUT);
pinMode(motorAUpper2_pwm,OUTPUT);
pinMode(motorALower1_pwm,OUTPUT);
pinMode(motorALower2_pwm,OUTPUT);
pinMode(motorBUpper1_pwm,OUTPUT);
pinMode(motorBUpper2_pwm,OUTPUT);
pinMode(motorBLower1_pwm,OUTPUT);
pinMode(motorBLower2_pwm,OUTPUT);

pinMode(motorAUpper1_cloc,OUTPUT);
pinMode(motorAUpper2_cloc,OUTPUT);
pinMode(motorALower1_cloc,OUTPUT);
pinMode(motorALower2_cloc,OUTPUT);
pinMode(motorBUpper1_cloc,OUTPUT);
pinMode(motorBUpper2_cloc,OUTPUT);
pinMode(motorBLower1_cloc,OUTPUT);
pinMode(motorBLower2_cloc,OUTPUT);

pinMode(motorAUpper1_anticloc,OUTPUT);
pinMode(motorAUpper2_anticloc,OUTPUT);
pinMode(motorALower1_anticloc,OUTPUT);
pinMode(motorALower2_anticloc,OUTPUT);
pinMode(motorBUpper1_anticloc,OUTPUT);
pinMode(motorBUpper2_anticloc,OUTPUT);
pinMode(motorBLower1_anticloc,OUTPUT);
pinMode(motorBLower2_anticloc,OUTPUT);

 
}



void loop() { 
 
    if(bluetooth.available()>0){
      ident=bluetooth.read();

      switch(ident){
        
        case 'm':
        
        motorAUpper1Speed=bluetooth.parseInt();
        motorAUpper2Speed=bluetooth.parseInt();
        motorALower1Speed=bluetooth.parseInt();
        motorALower2Speed=bluetooth.parseInt();
        motorBUpper1Speed=bluetooth.parseInt();
        motorBUpper2Speed=bluetooth.parseInt();
        motorBLower1Speed=bluetooth.parseInt();
        motorBLower2Speed=bluetooth.parseInt();
        
          Serial.println(motorControl);
          Serial.println(motorAUpper1Speed);
          Serial.println(motorAUpper2Speed);
          Serial.println(motorALower1Speed);
          Serial.println(motorALower2Speed);
          Serial.println(motorBUpper1Speed);
          Serial.println(motorBUpper2Speed);
          Serial.println(motorBLower1Speed);
          Serial.println(motorBLower2Speed);
          Serial.println(" ");

          motorSpeed();
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

          previousMillis=millis();
          currentMillis=previousMillis;

          while(currentMillis-previousMillis > 100){
           currentMillis+=20;
          
          input_upper_A_front_back=analogRead(pot_A_upper_front_back_pin) - pot_A_upper_front_back_offset;
          input_lower_A_front_back=analogRead(pot_A_lower_front_back_pin) - pot_A_lower_front_back_offset;
          input_upper_B_front_back=analogRead(pot_B_upper_front_back_pin) - pot_B_upper_front_back_offset;
          input_lower_B_front_back=analogRead(pot_B_lower_front_back_pin) - pot_B_lower_front_back_offset;
          
          pid_upper_A_front_back.Compute();
          pid_lower_A_front_back.Compute();
          pid_upper_B_front_back.Compute();
          pid_lower_B_front_back.Compute();

          motorAUpper1Speed=output_upper_A_front_back;
          motorAUpper2Speed=output_upper_A_front_back;
          motorALower1Speed=output_lower_A_front_back;
          motorALower2Speed=output_lower_A_front_back;
          motorBUpper1Speed=output_upper_B_front_back;
          motorBUpper2Speed=output_upper_B_front_back;
          motorBLower1Speed=output_lower_B_front_back;
          motorBLower2Speed=output_lower_B_front_back;

          motorSpeed();
        }
          
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
          
          for(int i=0;i<6;i++) {

          previousMillis=millis();
          currentMillis=previousMillis;

          while(currentMillis-previousMillis>100){
            currentMillis+=20;

            pid_Imu(0);
            
            input_upper_A_front_back=analogRead(pot_A_upper_front_back_pin) - pot_A_upper_front_back_offset;
            input_lower_A_front_back=analogRead(pot_A_lower_front_back_pin) - pot_A_lower_front_back_offset;
            pid_upper_A_front_back.Compute();
            pid_lower_A_front_back.Compute();

            output_upper_B_front_back=0;
            output_lower_B_front_back=0;
            
            setpoint_upper_A_side_side=output_IMU;
            setpoint_lower_A_side_side=output_IMU;
            input_upper_A_side_side=analogRead(pot_A_upper_side_side)-pot_A_upper_side_side_offset;
            input_lower_A_side_side=analogRead(pot_A_lower_side_side)-pot_A_lower_side_side_offset;
            pid_upper_A_side_side.Compute();
            pid_lower_A_side_side.Compute();

            setpoint_upper_B_side_side=output_IMU;
            setpoint_lower_B_side_side=output_IMU;
            input_upper_B_side_side=analogRead(pot_B_upper_side_side)-pot_B_upper_side_side_offset;
            input_lower_B_side_side=analogRead(pot_B_lower_side_side)-pot_B_lower_side_side_offset;
            pid_upper_B_side_side.Compute();
            pid_lower_B_side_side.Compute();

            motorAUpper1Speed=output_upper_A_front_back - output_upper_A_side_side;
            motorALower1Speed=output_lower_A_front_back - output_lower_A_side_side;
            motorBUpper1Speed=output_upper_B_front_back - output_upper_B_side_side;
            motorBLower1Speed=output_lower_B_front_back - output_lower_B_side_side;
            motorAUpper2Speed=output_upper_A_front_back + output_upper_A_side_side;
            motorALower2Speed=output_lower_A_front_back + output_lower_A_side_side;
            motorBUpper2Speed=output_upper_B_front_back + output_upper_B_side_side;
            motorBLower2Speed=output_lower_B_front_back + output_lower_B_side_side;

            motorSpeed();

           }

            previousMillis=millis();
            currentMillis=previousMillis;

            while(currentMillis-previousMillis>100){
            currentMillis+=20;

            pid_Imu(0);
            
            input_upper_B_front_back=analogRead(pot_B_upper_front_back_pin) - pot_B_upper_front_back_offset;
            input_lower_B_front_back=analogRead(pot_B_lower_front_back_pin) - pot_B_lower_front_back_offset;
            pid_upper_B_front_back.Compute();
            pid_lower_B_front_back.Compute();

            output_upper_A_front_back=0;
            output_lower_A_front_back=0;
            
            setpoint_upper_B_side_side=output_IMU;
            setpoint_lower_B_side_side=output_IMU;
            input_upper_B_side_side=analogRead(pot_B_upper_side_side)-pot_B_upper_side_side_offset;
            input_lower_B_side_side=analogRead(pot_B_lower_side_side)-pot_B_lower_side_side_offset;
            pid_upper_B_side_side.Compute();
            pid_lower_B_side_side.Compute();

            setpoint_upper_A_side_side=output_IMU;
            setpoint_lower_A_side_side=output_IMU;
            input_upper_A_side_side=analogRead(pot_A_upper_side_side)-pot_A_upper_side_side_offset;
            input_lower_A_side_side=analogRead(pot_A_lower_side_side)-pot_A_lower_side_side_offset;
            pid_upper_A_side_side.Compute();
            pid_lower_A_side_side.Compute();

            motorAUpper1Speed=output_upper_A_front_back - output_upper_A_side_side;
            motorALower1Speed=output_lower_A_front_back - output_lower_A_side_side;
            motorBUpper1Speed=output_upper_B_front_back - output_upper_B_side_side;
            motorBLower1Speed=output_lower_B_front_back - output_lower_B_side_side;
            motorAUpper2Speed=output_upper_A_front_back + output_upper_A_side_side;
            motorALower2Speed=output_lower_A_front_back + output_lower_A_side_side;
            motorBUpper2Speed=output_upper_B_front_back + output_upper_B_side_side;
            motorBLower2Speed=output_lower_B_front_back + output_lower_B_side_side;

            motorSpeed();
            }

      }

        break;

        case 'r':

        setpoint_IMU=bluetooth.parseInt();
        Serial.println(right);
        Serial.println(setpoint_IMU);
        Serial.println(" ");

        previousMillis=millis();
        currentMillis=previousMillis;

        while(currentMillis-previousMillis>100){
        currentMillis+=20;
       
        pid_Imu(setpoint_IMU);
        
            setpoint_upper_A_side_side=output_IMU;
            setpoint_lower_A_side_side=output_IMU;
            input_upper_A_side_side=analogRead(pot_A_upper_side_side)-pot_A_upper_side_side_offset;
            input_lower_A_side_side=analogRead(pot_A_lower_side_side)-pot_A_lower_side_side_offset;
            pid_upper_A_side_side.Compute();
            pid_lower_A_side_side.Compute();

            setpoint_upper_B_side_side=output_IMU;
            setpoint_lower_B_side_side=output_IMU;
            input_upper_B_side_side=analogRead(pot_B_upper_side_side)-pot_B_upper_side_side_offset;
            input_lower_B_side_side=analogRead(pot_B_lower_side_side)-pot_B_lower_side_side_offset;
            pid_upper_B_side_side.Compute();
            pid_lower_B_side_side.Compute();

            output_upper_A_front_back=0;
            output_lower_A_front_back=0;
            output_upper_B_front_back=0;
            output_lower_B_front_back=0;

            motorAUpper1Speed=output_upper_A_front_back - output_upper_A_side_side;
            motorALower1Speed=output_lower_A_front_back - output_lower_A_side_side;
            motorBUpper1Speed=output_upper_B_front_back - output_upper_B_side_side;
            motorBLower1Speed=output_lower_B_front_back - output_lower_B_side_side;
            motorAUpper2Speed=output_upper_A_front_back + output_upper_A_side_side;
            motorALower2Speed=output_lower_A_front_back + output_lower_A_side_side;
            motorBUpper2Speed=output_upper_B_front_back + output_upper_B_side_side;
            motorBLower2Speed=output_lower_B_front_back + output_lower_B_side_side;

            motorSpeed();

        
        }
          
        break;

        case 'l':

        setpoint_IMU=bluetooth.parseInt();

          Serial.println(left);
          Serial.println(setpoint_IMU);
          Serial.println(" ");

        previousMillis=millis();
        currentMillis=previousMillis;

        while(currentMillis-previousMillis>100){
        currentMillis+=20;
        
        pid_Imu(-(setpoint_IMU));
        
            setpoint_upper_A_side_side=output_IMU;
            setpoint_lower_A_side_side=output_IMU;
            input_upper_A_side_side=analogRead(pot_A_upper_side_side)-pot_A_upper_side_side_offset;
            input_lower_A_side_side=analogRead(pot_A_lower_side_side)-pot_A_lower_side_side_offset;
            pid_upper_A_side_side.Compute();
            pid_lower_A_side_side.Compute();

            setpoint_upper_B_side_side=output_IMU;
            setpoint_lower_B_side_side=output_IMU;
            input_upper_B_side_side=analogRead(pot_B_upper_side_side)-pot_B_upper_side_side_offset;
            input_lower_B_side_side=analogRead(pot_B_lower_side_side)-pot_B_lower_side_side_offset;
            pid_upper_B_side_side.Compute();
            pid_lower_B_side_side.Compute();

            output_upper_A_front_back=0;
            output_lower_A_front_back=0;
            output_upper_B_front_back=0;
            output_lower_B_front_back=0;

            motorAUpper1Speed=output_upper_A_front_back - output_upper_A_side_side;
            motorALower1Speed=output_lower_A_front_back - output_lower_A_side_side;
            motorBUpper1Speed=output_upper_B_front_back - output_upper_B_side_side;
            motorBLower1Speed=output_lower_B_front_back - output_lower_B_side_side;
            motorAUpper2Speed=output_upper_A_front_back + output_upper_A_side_side;
            motorALower2Speed=output_lower_A_front_back + output_lower_A_side_side;
            motorBUpper2Speed=output_upper_B_front_back + output_upper_B_side_side;
            motorBLower2Speed=output_lower_B_front_back + output_lower_B_side_side;

            motorSpeed();
            
        break;
          
        }
     
      }
   }
   delay(10);
}

void pid_Imu(int set){

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

        setpoint_IMU=set;
        input_IMU=roll;
        pid_IMU.Compute();

    }
}
void motorSpeed(){
      
      if(motorAUpper1Speed<0){
              digitalWrite(motorAUpper1_cloc,LOW);
              digitalWrite(motorAUpper1_anticloc,HIGH);

              motorAUpper1Speed=abs(motorAUpper1Speed);
              analogWrite(motorAUpper1_pwm,motorAUpper1Speed);
            }
            else if(motorAUpper1Speed>0){
              digitalWrite(motorAUpper1_cloc,HIGH);
              digitalWrite(motorAUpper1_anticloc,LOW);
              
              analogWrite(motorAUpper1_pwm,motorAUpper1Speed);
              }
             else{
              digitalWrite(motorAUpper1_cloc,HIGH);
              digitalWrite(motorAUpper1_anticloc,HIGH);
              }

          if(motorAUpper2Speed<0){
              digitalWrite(motorAUpper2_cloc,LOW);
              digitalWrite(motorAUpper2_anticloc,HIGH);

              motorAUpper2Speed=abs(motorAUpper2Speed);
              analogWrite(motorAUpper2_pwm,motorAUpper2Speed);
            }
            else if(motorAUpper2Speed>0){
              digitalWrite(motorAUpper2_cloc,HIGH);
              digitalWrite(motorAUpper2_anticloc,LOW);
              
              analogWrite(motorAUpper2_pwm,motorAUpper2Speed);
              }
             else{
              digitalWrite(motorAUpper2_cloc,HIGH);
              digitalWrite(motorAUpper2_anticloc,HIGH);
              }

          if(motorALower1Speed<0){
              digitalWrite(motorALower1_cloc,LOW);
              digitalWrite(motorALower1_anticloc,HIGH);

              motorALower1Speed=abs(motorALower1Speed);
              analogWrite(motorALower1_pwm,motorALower1Speed);
            }
            else if(motorALower1Speed>0){
              digitalWrite(motorALower1_cloc,HIGH);
              digitalWrite(motorALower1_anticloc,LOW);
              
              analogWrite(motorALower1_pwm,motorALower1Speed);
              }
             else{
              digitalWrite(motorALower1_cloc,HIGH);
              digitalWrite(motorALower1_anticloc,HIGH);
              }

          if(motorALower2Speed<0){
              digitalWrite(motorALower2_cloc,LOW);
              digitalWrite(motorALower2_anticloc,HIGH);

              motorALower2Speed=abs(motorALower2Speed);
              analogWrite(motorALower2_pwm,motorALower2Speed);
            }
            else if(motorALower2Speed>0){
              digitalWrite(motorALower2_cloc,HIGH);
              digitalWrite(motorALower2_anticloc,LOW);
              
              analogWrite(motorALower2_pwm,motorALower2Speed);
              }
             else{
              digitalWrite(motorALower2_cloc,HIGH);
              digitalWrite(motorALower2_anticloc,HIGH);
              }

           if(motorBUpper1Speed<0){
              digitalWrite(motorBUpper1_cloc,LOW);
              digitalWrite(motorBUpper1_anticloc,HIGH);

              motorBUpper1Speed=abs(motorBUpper1Speed);
              analogWrite(motorBUpper1_pwm,motorBUpper1Speed);
            }
            else if(motorBUpper1Speed>0){
              digitalWrite(motorBUpper1_cloc,HIGH);
              digitalWrite(motorBUpper1_anticloc,LOW);
              
              analogWrite(motorBUpper1_pwm,motorAUpper1Speed);
              }
             else{
              digitalWrite(motorBUpper1_cloc,HIGH);
              digitalWrite(motorBUpper1_anticloc,HIGH);
              }

          if(motorBUpper2Speed<0){
              digitalWrite(motorBUpper2_cloc,LOW);
              digitalWrite(motorBUpper2_anticloc,HIGH);

              motorBUpper2Speed=abs(motorBUpper2Speed);
              analogWrite(motorBUpper2_pwm,motorBUpper2Speed);
            }
            else if(motorBUpper2Speed>0){
              digitalWrite(motorBUpper2_cloc,HIGH);
              digitalWrite(motorBUpper2_anticloc,LOW);
              
              analogWrite(motorBUpper2_pwm,motorBUpper2Speed);
              }
             else{
              digitalWrite(motorBUpper2_cloc,HIGH);
              digitalWrite(motorBUpper2_anticloc,HIGH);
              }

          if(motorBLower1Speed<0){
              digitalWrite(motorBLower1_cloc,LOW);
              digitalWrite(motorBLower1_anticloc,HIGH);

              motorBLower1Speed=abs(motorBLower1Speed);
              analogWrite(motorBLower1_pwm,motorBLower1Speed);
            }
            else if(motorBLower1Speed>0){
              digitalWrite(motorBLower1_cloc,HIGH);
              digitalWrite(motorBLower1_anticloc,LOW);
              
              analogWrite(motorBLower1_pwm,motorBLower1Speed);
              }
             else{
              digitalWrite(motorBLower1_cloc,HIGH);
              digitalWrite(motorBLower1_anticloc,HIGH);
              }

          if(motorBLower2Speed<0){
              digitalWrite(motorBLower2_cloc,LOW);
              digitalWrite(motorBLower2_anticloc,HIGH);

              motorBLower2Speed=abs(motorBLower2Speed);
              analogWrite(motorBLower2_pwm,motorBLower2Speed);
            }
            else if(motorBLower2Speed>0){
              digitalWrite(motorBLower2_cloc,HIGH);
              digitalWrite(motorBLower2_anticloc,LOW);
              
              analogWrite(motorBLower2_pwm,motorBLower2Speed);
              }
             else{
              digitalWrite(motorBLower2_cloc,HIGH);
              digitalWrite(motorBLower2_anticloc,HIGH);
              }
     }
  

#include <basicMPU6050.h>


//Arduino layout with connections
//Radio       MISO MOSI SCLK 2pin   pins 12+11+13+7+8  5V+GND    CSN -> 7  CE -> 8
//GY-5212     I2C                   pins A4+A5                   SDA -> A4 SCL -> A5
//Motors      PWM                   pins D5+D6+D9+D10            M1(A2)CCW -> 5 M2(A4) REV -> 6 M3(A1) REV -> 9 M4(A3)CCW -> 10


#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

//Init Radio
RF24 radio(8,7);

const byte address[6] = "00001";
uint16_t Joysticks[] = {0, 0, 0, 0, 0, 0}; //(Thr, Yaw, Pitch, Roll, RotL, RotR}

//Init MPU
const int MPU_addr=0x68;  // I2C address of the MPU-6050
double MPU_data[] = {0, 0, 0, 0, 0, 0}; //Data from MPU {AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ};

basicMPU6050<> mpu;

int Thr;
float input[] = {0, 0, 0}; //Yaw, Pitch, Roll from Joystick
double contr[] = {0, 0, 0}; //Yaw, Pitch, Roll to control motors

double cumerror[] = {0, 0, 0};
double last_error[] = {0, 0, 0};
double last_time[] = {0, 0, 0};
int Kp[] = {10, 10, 10};
float Ki[] = {0.1, 0.1, 0.1};
int Kd[] = {1, 1, 1};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(38400);

  //start radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //start MPU
  //mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  mpu.setup();
  

  //start Motors
  Motor1.attach(5);
  Motor1.writeMicroseconds(700);
  Motor2.attach(6);
  Motor2.writeMicroseconds(700);
  Motor3.attach(9);
  Motor3.writeMicroseconds(700);
  Motor4.attach(10);
  Motor4.writeMicroseconds(700);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (radio.available()){
    radio.read(&Joysticks, sizeof(Joysticks));
    //Serial.println(Joysticks[1]);
  }
  UpdateMPU();
  Serial.print(MPU_data[0]);
  Serial.print(" ");
  Serial.print(MPU_data[1]);
  Serial.print(" ");
  Serial.println(MPU_data[2]);
  //Assign Throttle
  Thr = Joysticks[0];            // reads the value of the potentiometer (value between 0 and 1023)
  Thr = map(Thr, 0, 511, 1000, 2000); //map throttle to PWM signal

  //Assign Yaw Pitch Roll
  for (byte i = 0; i<= 2; i++){
    input[i] = Joysticks[i+1];
  }

  //Map desired rates to deg/s
  input[0] = map(input[0], 0, 511, -90, 90); //map from 0-511 range to -90 to 90 deg/s desired yaw rate
  input[1] = map(input[1], 0, 511, -180, 180); //map from 0-511 range to -180 to 180 deg/s desired pitch rate
  input[2] = map(input[2], 0, 511, -180, 180); //map from 0-511 range to -180 to 180 deg/s desired roll rate


  for (byte i = 0; i<= 2; i++){
    contr[i] = PIDcontrol(i);
  }


  //Write values to motor
  Motor1.writeMicroseconds(Thr - contr[2]/2 + contr[1]/2 - contr[0]/2); //set Motor 3 to 10/180
  Motor2.writeMicroseconds(Thr - contr[2]/2 - contr[1]/2 + contr[0]/2);
  Motor3.writeMicroseconds(Thr + contr[2]/2 + contr[1]/2 + contr[0]/2);
  Motor4.writeMicroseconds(Thr + contr[2]/2 - contr[1]/2 - contr[0]/2);



  
  delay(100);
}


void UpdateMPU(){
  MPU_data[0] = mpu.ax();
  MPU_data[1] = mpu.ay();
  MPU_data[2] = mpu.az();
  MPU_data[3] = mpu.temp()/340.00+36.53;  //equation for temperature in degrees C from datasheet
  MPU_data[4] = mpu.gx();
  MPU_data[5] = mpu.gy();
  MPU_data[6] = mpu.gz();
}


double PIDcontrol(int i){
  double currentTime = millis();
  double elapsedTime = currentTime - last_time[i];
  
  double error = MPU_data[i+4] - input[i];
  double derror = (error - last_error[i])/elapsedTime;
  cumerror[i] += error*elapsedTime;
  
  double P = Kp[i]*error;
  double I = Ki[i]*cumerror[i];
  double D = Kd[i]*derror;

  double compensation = P+I+D;
  
  last_time[i] = currentTime;
  last_error[i] = error;  
  return compensation;
}

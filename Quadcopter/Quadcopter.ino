


//Arduino layout with connections
//Radio       MISO MOSI SCLK 2pin   pins 12+11+13+7+8  5V+GND    CSN -> 7  CE -> 8
//GY-5212     I2C                   pins A4+A5                   SDA -> A4 SCL -> A5
//Motors      PWM                   pins D5+D6+D9+D10            M1(A2)CCW -> 5 M2(A4) REV -> 6 M3(A1) REV -> 9 M4(A3)CCW -> 10


#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <Servo.h>
#include <basicMPU6050.h>

Servo Motor1;
Servo Motor2;
Servo Motor3;
Servo Motor4;

//Init Radio
RF24 radio(8,7);

const byte address[6] = "00001";
uint16_t Joysticks[] = {0, 255, 255, 255, 0, 0}; //(Thr, Yaw, Pitch, Roll, RotL, RotR}

//Init MPU
float MPU_data[] = {0, 0, 0, 0, 0, 0, 0}; //Data from MPU {AcX,AcY,AcZ,Tmp,GyZ,GyY,GyX};

basicMPU6050<> mpu;

int Thr;
float input[] = {90, 180, 180}; //Yaw, Pitch, Roll from Joystick INITIALISE AS 255 ALL
double contr[] = {0, 0, 0}; //Yaw, Pitch, Roll to control motors

double cumerror[] = {0, 0, 0};
double last_error[] = {0, 0, 0};
double last_time[] = {0, 0, 0};
float Kp[] = {1, 1, 1};
float Ki[] = {0, 0, 0};
float Kd[] = {0.1, 0.1, 0.1};


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

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
  mpu.setBias();
  

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

    //Assign Throttle
    Thr = Joysticks[0];            // reads the value of the potentiometer (value between 0 and 1023)
    Thr = map(Thr, 0, 511, 1000, 2000); //map throttle to PWM signal

      //Assign Yaw Pitch Roll
    for (byte i = 0; i<= 2; i++){
      input[i] = Joysticks[i+1];
    }
    
    //Map desired rates to deg/s
    input[0] = map(input[0], 0, 511, -90, 90); //map from 0-511 range to -90 to 90 deg/s desired yaw rate
    input[1] = map(input[1], 0, 511, -180, 180); //map from 0-511 range to -45 to 45 deg/s desired pitch rate
    input[2] = map(input[2], 0, 511, -180, 180); //map from 0-511 range to -45 to 45 deg/s desired roll rate
  }

  UpdateMPU();

  for (byte i = 0; i<= 2; i++){
    contr[i] = PIDcontrol(i);
  }

  Serial.print(contr[0]);
  Serial.print("c ");
  Serial.print(contr[1]);
  Serial.print("c ");
  Serial.println(contr[2]);
//  Serial.print(MPU_data[3]);
//  Serial.print(" ");
//  Serial.print(MPU_data[4]);
//  Serial.print(" ");
//  Serial.print(MPU_data[5]);
//    Serial.print(" ");
Serial.print(input[0]);
Serial.print("e ");
Serial.print(input[1]);
Serial.print("e ");
Serial.print(input[2]);
Serial.print("e ");
  Serial.print(MPU_data[0]);
    Serial.print("d ");
  Serial.print(MPU_data[1]);
    Serial.print("d ");
  Serial.println(MPU_data[2]);
//  Serial.print(" ");
//  Serial.println(MPU_data[6]);

  //Write values to motor
  Motor1.writeMicroseconds(Thr - contr[2]/2 + contr[1]/2 - contr[0]/2);
  Motor2.writeMicroseconds(Thr - contr[2]/2 - contr[1]/2 + contr[0]/2);
  Motor3.writeMicroseconds(Thr + contr[2]/2 + contr[1]/2 + contr[0]/2);
  Motor4.writeMicroseconds(Thr + contr[2]/2 - contr[1]/2 - contr[0]/2);



}


void UpdateMPU(){
  MPU_data[0] = mpu.gz()*RAD_TO_DEG; //Yaw rate: Left +ve, Right -ve
  MPU_data[1] = mpu.gy()*RAD_TO_DEG; //Pitch rate: Up +ve, Down -ve
  MPU_data[2] = mpu.gx()*RAD_TO_DEG; //Roll rate:  Left +ve, Right -ve
  MPU_data[3] = mpu.ax()*9.81; //Nose Down +ve
  MPU_data[4] = mpu.ay()*9.81; //Bank Left +ve
  MPU_data[5] = mpu.az()*9.81; //Down towards Earth +ve
  MPU_data[6] = mpu.temp();  //equation for temperature in degrees C from datasheet

//    // Accel
//  Serial.print( mpu.ax() );
//  Serial.print( " " );
//  Serial.print( mpu.ay() );
//  Serial.print( " " );
//  Serial.print( mpu.az() );
//  Serial.print( "    " );
//  
//  // Gyro
//  Serial.print( mpu.gx() );
//  Serial.print( " " );
//  Serial.print( mpu.gy() );
//  Serial.print( " " );
//  Serial.print( mpu.gz() );
//  Serial.print( "    " );  
//  
//  // Temp
//  Serial.print( mpu.temp() );
//  Serial.println(); 

}


double PIDcontrol(int i){
  double currentTime = millis();
  double elapsedTime = currentTime - last_time[i];
  
  double error = MPU_data[i] - input[i];
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

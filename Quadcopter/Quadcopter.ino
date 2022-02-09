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
uint16_t Joysticks[] = {0, 0, 0, 0, 0, 0};

//Init MPU
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

basicMPU6050<> mpu;

uint8_t value;

//Init Motors
int potpin = A3;  // analog pin used to connect the potentiometer
int val;

void setup() {
  // put your setup code here, to run once:
  //Serial.begin(38400);

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
    Serial.println(Joysticks[1]);
  }

//  Wire.beginTransmission(0x68);
//  Wire.write(0x75); //Check if 77 address works. if output, then probably 20689, if not. Then WTF is going on
//  Wire.endTransmission(false);
//  int n = Wire.requestFrom(0x68, 1);
//  if( n == 1)
//  {
//    uint8_t data = Wire.read();
//
//    Serial.print("0x");
//    Serial.println( data, BIN);
//  }
//  else
//  {
//    Serial.println("Sensor not found");
//  }
  UpdateMPU();
  //Serial.print(" ");
  //Serial.println( mpu.rawAx() );

  val = analogRead(potpin);            // reads the value of the potentiometer (value between 0 and 1023)
  val = map(val, 0, 1023, 1000, 2000);
  Motor1.writeMicroseconds(val); //set Motor 3 to 10/180
  Motor2.writeMicroseconds(val);
  Motor3.writeMicroseconds(val);
  Motor4.writeMicroseconds(val);
  //Serial.println(val);
  delay(15);
}


void UpdateMPU(){
  //Wire.beginTransmission(MPU_addr);
  //Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  //Wire.endTransmission(false);
  //Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  //AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  //AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  //AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  //GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  //GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  //GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(mpu.ax());
  Serial.print(" | AcY = "); Serial.print(mpu.ay());
  Serial.print(" | AcZ = "); Serial.print(mpu.az());
  Serial.print(" | Tmp = "); Serial.print(mpu.temp()/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(mpu.gx());
  Serial.print(" | GyY = "); Serial.print(mpu.gy());
  Serial.print(" | GyZ = "); Serial.println(mpu.gz());
}

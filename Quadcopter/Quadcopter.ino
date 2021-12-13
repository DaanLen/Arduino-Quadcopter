//Arduino layout with connections
//Radio       MISO MOSI SCLK 2pin   pins 12+11+13+7+8  5V+GND    CSN -> 7  CE -> 8
//GY-5212     I2C                   pins A4+A5                   SDA -> A4 SCL -> A5

#include <SPI.h>
#include <RF24.h>
#include <Wire.h>
#include <MPU6050.h>

//Init Radio
RF24 radio(8,7);

const byte address[6] = "00001";
uint16_t Joysticks[] = {0, 0, 0, 0, 0, 0};

//Init MPU
const int MPU_addr=0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;

MPU6050 mpu;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //start radio
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //start MPU
  mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G);
  // If you want, you can set accelerometer offsets
  // mpu.setAccelOffsetX();
  // mpu.setAccelOffsetY();
  // mpu.setAccelOffsetZ();
  
}

void loop() {
  // put your main code here, to run repeatedly:
  if (radio.available()){
    radio.read(&Joysticks, sizeof(Joysticks));
    Serial.println(Joysticks[1]);
  }
  Vector normAccel = mpu.readRawAccel();

  Serial.print(" Xnorm = ");
  Serial.print(normAccel.XAxis*.000061f);
  Serial.print(" Ynorm = ");
  Serial.print(normAccel.YAxis*.000061f);
  Serial.print(" Znorm = ");
  Serial.println(normAccel.ZAxis*.000061f);
  
  delay(200);
}


void UpdateMPU(){
    Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
}

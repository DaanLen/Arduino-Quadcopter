//Arduino layout with connections
//Radio       MISO MOSI SCLK 2pin   pins 12+11+13+7+8  5V+GND    CSN -> 7  CE -> 8

#include <SPI.h>
#include <RF24.h>


RF24 radio(8,7); //CE, CSN WAAROM????

const byte address[6] = "00001";
int Joysticks[] = {0, 0, 0, 0};

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(0, address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();

  //pinMode(2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  if (radio.available()){
    radio.read(&Joysticks, sizeof(Joysticks));
    Serial.println(Joysticks[1]);
  }
}

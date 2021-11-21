//Arduino layout with connections
//Radio       MISO MOSI SCLK 2pin   pins x+x+x+x+x  5V+GND    CSN -> x  CE -> x


#include <SPI.h>
#include <RF24.h>

RF24 radio(7,10); //CE, CSN
const byte address = "00001";
//Define pins


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  //Setup radio
  radio.begin();
  radio.openReadingPipe(0,address);
  radio.setPALevel(RF24_PA_MIN);
  radio.startListening();
}

void loop() {
  // put your main code here, to run repeatedly:

  //Read Radio
  if (radio.available()) {
    char text[32] = "";
    radio.read(&text, sizeof(text));
    Serial.println(text);
  }

}

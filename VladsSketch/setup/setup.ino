/*
Bluetooth module -> Arduino
Grnd -> Grnd
VCC -> 5v
Rx -> pin3
Tx -> pin4
*/

#include <SoftwareSerial.h>

SoftwareSerial bt(2,4);//tx rx

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);

  bt.begin(9600); 
  delay(1000);
  
  //bt.print("AT");
  //delay(1000);

  Serial.println("Done\n");


}

void loop() {

  if (bt.available())
    Serial.write(bt.read());
  if (Serial.available())
    bt.write(Serial.read()); 
    

}




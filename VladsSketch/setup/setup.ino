/*
 * 1 set to 1200bps
    2 set to 2400bps 
    3 set to 4800bps 
    4 set to 9600bps (Default) 
    5 set to 19200bps 
    6 set to 38400bps 
    7 set to 57600bps 
    8 set to 115200bps

 */
const byte rxPin = 5;  //bluetooth tx pin goes here
const byte txPin = 6;  //bluetooth rx pin goes here

#include <SoftwareSerial.h>

SoftwareSerial bt(rxPin, txPin); //tx rx


void setup() {
  // put your setup code here, to run once:

  //Serial.begin(115200);

  bt.begin(9600);//////////////////////////////Starting
  delay(1000);
  bt.print("AT");
  delay(1000);
  bt.print("AT+BAUD1");//1200bps
  //bt.print("AT+BAUD6");//38400bps//////////////ending
  delay(1000);
  //delay(500);

  bt.begin(1200);//////////////////////////////ending
 // Serial.println("Done\n");

}

void loop() {
/*
  // if (bt.available())
  //   Serial.print((char)bt.read());
  while (Serial.available())
    bt.print((char)Serial.read());
  
  while (bt.available())
      Serial.print((char)bt.read());
  */ 
  bt.read(); 

}



/*
Bluetooth module -> Arduino
Grnd -> Grnd
VCC -> 5v
Rx -> pin3
Tx -> pin4






AT : check the connection
AT+NAME: Change name. No space between name and command.
AT+PIN: change pin, xxxx is the pin, again, no space.
AT+VERSION

AT+BAUDX: change baud rate, x is baud rate code, no space between command and code.
1 set to 1200bps
2 set to 2400bps 
3 set to 4800bps 
4 set to 9600bps (Default) 
5 set to 19200bps 
6 set to 38400bps 
7 set to 57600bps 
8 set to 115200bps


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




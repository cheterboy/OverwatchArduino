/*
 * This prints everything to the screen 
 */



/*
Bluetooth module -> Arduino
Grnd -> Grnd
VCC -> 5v
Rx -> pin3
Tx -> pin2



0


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



String inputString;
bool stringComplete = false; 
const byte rxPin = 2;  //bluetooth tx pin goes here
const byte txPin = 3;  //bluetooth rx pin goes here


#include <SoftwareSerial.h>

SoftwareSerial bt(rxPin,txPin);//tx r


void setup() {
  // put your setup code here, to run once:

  Serial.begin(9600);
/*
  bt.begin(1200); 
 // delay(1000);
  bt.print("AT");
  delay(1000);
  //bt.print("AT+BAUD1");//1200bps
  bt.print("AT+BAUD7");//57600bps
  //delay(1000);
  delay(500); 
  Serial.println("Done\n");
  bt.end();
  delay(500); 
  */
  bt.begin(9600); 
 /* delay(1000);
  bt.print("AT");
  delay(1000);
  bt.print("AT+UAD");
  delay(1000);*/
}


void loop() {
  while(bt.available())  // If the bluetooth sent any characters
  {
    // Send any characters the bluetooth prints to the serial monitor
    Serial.print((char)bt.read());  
  }
  while(Serial.available())  // If stuff was typed in the serial monitor
  {
    // Send any characters the Serial monitor prints to the bluetooth
    bt.print((char)Serial.read());
    delay(100); 
  }
}


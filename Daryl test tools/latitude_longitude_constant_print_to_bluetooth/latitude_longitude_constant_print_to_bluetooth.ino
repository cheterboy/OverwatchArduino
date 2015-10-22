/*
  Software serial multple serial test

 Receives from the hardware serial, sends to software serial.
 Receives from software serial, sends to hardware serial.

 The circuit:
 * RX is digital pin 10 (connect to TX of other device)
 * TX is digital pin 11 (connect to RX of other device)

 Note:
 Not all pins on the Mega and Mega 2560 support change interrupts,
 so only the following can be used for RX:
 10, 11, 12, 13, 50, 51, 52, 53, 62, 63, 64, 65, 66, 67, 68, 69

 Not all pins on the Leonardo support change interrupts,
 so only the following can be used for RX:
 8, 9, 10, 11, 14 (MISO), 15 (SCK), 16 (MOSI).

 created back in the mists of time
 modified 25 May 2012
 by Tom Igoe
 based on Mikal Hart's example

 This example code is in the public domain.

 */
#include <SoftwareSerial.h>

SoftwareSerial mySerial(6, 7); // RX, TX

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }


 
  // set the data rate for the SoftwareSerial port
  mySerial.begin(9600);
 // mySerial.println("Hello, world?");
}
float latitude = 0.000000; 
float longitude = 0.000000; 
void loop() // run over and over
{
   

    
   // mySerial.print(counter);
   // mySerial.print("\n"); 

   
    Serial.print(latitude,6); 
    Serial.print(","); 
    Serial.println(longitude,6); 
    mySerial.print(latitude,6); 
    mySerial.print(","); 
    mySerial.print(longitude,6);     
    mySerial.print("\n"); 


   latitude += 1.111111; 
   longitude -= 1.111111; 
   
   if (latitude > 40 )
    {  latitude = 0;
      longitude = 0;  
    }
    
   delay(500); 
   
}


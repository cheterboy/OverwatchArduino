// nrf24_reliable_datagram_client.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging client
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_server
// Tested on Uno with Sparkfun WRL-00691 NRF24L01 module
// Tested on Teensy with Sparkfun WRL-00691 NRF24L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module

#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <String.h> 



#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2


// Singleton instance of the radio driver
RH_NRF24 driver;

//lowers the transmission rate to 250kBPS
//RH_NRF24 driver(2, 3);

//RH_NRF24 driver(8, 10);   // For RFM73 on Anarduino Mini






SoftwareSerial bt(6,7);//tx rx
String inputString;         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, CLIENT_ADDRESS);






#define buttonPin 3

void setup() 
{
  //driver.setRF(driver.DataRate250kbps, driver.TransmitPower0dBm);


  
  Serial.print("Here");
  pinMode(buttonPin, INPUT); 
  
  Serial.begin(250000);
  if (!manager.init())
    Serial.println("init failed");

manager.setRetries(1);


 //driver.setRF(driver.DataRate250kbps, driver.TransmitPower0dBm); 
 //driver.setRF(driver.DataRate1Mbps, driver.TransmitPower0dBm);
  
   



   bt.begin(9600);
  /* delay(1000); 
   bt.print("AT"); 
   delay(1000); 
   bt.print("AT+NAMEHC-06");
   delay(1000); 
   bt.print("AT+PIN1234");
   delay(1000); */
   
    

  
   inputString.reserve(200);
  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
}

uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];
// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
int counter = 0; 
void loop()
{

  //Serial.println(analogRead(A0)); 
  
  //Serial.println("Sending to nrf24_reliable_datagram_server");
  if(digitalRead(buttonPin) == LOW) 
    {
        
        inputString = "kill\n"; 
        stringComplete = true; 
        //Serial.print(inputString); 
      
    }


 if((counter < 2000) && (stringComplete == false))
  {
       inputString = String(counter);
       inputString += "\n";  
       counter++;
       if (counter > 1000)
          counter = 0;
       stringComplete = true; 
       Serial.print(inputString);
  }

    
  if(stringComplete == false)
  if(Serial.available())
  if(inputString = Serial.readString()) 
  {
    stringComplete = true;
    Serial.println(inputString); 
  }

  ///////////////////////////////////////enable this when using blueooth
  //serialEvent();


  if(stringComplete){
        Serial.println(inputString); 
 
         
        inputString.getBytes(data,RH_NRF24_MAX_MESSAGE_LEN);
        inputString.remove(0, RH_NRF24_MAX_MESSAGE_LEN); 
        
         if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
                  {
                         
                  // Now wait for a reply from the server
                  uint8_t len = sizeof(inputString);
                  uint8_t from;   
                  
                  
                  if (manager.recvfromAckTimeout(buf, &len, 200, &from))
                  {
                    Serial.print("got reply from : 0x");
                    Serial.print(from, HEX);
                    Serial.print(": ");
                    Serial.print((char*)data);
          
          

                    //this might have to get moved
                    Serial.print("        ");
                    Serial.println(inputString.length()); 
                    if((inputString.length() == 0))
					{
                      inputString ="";
                      stringComplete = false;
					}

                  }else{
                    Serial.println("No reply, is nrf24_reliable_datagram_server running?");
                  }
                }
                else
                  Serial.println("sendtoWait failed Resending");
          
           
                    
                  }


                  //delay(200);
  }
  




void serialEvent() {
  

  //if(bt.isListening())
  while (bt.available()) {
    // get the new byte:
    char inChar = (char)bt.read();
    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}






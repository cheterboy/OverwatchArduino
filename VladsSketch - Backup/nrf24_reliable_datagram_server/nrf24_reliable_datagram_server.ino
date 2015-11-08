// nrf24_reliable_datagram_server.pde
// -*- mode: C++ -*-
// Example sketch showing how to create a simple addressed, reliable messaging server
// with the RHReliableDatagram class, using the RH_NRF24 driver to control a NRF24 radio.
// It is designed to work with the other example nrf24_reliable_datagram_client
// Tested on Uno with Sparkfun WRL-00691 NRF24L01 module
// Tested on Teensy with Sparkfun WRL-00691 NRF24L01 module
// Tested on Anarduino Mini (http://www.anarduino.com/mini/) with RFM73 module
// Tested on Arduino Mega with Sparkfun WRL-00691 NRF25L01 module
/*
#include <RH_TCP.h>
#include <RH_Serial.h>
#include <RH_RF95.h>
#include <RH_RF69.h>
#include <RH_RF24.h>
#include <RH_RF22.h>
#include <RH_NRF905.h>
#include <RH_NRF51.h>
#include <RH_ASK.h>
#include <RHTcpProtocol.h>
#include <RHSPIDriver.h>
#include <RHSoftwareSPI.h>
#include <RHRouter.h>
#include <RHNRFSPIDriver.h>
#include <RHMesh.h>
#include <RHHardwareSPI.h>
#include <RHGenericSPI.h>
#include <RHGenericDriver.h>
#include <RHDatagram.h>
#include <RHCRC.h>
#include <radio_config_Si4460.h>
*/

#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_NRF24 driver;



//lowers the transmission rate to 250kBPS keeps the power the same 
//RH_NRF24 driver(2, 3);




/*
DataRate1Mbps = 0,   ///< 1 Mbps
DataRate2Mbps = 1,       ///< 2 Mbps
DataRate250kbps = 2      ///< 250 kbps

*/

//This is used to lower the power consumption at the cost of range We will probably not use this 
/*
TransmitPowerm18dBm = 0,        ///< On nRF24, -18 dBm
TransmitPowerm12dBm = 1,            ///< On nRF24, -12 dBm
TransmitPowerm6dBm = 2,             ///< On nRF24, -6 dBm
TransmitPower0dBm = 3,              ///< On nRF24, 0 dBm

*/




// RH_NRF24 driver(8, 7);   // For RFM73 on Anarduino Mini

// Class to manage message delivery and receipt, using the driver declared above
RHReliableDatagram manager(driver, SERVER_ADDRESS);



void setup() 
{

 // driver.setRF(driver.DataRate250kbps, driver.TransmitPower18dBm);
  
	 Serial.begin(250000);
	

  if (!manager.init())
    Serial.println("init failed");

  
  
  
// driver.setRF(driver.DataRate1Mbps, driver.TransmitPower0dBm);
 
  // Defaults after init are 2.402 GH (channel 2), 2Mbps, 0dBm
}

uint8_t data[1] = { 1 };
// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
String inputString; 
bool stringComplete; 



void loop()
{
   if(stringComplete) {
    Serial.println(inputString);
    stringComplete = false; 
    inputString = ""; 
    
   }
  
  if (manager.available())
  {
    // Wait for a message addressed to us from the client
    uint8_t len = sizeof(buf);
    uint8_t from;
    if (manager.recvfromAck(buf, &len, &from))
    {
      
      Serial.print("got request from : 0x");
      Serial.print(from, HEX);
      Serial.print(": ");
      Serial.println((char*)buf);

      inputString += (char*)buf; 
      if(inputString.endsWith("\n") )
      {
        stringComplete = true; 
                
      }


      // Send a reply back to the originator client
      if (!manager.sendtoWait(data, sizeof(data), from))
        Serial.println("sendtoWait failed");
    }
  }

  delay(500); 


}


#include <RHReliableDatagram.h>
#include <RH_NRF24.h>
#include <SPI.h>

#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2
RH_NRF24 driver;
RHReliableDatagram manager(driver, CLIENT_ADDRESS);


#include <SoftwareSerial.h>

SoftwareSerial bt(6, 7); //tx rx
String inputString;         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

#define buttonPin 3



void setup()
{
  Serial.begin(250000);
  while (!Serial)
    ; // wait for serial port to connect. Needed for Leonardo only

  // Defaults after init are 2.402 GHz (channel 2), 2Mbps, 0dBm
  pinMode(buttonPin, INPUT_PULLUP);
  bt.begin(9600);
  /*  delay(1000);
    bt.print("AT");
    delay(1000);
    bt.print("AT+BAUD2");
    delay(1000);*/


  if (!manager.init())
    Serial.println("Nrf manager init failed");


  Serial.print("done");
}

//outgoing
uint8_t data[RH_NRF24_MAX_MESSAGE_LEN];
// incoming
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];



void loop()
{

  if (digitalRead(buttonPin) == 0)
  {
    inputString = "~stoproute\n";
    stringComplete = true;
  }

  if (!stringComplete)
    serialEvent();

  while (stringComplete) {
    Serial.print("Incoming string:  "); 
    Serial.println(inputString);

    inputString.getBytes(data, RH_NRF24_MAX_MESSAGE_LEN);
    inputString.remove(0, RH_NRF24_MAX_MESSAGE_LEN);


    Serial.print("Bytes array:  ");
    for(int counter = 0; counter < 28; counter++)
    Serial.print((char)data[counter]); 
    
    if (manager.sendtoWait(data, sizeof(data), SERVER_ADDRESS))
    {
      Serial.println("here");
      uint8_t len = sizeof(buf);
      uint8_t from;
      if (manager.recvfromAckTimeout(buf, &len, 2000, &from))
      {
        Serial.print("got reply from : 0x");
        Serial.print(from, HEX);
        Serial.print(": ");
        Serial.println((char*)buf);
      }
      else
      {
        Serial.println("No reply, is nrf24_reliable_datagram_server running?");
      }
    }
    else
      Serial.println("sendtoWait failed");


    if (inputString.length() == 0)
    {
      stringComplete = false;
      inputString = "";
    }




  }

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


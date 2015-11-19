/*
Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
version 2 as published by the Free Software Foundation.
*/

/**
* Example for Getting Started with nRF24L01+ radios.
*
* This is an example of how to use the RF24 class.  Write this sketch to two
* different nodes.  Put one of the nodes into 'transmit' mode by connecting
* with the serial monitor and sending a 'T'.  The ping node sends the current
* time to the pong node, which responds by sending the value back.  The ping
* node can then see how long the whole cycle took.
*/

#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"


//
// Hardware configuration
//

// Set up nRF24L01 radio on SPI bus plus pins 9 & 10 

RF24 radio(8, 10);

//
// Topology
//

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[1] = { 0xF0F0F0F0D2LL};




void setup(void)
{
  //
  // Print preamble
  //

  Serial.begin(57600);


  //
  // Setup and configure rf radio
  //

  radio.begin();
  //radio.disableCRC(); 
  radio.setChannel(120);
  //radio.enableDynamicPayloads();
  // optionally, increase the delay between retries & # of retries
  //radio.setRetries(15, 15);

  // optionally, reduce the payload size.  seems to
  // improve reliability
  //radio.setPayloadSize(8);


  //if ( role == role_ping_out )
  {
    radio.stopListening();
  radio.openWritingPipe(pipes[0]);
    //radio.openReadingPipe(0, pipes[0]);
  }
  //else
  {
    //radio.openWritingPipe(pipes[1]);
    //radio.openReadingPipe(1,pipes[0]);
  }

  //
  // Start listening
  //

  

  //
  // Dump the configuration of the rf unit for debugging
  //

  radio.printDetails();
}

void loop(void)
{
  
  

    // First, stop listening so we can talk.
    //radio.stopListening();

    // Take the time, and send it.  This will block until complete
    
  Serial.println("Now sending ");
  

    unsigned char x[32]; 
    String test = "test"; 
    test.getBytes(x, sizeof(test)); 
    
    //unsigned long x = 10;
    Serial.print((char*)x);
    Serial.println(sizeof(x)); 
    radio.write(x, sizeof(x));

  
    

}
// vim:cin:ai:sts=2 sw=2 ft=cpp


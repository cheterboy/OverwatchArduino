
#include <SoftwareSerial.h>

SoftwareSerial bt(2, 3); // RX, TX

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 
  bt.begin(9600); 
}

void loop() {
  // put your main code here, to run repeatedly:
  if(Serial.available()) 
    Serial.print((char)Serial.read()); 
  if(bt.available()) 
    Serial.print((char)bt.read()); 
    
    
}

#include <SoftwareSerial.h>

void setup() {
  // put your setup code here, to run once:
 Serial.begin(115200);
}

float x = 0; 
double y = 0;


void loop() {
  // put your main code here, to run repeatedly:
float x = 0.123456;
int header = 100; 

double test = x + header; 


//double x = strtod(test.c_str(),NULL);

Serial.print(test,6); 
Serial.print("   "); 

Serial.println(x,6); 
delay(1000);

/*
  if (x < 1);
 {   
    x += 0.0000001; 
    y += 0.0000001; 
    Serial.print(x,6); 
    Serial.print(","); 
    Serial.println(y,6); 

 }*/

 
}

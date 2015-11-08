//GPS
#include <TinyGPS++.h>
#include <SoftwareSerial.h>


//#include <HM55B_Compass.h>
//compass
#include <HMC5883L.h>
#define COMPASSTOLERANCE 5


#include <Wire.h>

//NRF
/////////////////////////////////////////////////////////////
//nrf libraries 
#include <RH_NRF24.h>
#include <SPI.h>


//nrf 
#define CLIENT_ADDRESS 1
#define SERVER_ADDRESS 2

// Singleton instance of the radio driver
RH_NRF24 nrf24;




//          PPM
///////////////////////////////////////////////////
#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 3  //set PPM signal output pin on the arduino
#define FowardReverse 0 //set foward and reverse channel
#define LeftRight 1 //set left and right channel
/*
#define straight 1500
#define slightLeft 1250
#define slightRight 1750
#define hardLeft 1100
#define hardRight 1900
*/
#define MAXWAYPOINTS 10


//tylers car 
#define straight 1450


//#define straight 1500
#define slightLeft 1650
#define slightRight 1350
#define hardLeft 1900
#define hardRight 1100


#define neutral 1500
#define slowForward 1650 
#define fastForward 1650 
#define slowBackward 1350
#define fastBackward  1350

//PPM
int ppm[chanel_number];
int FRValue = 1500;
int LRValue = 1500;
///////////////////////////////////////////////////////





typedef struct waypoint {
  float latitude = 0.0000000; 
  float longitude = 0.0000000;
  float courseToWaypoint; 
  float distance; 
  //int estimatedArrivalTime;  
 };


//GPS
//////////////////////////////////////////
//static const int RXPin = 4, TXPin = 2;
//static const uint32_t GPSBaud = 9600;
//////////////////////////////////////

// The TinyGPS++ object
TinyGPSPlus gps;
SoftwareSerial ssGPS(5, 4);


//bluetooth
//SoftwareSerial bt(6,7);//tx rx





//HM55B_Compass compass(8, 9, 10);
HMC5883L compass;
int error =0; 

//BlueTooth
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
//String coordinateBuffer[11]; 
//int numberOfCoordinatesInBuffer = 0; 



void setup()
{

  Serial.begin(250000);
  Wire.begin();
    
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)

 compass = HMC5883L(); // Construct a new HMC5883 compass.

  //Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    //Serial.println(compass.GetErrorText(error));

  //Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    //Serial.println(compass.GetErrorText(error));


  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
 
  
   
  ssGPS.begin(9600);
  //while(!ssGPS); 
  //bt.begin(1200);
  //while(!bt); 


  //compass.initialize();//start compass
  //delay(100); 

  
  //getGPSData();
  //inputString.reserve(200);
  //Serial.println("Done"); 

  //smartDelay(3000);
  

 
  if (!nrf24.init())
	  Serial.println("init failed");
  else
    Serial.println("init OK");


  nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm);
  
  


}



//The response message
uint8_t data[] =  "1" ;

// Dont put this on the stack:
uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];


//String inputString;
//bool stringComplete;




//used to controll gps mode
bool followingGPS = false;
waypoint Waypoint[MAXWAYPOINTS];
//number of waypoints in buffer
int waypointCounter = 0;
//the waypoint that the uad is on
int routeCounter = 0;  






void loop()
{
  
  //Serial.println("here"); 



  
  if(followingGPS)
  {
      getGPSData();
      wichWay();
	  //nrf 
	  nrf(); 
  }  else
	  nrf();

   
  ppm[FowardReverse] = FRValue;
  ppm[LeftRight] = LRValue;

    
}


ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
  
  TCNT1 = 0;
  
  if(state) {  //start pulse
    digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2;
    state = false;
  }
  else{  //end pulse and calculate when to start the next pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    digitalWrite(sigPin, !onState);
    state = true;

    if(cur_chan_numb >= chanel_number){
      cur_chan_numb = 0;
      calc_rest = calc_rest + PPM_PulseLen;// 
      OCR1A = (PPM_FrLen - calc_rest) * 2;
      calc_rest = 0;
    }
    else{
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2;
      calc_rest = calc_rest + ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}



void nrf()
{
	uint8_t len = sizeof(buf);
	nrf24.recv(buf, &len);
    if(nrf24.available())
    {
    
      nrf24.recv(buf, &len);
	 

     
      Serial.print((char*)buf);
        //inputString = (char*)buf;
      
    }






}

/*
void bluetooth()
{
  serialEvent();

  Serial.println(stringComplete);


    if(stringComplete){
 Serial.println(inputString);  
if(inputString == "~forward\n"){
     FRValue = slowForward;
}else if(inputString == "~notforward\n"){
     FRValue = neutral;   
}else if(inputString == "~backward\n"){
     FRValue = slowBackward;
}else if(inputString == "~notbackward\n"){
     FRValue = neutral; 
}else if(inputString == "~left\n"){
     LRValue = hardLeft;
}else if(inputString == "~notleft\n"){
     LRValue = straight;
}else if(inputString == "~right\n"){
     LRValue = hardRight;
}else if(inputString == "~notright\n"){
     LRValue = straight;
}else if(inputString == "~start\n"){

     LRValue = straight; 
     FRValue = neutral;
     

}else if(inputString == "~startroute\n"){

    routeCounter = 0;
    followingGPS = true;

}else if(inputString == "~stoproute\n"){
     followingGPS = false;
     LRValue = straight; 
     FRValue = neutral;
     routeCounter = 0; 
     waypointCounter = 0; 
     //clearWaypointData();

     

}else if(inputString == "~uadlocation\n"){
    smartDelay(1000);
    float x = gps.location.lat();
    float y = gps.location.lng(); 
     bt.print(x,6); 
     bt.print(",");
     bt.print(y,6); 
     bt.print("\n"); 
//bt.print("Hello"); 

 
     
}else if(inputString == "~select\n"){

      
}else if(inputString == "~a\n"){

}else if(inputString == "~b\n"){
     while(FRValue > 1000) 
     {
        ppm[FowardReverse] = FRValue;
        ppm[LeftRight] = LRValue;
        FRValue--; 
        //Serial.println(FRValue); 
     }

}else if (isdigit(inputString[0])){





      String waypointNumber = inputString; 
      //delay(10);
      String latitude = inputString;
      //delay(10);
      String longitude = inputString;
      //delay(10);
      //find where the first comma is at
      int one = inputString.indexOf(',');
      //find where the second comma is at
      int two = inputString.lastIndexOf(',');

      //trim the string to just the number 
      waypointNumber.remove(one); 
      latitude.remove(0,one+1);
      latitude.remove(two-2);
      longitude.remove(0,two+1); 

     



     
      Waypoint[waypointCounter].latitude = (double)strtod(latitude.c_str(),NULL);
      delay(10);
      Waypoint[waypointCounter].longitude = (double)strtod(longitude.c_str(),NULL);

  
      waypointCounter++; 
  //    Serial.println(waypointCounter);
      if (waypointCounter > 10)
        waypointCounter =0; 
  
}else {
     followingGPS = false;
     LRValue = straight; 
     FRValue = neutral;
}


inputString ="";
stringComplete = false;
}

}*/
  


/*
void serialEvent() {
  
  bt.listen();
  
  //Serial.println(bt.isListening());

 
  //if(bt.isListening())
  
  while (bt.available()) {
    // get the new byte:
    char inChar = (char)bt.read();

  Serial.println(inChar);

    // add it to the inputString:
    inputString += inChar;
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
  bt.stopListening();
}*/


void getGPSData()
{

  
  smartDelay(200);

  Waypoint[routeCounter].distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      Waypoint[routeCounter].latitude, 
      Waypoint[routeCounter].longitude);
  //Serial.print(waypoint[routeCounter].distance);
  //printInt(distance, gps.location.isValid(), 9);

  Waypoint[routeCounter].courseToWaypoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      Waypoint[routeCounter].latitude, 
      Waypoint[routeCounter].longitude);

  //if (millis() > 5000 && gps.charsProcessed() < 10)
    //Serial.println(F("No GPS data received: check wiring"));

}


void clearWaypointData()
{
     int counter =0; 
     while (counter < MAXWAYPOINTS)
     {
        Waypoint[counter].latitude = 0; 
        Waypoint[counter].longitude =0; 
        Waypoint[counter].courseToWaypoint = 0; 
        Waypoint[counter].distance =0; 
        //Waypoint[counter].estimatedArrivalTime = 0; 
     }

  
}

  

void wichWay()
{

   MagnetometerRaw raw = compass.ReadRawAxis();
   MagnetometerScaled scaled = compass.ReadScaledAxis();
   int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
   float heading = atan2(scaled.YAxis, scaled.XAxis);

   float declinationAngle = 0.0346;
   heading += declinationAngle;
  
  
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  int angle = heading * 180/M_PI; 
  
  
  
  
  //int angle = compass.read();






 
  //angle += 180; 
  /*
  Serial.print("  Compas = "); 
  Serial.print(angle); 
  Serial.print("   Going to = "); 
  Serial.println(waypoint[routeCounter].courseToWaypoint); 
  Serial.print("      ");
*/
  
  float newCourse;
  if ((Waypoint[routeCounter].courseToWaypoint <= angle + COMPASSTOLERANCE) && 
  (Waypoint[routeCounter].courseToWaypoint >= angle - COMPASSTOLERANCE)){
  //  Serial.print("Go straight");
    LRValue = straight;//go straight
  }
  else if (Waypoint[routeCounter].courseToWaypoint <= 180){//---------------- waypoint is in 2/3 qudrant 
    newCourse = angle - Waypoint[routeCounter].courseToWaypoint ;
    if (newCourse < 0)
      newCourse += 360;
    if (newCourse == 180){
      //Serial.print("Turn around");
      LRValue = hardLeft;
    } else if (newCourse < 180) 
      {

        LRValue = slightLeft;
      }
        else if (newCourse > 180) 
      {
        LRValue = slightRight;
      }}
  else if (Waypoint[routeCounter].courseToWaypoint > 180){//-----------------------waypoint is in 1/4 qudrant 
    newCourse = (angle + (360 - Waypoint[routeCounter].courseToWaypoint)); 
    if (newCourse > 360) 
      newCourse -= 360; 
    if (newCourse == 180){
      //Serial.print("Turn around"); 
      LRValue = hardLeft; 
    }else if (newCourse < 180) 
      {
        //Serial.print("Go left  "); 
        //Serial.print(newCourse);  
        //Serial.println(); 
        LRValue = slightLeft;
      }
    else if (newCourse > 180) 
      {
        //Serial.print("Go right   "); 
        //Serial.print(360 - newCourse); 
        //Serial.println(); 
        LRValue = slightRight;
      }}
/*
    Serial.print("   ");
    Serial.print(waypoint[routeCounter].distance);
    Serial.print("   ");
*/    
    if (Waypoint[routeCounter].distance > 10){    //tells the car to go forwards fast as long as the car is at least 10 meters away from the way point
       FRValue = fastForward;
    }else if ((Waypoint[routeCounter].distance <= 10 ) && (Waypoint[routeCounter].distance  > 2)){    // slows the car down as it approaches the way point
        FRValue = slowForward;
    }else if(Waypoint[routeCounter].distance <= 2){    // once within 2 meters of way point it increases the counter for the next point if its the last point it ends the trip
      routeCounter++; 
      if (routeCounter >= waypointCounter){
        FRValue = neutral; 
        LRValue = straight;
        followingGPS = false;
        clearWaypointData();
      }
      
    }
    
    //else {
     // FRValue = neutral;    //idk if we actually need this i think its here for a safety if the program freaks out
    //} 
 
   
    
      
}



// This custom version of delay() ensures that the gps object
// is being "fed".
/*
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    Wire.requestFrom(8, 1);
    while ((Wire.available()))
      gps.encode(Wire.read());
  } while (millis() - start < ms);

}*/




// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  ssGPS.listen();
   // delay(100);

  
  unsigned long start = millis();
  do 
  {
    
    if(ssGPS.isListening())
    {
      //Serial.println(ssGPS.available()); 
    while (ssGPS.available())
    {
     gps.encode((char)ssGPS.read());
   
     //Serial.print((char)ssGPS.read()); 
    }
    //Serial.println("");
  }
  } while (millis() - start < ms);
  ssGPS.stopListening();
}



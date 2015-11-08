#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <HM55B_Compass.h>

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
///////////////////////////////////////////////////////

int ppm[chanel_number];

struct waypoint {
  float latitude; 
  float longitude;
  float courseToWaypoint; 
  float distance; 
  int estimatedArrivalTime;  
 };

#include <Wire.h>
 

//GPS
//////////////////////////////////////////
static const int RXPin = 2, TXPin = 4;
static const uint32_t GPSBaud = 9600;
//////////////////////////////////////



// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ssGPS(RXPin, TXPin);


SoftwareSerial bt(6,7);//tx rx


HM55B_Compass compass(8, 9, 10);



//PPM
int FRValue = 1500;
int LRValue = 1500;

//BlueTooth
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete

/*
#define straight 1500
#define slightLeft 1250
#define slightRight 1750
#define hardLeft 1100
#define hardRight 1900
*/
#define straight 1500
#define slightLeft 1750
#define slightRight 1250
#define hardLeft 1900
#define hardRight 1100


#define neutral 1500
#define slowForward 1600 
#define fastForward 1600 
#define slowBackward 1450
#define fastBackward  1450


#define COMPASSTOLERANCE 10
















void setup()
{
    Wire.begin();
    
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }

  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);  //set the PPM signal pin to the default state (off)
  
  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;
  
  OCR1A = 100;  // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
 
  //Serial.begin(115200);
  //ssGPS.begin(GPSBaud);
  //while(!ssGPS); 
  bt.begin(1200);
  while(!bt); 


  compass.initialize();//start compass
  delay(100); 

  
  getGPSData();
  inputString.reserve(200);
  
}

bool followingGPS = false;
int counter = 0; 

bool test = false;


//Daryl House Staight ahead
waypoint waypoint  = { 37.6960168311, -97.423266917 , 0 , 0 ,0};

void loop()
{
  
 
   

  if(followingGPS)
  {
      getGPSData();
      wichWay();
      bluetooth();
  }else
    bluetooth();
  
  
  Serial.print(LRValue); 
  Serial.print("   "); 
  Serial.println(FRValue); 
  


/*
  Serial.print("FRValue = "); 
  Serial.print(FRValue); 
  Serial.print(" LRValue = "); 
  Serial.print(LRValue);
  Serial.print(" Counter = "); 
  Serial.println(counter++);

  if (counter > 100 )
    counter =0;
*/    
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


void bluetooth()
{
  serialEvent();
    if(stringComplete){
  //Serial.print(inputString); 
//delay(100);
  
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
     followingGPS = false;
     LRValue = straight; 
     FRValue = neutral;
}else if(inputString == "~select\n"){
    //getGPSData(); 
    followingGPS = true;
}else if(inputString == "~a\n"){
     while(FRValue < 2000) 
     {
        ppm[FowardReverse] = FRValue;
        ppm[LeftRight] = LRValue;
        FRValue++;
        Serial.println(FRValue);  
     }
 
}else if(inputString == "~b\n"){
     while(FRValue > 1000) 
     {
        ppm[FowardReverse] = FRValue;
        ppm[LeftRight] = LRValue;
        FRValue--; 
        Serial.println(FRValue); 
     }

}else {
     followingGPS = false;
     LRValue = straight; 
     FRValue = neutral;
}


inputString ="";
stringComplete = false;
}


  
}


void serialEvent() {
  

  if(bt.isListening())
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


void getGPSData()
{

  
  smartDelay(100);

  waypoint.distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      waypoint.latitude, 
      waypoint.longitude);
  //printInt(distance, gps.location.isValid(), 9);

  waypoint.courseToWaypoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      waypoint.latitude, 
      waypoint.longitude);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

}


  

void wichWay()
{
  
  int angle = compass.read();

  angle += 180; 
  /*
    Serial.print("  Compas = "); 
  Serial.print(angle); 
  Serial.print("   Going to = "); 
  Serial.print(waypoint.courseToWaypoint); 
  Serial.print("      ");
*/
  
  float newCourse;
  if ((waypoint.courseToWaypoint <= angle + COMPASSTOLERANCE) && (waypoint.courseToWaypoint >= angle - COMPASSTOLERANCE)){
  //  Serial.print("Go straight");
    LRValue = straight;//go straight
  }
  else if (waypoint.courseToWaypoint <= 180){//---------------- waypoint is in 2/3 qudrant 
    newCourse = angle - waypoint.courseToWaypoint ;
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
  else if (waypoint.courseToWaypoint > 180){//-----------------------waypoint is in 1/4 qudrant 
    newCourse = (angle + (360 - waypoint.courseToWaypoint)); 
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

    
    //distance
    //Serial.print(waypoint.distance);
    if (waypoint.distance > 20)
    FRValue = fastForward; 
    else if(waypoint.distance < 20) 
    FRValue = slowForward;
    else if(waypoint.distance < 5)
    FRValue = neutral;
    else {
    FRValue = neutral; 
    } 
 
    return;
    
      
}



// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{

  unsigned long start = millis();
  do 
  {
    Wire.requestFrom(8, 1);
   
    while ((Wire.available()))
      gps.encode(Wire.read());
  } while (millis() - start < ms);

}


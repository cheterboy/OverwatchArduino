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
#define FowardReverse 2 //set foward and reverse channel
#define LeftRight 1 //set left and right channel
///////////////////////////////////////////////////////

int ppm[chanel_number];


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


//Data from gps fed to the compas
double courseToWaypoint;
double distance; 

//PPM
int FRValue = 1500;
int LRValue = 1500;

//BlueTooth
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete


#define straight 1500
#define slightLeft 1250
#define slightRight 1750
#define hardLeft 1100
#define hardRight 1900

#define neutral 1500
#define slowForward 1600 
#define fastForward 1600 
#define slowBackward 1400
#define fastBackward  1400
















void setup()
{

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
 
  Serial.begin(115200);
  ssGPS.begin(GPSBaud);
  bt.begin(9600);

  compass.initialize();//start compass

  inputString.reserve(200);
  
}

bool followingGPS = false;

void loop()
{
    //compas reading for testing 
    //Serial.print("The compass reading is "); 
    //Serial.println(compass.read());
   
  
  /*
   * This is a test scenario that will either handle the gps or the bluetooh 
   * */
  if(followingGPS)
  {
   //getGPSData();
   wichWay();
  }//else
  bluetooth();
  

  Serial.print("FRValue = "); 
  Serial.print(FRValue); 
  Serial.print(" LRValue = "); 
  Serial.println(LRValue);
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
  Serial.print(inputString); 
//delay(100);
  
if(inputString == "~forward\n"){
     //FRValue = increase(FRValue, FowardReverse, 1600); 
     FRValue = slowForward;
}else if(inputString == "~notforward\n"){
     FRValue = neutral;   
}else if(inputString == "~backward\n"){
     /*
      * Attempt at setting the car in to neutral then sending the reverse signal
      * 
      * 
     //FRValue = decrease(FRValue, FowardReverse, 1499);
     //FRValue = increase(FRValue, FowardReverse, 1500); 
     //FRValue = decrease(FRValue, FowardReverse, 1400);
     FRValue = slowBackwards;
     //ppm[FowardReverse] = FRValue;
    // FRValue = 1500; 
      ppm[FowardReverse] = FRValue;
     delay(500);
     FRValue = 1400; 
      ppm[FowardReverse] = FRValue;
      */
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
    getGPSData(); 
    followingGPS = true;
}else if(inputString == "~a\n"){
      Serial.print("A-ing");
      //FRValue = increase(FRValue, FowardReverse, 2000); 
}else if(inputString == "~b\n"){
  Serial.print("B-ing");
  //FRValue = decrease(FRValue, FowardReverse, 1000); 
}
Serial.print(inputString);

inputString ="";
stringComplete = false;
}


  
}


void serialEvent() {
  
  bt.listen();
  //sometimes the delay helps put the bluetooth in listening mode
  //delay(100);
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

  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  /*
  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);
  */
   smartDelay(100);
    
   distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);
  //printInt(distance, gps.location.isValid(), 9);

  courseToWaypoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  //printFloat(courseToWaypoint, gps.location.isValid(), 7, 2);

  //const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToWaypoint);

  //printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  //printInt(gps.charsProcessed(), true, 6);
  //printInt(gps.sentencesWithFix(), true, 10);
  //printInt(gps.failedChecksum(), true, 9);
  //Serial.println();
  


  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));



}



void wichWay()
{
  //compass.listen();
  int angle = compass.read();
  angle += 180;
  //int angle = 20;
  Serial.print("    "); 
  Serial.print(angle);
  Serial.print("    "); 
  Serial.println(courseToWaypoint);
     
  float newCourse;
  
  if ((courseToWaypoint <= angle + 5) && (courseToWaypoint >= angle - 5)){
    //Serial.print("Go straight");
    LRValue = straight;//go straight
  }
  else if (courseToWaypoint <= 180){//---------------- waypoint is in 2/3 qudrant 
    newCourse = angle - courseToWaypoint ;
    if (newCourse < 0)
      newCourse += 360;
    if (newCourse == 180){
      //Serial.print("Turn around");
      LRValue = hardLeft;
    } else if (newCourse < 180) 
      {
        //Serial.print("Go left "); 
        //Serial.print(newCourse); 
        //Serial.println(); 
        LRValue = slightLeft;
      }
        else if (newCourse > 180) 
      {
        //Serial.print("Go right "); 
        //Serial.print(360 - newCourse); 
        //Serial.println();
        LRValue = slightRight;
      }}
  else if (courseToWaypoint > 180){//-----------------------waypoint is in 1/4 qudrant 
    newCourse = (angle + (360 - courseToWaypoint)); 
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
    if (distance > 20)
    FRValue = fastForward; 
    else if(distance < 20) 
    FRValue = slowForward;
    else if(distance < 5)
    FRValue = neutral;
    else {
    FRValue = neutral; 
    //Serial.print("The distance is unrecognize"); 
    } 

    //Serial.print("The distance is "); 
    //Serial.println(distance);
    return;
    
      
}


// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  ssGPS.listen();
  unsigned long start = millis();
  do 
  {
   
    while (ssGPS.available())
      gps.encode(ssGPS.read());
  } while (millis() - start < ms);
  //ssGPS.stop();
}



/*
static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

//GPS libary printing date and time 
static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}*/

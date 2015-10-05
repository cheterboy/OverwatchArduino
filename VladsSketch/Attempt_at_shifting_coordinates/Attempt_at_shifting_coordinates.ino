#include <TinyGPS++.h>
#include <SoftwareSerial.h>
/*
   This sample code demonstrates the normal use of a TinyGPS++ (TinyGPSPlus) object.
   It requires the use of SoftwareSerial, and assumes that you have a
   4800-baud serial GPS device hooked up on pins 4(rx) and 3(tx).
*/
static const int RXPin = 2, TXPin = 4;
static const uint32_t GPSBaud = 9600;

// The TinyGPS++ object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(115200);
  ss.begin(GPSBaud);

  Serial.println(F("FullExample.ino"));
  Serial.println(F("An extensive example of many interesting TinyGPS++ features"));
  Serial.print(F("Testing TinyGPS++ library v. ")); Serial.println(TinyGPSPlus::libraryVersion());
  Serial.println(F("by Mikal Hart"));
  Serial.println();
  Serial.println(F("Sats HDOP Latitude   Longitude   Fix  Date       Time     Date Alt    Course Speed Card  Distance Course Card  Chars Sentences Checksum"));
  Serial.println(F("          (deg)      (deg)       Age                      Age  (m)    --- from GPS ----  ---- to London  ----  RX    RX        Fail"));
  Serial.println(F("---------------------------------------------------------------------------------------------------------------------------------------"));
}

void loop()
{
//  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;


//north coordinate from house 
//double LONDON_LAT= 37.628392, LONDON_LON = -97.414020;


//CENTER OF BACK YARD
double LONDON_LAT= 37.628177, LONDON_LON = -97.413986;

//west coordinate from house 
//double LONDON_LAT = 37.628103, LONDON_LON = -97.414387;




 // printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
 // printInt(gps.hdop.value(), gps.hdop.isValid(), 5);
 // printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
 // printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
 // printInt(gps.location.age(), gps.location.isValid(), 5);
  //printDateTime(gps.date, gps.time);
  //printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
    Serial.print("Course from gps= "); 
    printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
 

  float courseFromGPS = gps.course.deg();
  //printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  //printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.value()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

      
  //printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  
    Serial.print("Course to london "); 
  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  //const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  //printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  //printInt(gps.charsProcessed(), true, 6);
  //printInt(gps.sentencesWithFix(), true, 10);
  //printInt(gps.failedChecksum(), true, 9);

  float newCourse;
  if (courseToLondon == courseFromGPS)
    Serial.print("Go straight");
  else if (courseToLondon <= 180){//---------------- waypoint is in 2/3 qudrant 
    newCourse = courseToLondon + courseFromGPS;
    if (newCourse > 360)
      newCourse -= 360;
    if (newCourse < 180) 
      {
        Serial.print("Go left  "); 
        Serial.print((newCourse / 360) * 100); 
        
        Serial.println(); 
      }
    else
      {
        Serial.print("Go right   "); 
        Serial.print((newCourse / 360) * 100); 
        Serial.println(); 
      }}
  else if (courseToLondon > 180){//-----------------------waypoint is in 1/4 qudrant 
    newCourse = courseFromGPS - 360 - courseToLondon; 
    if (newCourse < 0) 
      newCourse += 360; 
    if (newCourse == 180)
      Serial.print("Turn around"); 
    else if (newCourse == 0)
      Serial.print("Go straight"); 
    else if (newCourse < 180) 
      {
        Serial.print("Go left top loop  "); 
        Serial.print((newCourse / 360)*100); 
        Serial.println(); 
      }
    else
      {
        Serial.print("Go right top loop   "); 
        Serial.print((newCourse / 360)*100); 
        Serial.println(); 
      }}
  else if (courseToLondon = 0)
     if (newCourse == 180)
      Serial.print("Turn around"); 
     else if (newCourse == 0)
      Serial.print("Go straight"); 
     else if (newCourse < 180) 
      {
        Serial.print("Go left bottom loop  "); 
        Serial.print((newCourse / 360)*100); 
        Serial.println(); 
      }
    else
      {
        Serial.print("Go right bottom loop  "); 
        Serial.print((newCourse / 360)*100); 
        Serial.println(); 
      }
   
 





  /*
  Serial.print("course to london= ");
  Serial.print(courseToLondon);
  Serial.print("   ");

  Serial.print("course from gps= ");
  Serial.print(courseFromGPS);
  Serial.print("   ");
  
  Serial.print( courseFromGPS - courseToLondon);
  if((courseToLondon - courseFromGPS)    ==  0){
    Serial.print(" Go straght");//go straight 


  }  else if( courseToLondon - courseFromGPS   > 180)
  { 
    Serial.print(" Go left");//go left


  }
  else if( courseToLondon - courseFromGPS   < 180){


    Serial.print(" Go right");//go right 
  }
    
  
  */
  
  
  
  Serial.println();
  
  smartDelay(500);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));
}

// This custom version of delay() ensures that the gps object
// is being "fed".
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

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
}

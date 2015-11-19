//GPS

#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <Wire.h>

//#include <HM55B_Compass.h>
//compass
#include <HMC5883L.h>
#define COMPASSTOLERANCE 5


static void smartDelay(unsigned long ms);


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

int FRValue = 1500;
int LRValue = 1500;






typedef struct waypoint {
  float latitude = 0.0000000;
  float longitude = 0.0000000;
  float courseToWaypoint;
  float distance;
};



// The TinyGPS++ object
TinyGPSPlus gps;
SoftwareSerial ssGPS(5, 4);

//HM55B_Compass compass(8, 9, 10);
HMC5883L compass;
int error = 0;



void setup()
{

  Serial.begin(250000);
  Wire.begin();
 // Wire.begin(8);
  compass = HMC5883L(); // Construct a new HMC5883 compass.

  //Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));

    //Serial.println("Setting measurement mode to continous.");
    error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));


  ssGPS.begin(9600);

  Serial.println("Initialization Done");
}






//used to controll gps mode
bool followingGPS = false;
waypoint Waypoint[MAXWAYPOINTS];
//number of waypoints in buffer
int waypointCounter = 0;
//the waypoint that the uad is on
int routeCounter = 0;



void getGPSData();
void wichWay();


void loop()
{
	u16 x;

  getGPSData();
  wichWay();

  Serial.print("Lat "); 
  Serial.print( gps.location.lat()); 
  Serial.print("  lng "); 
  Serial.println( gps.location.lng());  


}


void getGPSData()
{
  smartDelay(1000);

  Waypoint[routeCounter].distance =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      Waypoint[routeCounter].latitude,
      Waypoint[routeCounter].longitude);
  

  Waypoint[routeCounter].courseToWaypoint =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      Waypoint[routeCounter].latitude,
      Waypoint[routeCounter].longitude);



  TinyGPSPlus::
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
  if (heading < 0)
    heading += 2 * PI;

  // Check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // Convert radians to degrees for readability.
  int angle = heading * 180 / M_PI;

  Wire.beginTransmission(9); // transmit to device #9
  Wire.write(angle);              // sends x 
  Wire.endTransmission();    // stop transmitting


  Serial.print("Heading  "); 
  Serial.println(angle); 


/*
  float newCourse;
  if ((Waypoint[routeCounter].courseToWaypoint <= angle + COMPASSTOLERANCE) &&
      (Waypoint[routeCounter].courseToWaypoint >= angle - COMPASSTOLERANCE)) {
    //  Serial.print("Go straight");
    LRValue = straight;//go straight
  }
  else if (Waypoint[routeCounter].courseToWaypoint <= 180) { //---------------- waypoint is in 2/3 qudrant
    newCourse = angle - Waypoint[routeCounter].courseToWaypoint ;
    if (newCourse < 0)
      newCourse += 360;
    if (newCourse == 180) {
      //Serial.print("Turn around");
      LRValue = hardLeft;
    } else if (newCourse < 180)
    {

      LRValue = slightLeft;
    }
    else if (newCourse > 180)
    {
      LRValue = slightRight;
    }
  }
  else if (Waypoint[routeCounter].courseToWaypoint > 180) { //-----------------------waypoint is in 1/4 qudrant
    newCourse = (angle + (360 - Waypoint[routeCounter].courseToWaypoint));
    if (newCourse > 360)
      newCourse -= 360;
    if (newCourse == 180) {
      //Serial.print("Turn around");
      LRValue = hardLeft;
    } else if (newCourse < 180)
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
    }
  }


 
  if (Waypoint[routeCounter].distance > 10) {   //tells the car to go forwards fast as long as the car is at least 10 meters away from the way point
    FRValue = fastForward;
  } else if ((Waypoint[routeCounter].distance <= 10 ) && (Waypoint[routeCounter].distance  > 2)) {   // slows the car down as it approaches the way point
    FRValue = slowForward;
  } else if (Waypoint[routeCounter].distance <= 2) {  // once within 2 meters of way point it increases the counter for the next point if its the last point it ends the trip
    routeCounter++;
    if (routeCounter >= waypointCounter) {
      FRValue = neutral;
      LRValue = straight;
      followingGPS = false;
      clearWaypointData();
    }

  }


*/

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
  //    unsigned long start = millis();
  //    do
  //    {
  while (ssGPS.available())
  {
    gps.encode((char)ssGPS.read());
  }
  //    } while (millis() - start < ms);
}



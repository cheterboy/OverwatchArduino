//this programm will put out a PPM signal

//////////////////////CONFIGURATION///////////////////////////////
#define chanel_number 6  //set the number of chanels
#define default_servo_value 1500  //set the default servo value
#define PPM_FrLen 22000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 300  //set the pulse length
#define onState 1  //set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin 9  //set PPM signal output pin on the arduino
#define FowardReverse 0 //set foward and reverse channel
#define LeftRight 1 //set left and right channel
//////////////////////////////////////////////////////////////////


int courseToWaypoint; 

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




#include <Wire.h>
#include <SoftwareSerial.h> 

#include <HMC5883L.h>
#define COMPASSTOLERANCE 5
HMC5883L compass;
int error = 0;


/*this array holds the servo values for the ppm signal
 change theese values in your code (usually servo values move between 1000 and 2000)*/
int ppm[chanel_number];
int FRValue = 1500; 
int LRValue = 1500; 



void setup(){  
  //initiallize default ppm values
  for(int i=0; i<chanel_number; i++){
    ppm[i]= default_servo_value;
  }
    Wire.begin();

   compass = HMC5883L(); // Construct a new HMC5883 compass.

  //Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
 
    //Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if (error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  Serial.begin(9600);

  
 
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
}

void loop(){

  //Serial.println(Wire.requestFrom(8, 1));
  //int FRValue = (1000+(x/10*100));
  //int LRValue = (1000+((x%10)*100));

  Serial.print(FRValue);
  Serial.print("      "); 
  Serial.print(LRValue); 
  Serial.print("      "); 
 

  Wire.requestFrom(8, 20);    // request 6 bytes from slave device #8
  String inputString; 
  while (Wire.available()) { // slave may send less than requested
    inputString += (char)Wire.read(); // receive a byte as character
   
  }
 Serial.print(inputString);         // print the character

  //sets the lr values based on compas reading
  //needs global variable courseToWaypoint
  wichWay();
  
 ppm[FowardReverse] = 1500; 
 ppm[LeftRight] = 1500; 
 delay(10);
}

ISR(TIMER1_COMPA_vect){  //leave this alone
  static boolean state = true;
    cli();
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

  sei();
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

  int newCourse;
  if ((courseToWaypoint <= angle + COMPASSTOLERANCE) &&
      (courseToWaypoint >= angle - COMPASSTOLERANCE)) {
    //  Serial.print("Go straight");
    LRValue = straight;//go straight
  }
  else if (courseToWaypoint <= 180) { //---------------- waypoint is in 2/3 qudrant
    newCourse = angle - courseToWaypoint ;
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
  else if (courseToWaypoint > 180) { //-----------------------waypoint is in 1/4 qudrant
    newCourse = (angle + (360 - courseToWaypoint));
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

}





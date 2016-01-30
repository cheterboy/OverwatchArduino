//Simple Digital Calliper Reader
//See http://j44industries.blogspot.com/

// Pin Declarations
int dataIn = 11;
int clockIn = 12;

// Variables
int clock = 1;
int lastClock = 1;
unsigned long time = 0;
unsigned long timeStart = 0;
int out = 0;


void setup() {
  // Pin Set Up
  pinMode(dataIn, INPUT);     
  pinMode(clockIn, INPUT);  


  Serial.begin(115200);
  Serial.println("Ready: ");
}


void loop(){


  lastClock = clock;
  clock = digitalRead(clockIn);

  if (lastClock == 1 && clock == 0){
   
    out = digitalRead(dataIn)+digitalRead(dataIn)+digitalRead(dataIn); // Tripple sampling to remove glitches
     //Serial.println("Here"); 
    if((micros() - time) > 800){
      Serial.println(" ");
    }
    else if((micros() - time) > 400){
      Serial.print("  ");
    }

    if (out > 1){
      Serial.print("1");
    }
    else{
      Serial.print("0");
    }
    Serial.print(",");
    time = clock;
  }
}


// Watch video here: https://www.youtube.com/watch?v=o09-9n5WrxE

//Connection pins provided in the diagram at the beginning of video

// library provided and code based on: https://github.com/Seeed-Studio/IMU_10DOF
#include <Wire.h>
#include <I2Cdev.h>
#include <MPU9250.h>

// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU9250 accelgyro;
I2Cdev   I2C_M;

int16_t ax, ay, az;
int16_t gx, gy, gz;
int16_t mx, my, mz;
float Axyz[3];
int rightbutton = 4;
int leftbutton = 5;

void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  Wire.begin();
  Serial.begin(9600);
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU9250 connection successful" : "MPU9250 connection failed");
  delay(1000);
  pinMode(leftbutton, OUTPUT);
  pinMode(rightbutton, OUTPUT);
  Mouse.begin();
}

void loop()
{
  getAccel_Data();

  float pitchrad = atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2])); // radians
  float rollrad = atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2])); // radians

  float rolldeg = 180 * (atan(Axyz[1] / sqrt(Axyz[0] * Axyz[0] + Axyz[2] * Axyz[2]))) / PI; // degrees
  float pitchdeg = 180 * (atan(Axyz[0] / sqrt(Axyz[1] * Axyz[1] + Axyz[2] * Axyz[2]))) / PI; // degrees
  
  float Min = -15;//-30, -45, -15
  float Max = 15;// 30, 45, 15
  int mapX = map(pitchdeg, Min, Max, -6, 6);
  int mapY = map(rolldeg, Min, Max, -6, 6);
  Mouse.move(-mapX, mapY, 0);
  Serial.print(pitchdeg);
  Serial.print(",");
  Serial.print(rolldeg);
  Serial.print(" - ");
  int leftstate = digitalRead(leftbutton);
  int rightstate = digitalRead(rightbutton);
  //Serial.print(leftstate); Serial.print(rightstate);
  if (leftstate == HIGH) {
    Serial.print(" Left Click! ");
    Mouse.press(MOUSE_LEFT);
  }
  if (leftstate == LOW) {
    Mouse.release(MOUSE_LEFT);
  }
  if (rightstate == HIGH) {
    Serial.print(" Right Click! ");
    Mouse.click(MOUSE_RIGHT);
  }
  Serial.println();
}

void getAccel_Data(void)
{
  accelgyro.getMotion9(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz);
  Axyz[0] = ((double) ax / 256) - 1.6;
  Axyz[1] = ((double) ay / 256) - 2.1;
  Axyz[2] = (double) az / 256;
}




void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); 

}

void loop() {
  // put your main code here, to run repeatedly:
  String one = "-100.12"; 
  String two = "3456"; 


  double testThree = -100.123456; 
  float testOne = 0; 
  float testTwo = 0; 

  testOne = strtod(one.c_str(),NULL); 
  testTwo = atoi(two.c_str());

  testTwo /= 1000000; 

  Serial.print(testOne, 2); 
  Serial.print(" "); 
  Serial.println(testTwo,6); 
  Serial.println(testThree,6); 
  Serial.println((testOne + testTwo) , 6); 

  delay(1000); 

}

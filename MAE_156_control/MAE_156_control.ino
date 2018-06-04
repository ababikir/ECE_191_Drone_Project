#include <Servo.h>
#include <servo.h>//Using servo library to control ESC

Servo esc; //Creating a servo class with name as esc
Servo myservo;  // create servo object to control a servo

void setup()
{
  esc.attach(8); //Specify the esc signal pin,Here as D8
  esc.writeMicroseconds(1000); //initialize the signal to 1000
  myservo.attach(11);
  Serial.begin(9600);
}
void loop()
{
  int val; //Creating a variable val
  //val= analogRead(A1); //Read input from analog pin a0 and store in val
  //val= map(val, 0, 1023,900,2100); //mapping val to minimum and maximum(Change if needed) 
  val = 1800;
  esc.writeMicroseconds(val); //using val as the signal to esc

  myservo.write(90);
  delay(10000);
  for(int i = 40;i < 140; i+= 1) {
    myservo.write(i);
    delay(100);
  }
  for(int i = 140;i > 40; i-= 1) {
    myservo.write(i);
    delay(100);
  }
}


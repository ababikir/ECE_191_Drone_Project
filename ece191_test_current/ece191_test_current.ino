/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>

Servo myservo;  // create servo object to control a servo
// twelve servo objects can be created on most boards
//Servo myservo2;
//Servo myservo3;
int pos = 0;    // variable to store the servo position

void setup() {
  myservo.attach(11);  // attaches the servo on pin 9 to the servo object
 // myservo2.attach(7);
 // myservo3.attach(4);
 Serial.begin(9600);
 myservo.write(90);
 delay(500);
 myservo.write(100);
 delay(500);
 myservo.write(90);
 delay(1000);
}

void loop() {
  double pos2;
  for (pos = 35; pos <= 145; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
  //  myservo2.write(pos);
  //  myservo3.write(pos);

    delay(15);                       // waits 15ms for the servo to reach the position
  }/*
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo.write(pos);              // tell servo to go to position in variable 'pos'
   // myservo2.write(pos);
   // myservo3.write(pos);
    delay(15);                       // waits 15ms for the servo to reach the position
  }*/
  Serial.println(pos);
  //pos = 90;
  //myservo.write(pos);
  //delay(500);
  //pos2 = myservo.read();
  //Serial.println(pos2);
  //pos = 80;
  //myservo.write(pos);
  //delay(500);
  
  
}


/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>



Servo myservo1;  // create servo object to control a servo
// twelve servo objects can be created on most boards

Servo myservo2; 

int pos = 0;    // variable to store the servo position
int cur_angle = 0;

void setup() {
    Serial.begin(9600);
  myservo1.attach(4);  // attaches the servo on pin 4 to the servo object
  //myservo2.attach(5);  // attaches the servo on pin 9 to the servo object


}

void loop() {

  
  for (pos = 90; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1000);                       // waits 15ms for the servo to reach the position
    
cur_angle = myservo1.read();    
Serial.println(cur_angle); 
  }

myservo1.read();
  
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos);              // tell servo to go to position in variable 'pos'
    delay(1000);                       // waits 15ms for the servo to reach the position


    
cur_angle = myservo1.read();    
Serial.println(cur_angle); 

    
  }
    
cur_angle = myservo1.read();    
Serial.println(cur_angle); 
}








/*
  for (pos = 0; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo2.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo2.write(pos);              // tell servo to go to position in variable 'pos'
    delay(15);                       // waits 15ms for the servo to reach the position
  }
*/



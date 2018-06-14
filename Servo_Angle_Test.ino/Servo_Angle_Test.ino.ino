/* Sweep
 by BARRAGAN <http://barraganstudio.com>
 This example code is in the public domain.
 modified 8 Nov 2013
 by Scott Fitzgerald
 http://www.arduino.cc/en/Tutorial/Sweep
*/

#include <Servo.h>
#include <servo.h>
#include <Wire.h>

Servo myservo1;  // create servo object to control flap 1
Servo myservo2;  // create servo object to control flap 2
Servo myservo3;  // create servo object to control flap 3
Servo myservo4;  // create servo object to control flap 4


Servo ESC1;
Servo ESC2;

int pos = 90;    // variable to store the servo position
int cur_angle_1 = 0;
int cur_angle_2 = 0;
int cur_angle_3 = 0;
int cur_angle_4 = 0;



void setup() {

 

 ESC1.attach(2);
 ESC1.writeMicroseconds(1000);
 ESC2.attach(3);
 ESC2.writeMicroseconds(1000);


  Serial.begin(9600);
    




  
    /*
 // myservo1.attach(7);   // Servo Motor for Flap 1 (Signal Pin 4)
 myservo2.attach(12);     // Servo Motor for Flap 2 (Signal Pin 10)
 // myservo3.attach(10);  // Servo Motor for Flap 3 (Signal Pin 12)
  myservo4.attach(5);   // Servo Motor for Flap 4 (Signal Pin 8)

//myservo1.write(pos);              // tell servo 1 (Flap 1) to go to position in variable 'pos'
delay(15);
myservo2.write(pos);              // tell servo 2 (Flap 2) to go to position in variable 'pos'
delay(15);
//myservo3.write(pos);              // tell servo 3 (Flap 3) to go to position in variable 'pos'
delay(15);
myservo4.write(pos);              // tell servo 4 (Flap 4) to go to position in variable 'pos'
delay(15);

*/


}



void loop () 

{
  //ESC1.writeMicroseconds(1300);
  //delay(1000);


/*
Serial.println(cur_angle_1); 
Serial.println(cur_angle_2); 
Serial.println(cur_angle_3); 
Serial.println(cur_angle_4); 
*/
/*
myservo1.write(pos);              // tell servo 1 (Flap 1) to go to position in variable 'pos'
myservo2.write(pos);              // tell servo 2 (Flap 2) to go to position in variable 'pos'
myservo3.write(pos);              // tell servo 3 (Flap 3) to go to position in variable 'pos'
myservo4.write(pos);              // tell servo 4 (Flap 4) to go to position in variable 'pos'
*/


/*
//myservo1.read();
cur_angle_2 = myservo2.read();    
//myservo3.read();    
//myservo4.read(); 

delay(1000);                       // waits 3 seconds for the servo to reach the position


//Serial.println(cur_angle_1); 
Serial.println(cur_angle_2); 
//Serial.println(cur_angle_3); 
//Serial.println(cur_angle_4); 

*/

}


























 /*   
cur_angle_1 = myservo1.read();    
cur_angle_2 = myservo2.read();    
cur_angle_3 = myservo3.read();    
cur_angle_4 = myservo4.read();    

Serial.println(cur_angle_1); 
Serial.println(cur_angle_2); 
Serial.println(cur_angle_3); 
Serial.println(cur_angle_4); 
*/












/*

void loop() {

  
  for (pos = 90; pos <= 180; pos += 1) { // goes from 0 degrees to 180 degrees
    // in steps of 1 degree
    myservo1.write(pos);              // tell servo 1 to go to position in variable 'pos'
    myservo2.write(pos);              // tell servo 2 to go to position in variable 'pos'
    myservo3.write(pos);              // tell servo 3 to go to position in variable 'pos'
    myservo4.write(pos);              // tell servo 4 to go to position in variable 'pos'

    delay(3000);                       // waits 3 seconds for the servo to reach the position
    
cur_angle_1 = myservo1.read();
cur_angle_2 = myservo2.read();    
cur_angle_3 = myservo3.read();    
cur_angle_4 = myservo4.read();    
    
Serial.println(cur_angle_1); 
Serial.println(cur_angle_2); 
Serial.println(cur_angle_3); 
Serial.println(cur_angle_4); 

  }

myservo1.read();
myservo2.read();
myservo3.read();
myservo4.read();

  
  for (pos = 180; pos >= 0; pos -= 1) { // goes from 180 degrees to 0 degrees
    myservo1.write(pos);              // tell servo 1 to go to position in variable 'pos'
    myservo2.write(pos);              // tell servo 2 to go to position in variable 'pos'
    myservo3.write(pos);              // tell servo 3 to go to position in variable 'pos'
    myservo4.write(pos);              // tell servo 4 to go to position in variable 'pos'

    delay(3000);                       // waits 15ms for the servo to reach the position


    
cur_angle_1 = myservo1.read();    
cur_angle_2 = myservo2.read();    
cur_angle_3 = myservo3.read();    
cur_angle_4 = myservo4.read();    

Serial.println(cur_angle_1); 
Serial.println(cur_angle_2); 
Serial.println(cur_angle_3); 
Serial.println(cur_angle_4); 

  }
    
cur_angle_1 = myservo1.read();
cur_angle_2 = myservo2.read();    
cur_angle_3 = myservo3.read();    
cur_angle_4 = myservo4.read();    
Serial.println(cur_angle_1); 
Serial.println(cur_angle_2); 
Serial.println(cur_angle_3); 
Serial.println(cur_angle_4); 
}


*/





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



#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>


Servo myservo;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
double calAccX = 0, calAccY = 0;
double calGyroX = 0, calGyroY = 0, calGyroZ = 0;
double angle_pitch = 0, angle_roll = 0, angle_yaw = 0;
double prev_time = 0;
double dt;
double cur_time = 0, prev_time;
double filterConstant = .9996;

// Set-up servos

double PID, error, previous_error, error_total;
double pid_p = 0, pid_i = 0;, pid_d = 0;



// PID constants (WILL ADJUST)

/*********************************************************************
Ammar: Recommendation by CSE 190 Quadcopter Professor:

1) Set all values : kp, ki, kd = 0  or  let kp,kd = 1   and ki = 0.01
2) Slowly increase P until you see that system is starting to oscillate. 
3) Then, slowly start increasing kd until you see that system starts to oscillate. 
4) Now, slowly increase ki to get rid of the offset.

OR


Use Ziegler Method:

1) Make I and D gains = 0 and slowly increase P. 
2) Increase P until the output has stable and consistent oscillations (No overshoot, small steady state error). 


***********************************************************************/


double kp = 3;
double ki = .01;
double kd = 2;

// Initial values or servos
double initial_pos = 90; // 180/2 for full range of motion
double desired_angle = 0; // to stay steady

// Function to set-up sensor
void configureSensor(void)
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}

// Function to calibrate sensor, ideally should get (0,0,0)
boolean calibrateSensor(int numData)
{
  sensors_event_t accel, mag, gyro, temp;
  int i;
  double calibrationValue = 0.3;
  double gyroX, gyroY, gyroZ;
  
  for (i = 0; i < numData; i += 1){

    lsm.getEvent(&accel, &mag, &gyro, &temp); 
    calAccX += accel.acceleration.x;
    calAccY += accel.acceleration.y;
    calGyroX += gyro.gyro.x;
    calGyroY += gyro.gyro.y;
    calGyroZ += gyro.gyro.z;

  }
  calGyroX /= numData;
  calGyroY /= numData;
  calGyroZ /= numData;
  calAccX /= numData;
  calAccY /= numData;

  // Test calibration
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  
  gyroX = gyro.gyro.x - calGyroX;
  gyroY = gyro.gyro.y - calGyroY;
  gyroZ = gyro.gyro.z - calGyroZ;
  
  if(gyroX > calibrationValue || gyroX < -1*calibrationValue) {
    Serial.print("Bad calibration value: ");
    Serial.println(gyroX);
    return false;
  }
  if(gyroY > calibrationValue || gyroY < -1*calibrationValue) {
    Serial.print("Bad calibration value: ");
    Serial.println(gyroY);
    return false;
  }
  if(gyroZ > calibrationValue || gyroZ < -1*calibrationValue) {
    Serial.print("Bad calibration value: ");
    Serial.println(gyroZ);
    return false;
  }
  return true;
}


void setup() {
  boolean goodCalibration = false;
#ifndef ESP8266
  while (!Serial);     // will pause Zero, Leonardo, etc until serial console opens
#endif
  Serial.begin(9600);
  Serial.println(F("LSM9DS0 9DOF Sensor Test")); Serial.println("");
  
  /* Initialise the sensor */
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections */
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  configureSensor();

  while(!goodCalibration) {
    Serial.println("Needs New Calibration");
    goodCalibration = calibrateSensor(2000);
  }


  // Attach servos to arduino using the servo.h library and attach method
  
  
  cur_time = millis();
}

void loop() {

  sensors_event_t accel, mag, gyro, temp;
  double accelX, accelY, accelZ;
  double gyroX, gyroY, gyroZ;
  double convertTimeRadians;
  double acc_mag_vector;
  double angle_pitch_acc, angle_roll_acc;
  double convertToDegrees;

  prev_time = cur_time;
  cur_time = millis();
  dt = (cur_time - prev_time) / (1000.0);

  lsm.getEvent(&accel, &mag, &gyro, &temp);
 
  accelX = accel.acceleration.x - calAccX;
  accelY = accel.acceleration.y - calAccY;
  accelZ = accel.acceleration.z;
  
  gyroX = gyro.gyro.x - calGyroX;
  gyroY = gyro.gyro.y - calGyroY;
  gyroZ = gyro.gyro.z - calGyroZ;

  // Gyro Integration
  prev_time = millis();
  angle_pitch += (gyroX*dt); 
  angle_roll += gyroY*dt; 

  // Adjust for yaw rotation
  convertTimeRadians = dt * PI/ 180.0;
  angle_pitch += angle_roll * sin(gyroZ * convertTimeRadians);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyroZ * convertTimeRadians);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  //Accelerometer angle calculations
  acc_mag_vector = sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));  //Calculate the total accelerometer vector
  convertToDegrees = 1 / (PI / 180.0);
  angle_pitch_acc = asin((double)accelY/acc_mag_vector)* convertToDegrees;       //Calculate the pitch angle
  angle_roll_acc = asin((double)accelX/acc_mag_vector)* -convertToDegrees;       //Calculate the roll angle

  angle_pitch = angle_pitch * filterConstant + angle_pitch_acc * (1.0-filterConstant);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angle_roll = angle_roll * filterConstant + angle_roll_acc * -(1.0-filterConstant);        //Correct the drift of the gyro roll angle with the accelerometer roll angle


  

  // Calculate error between either angle_pitch/ angle_roll and desired angle



  /*

  Tuesday May 15th, 2018:

  
  Ammar:
  

Error will always be the difference between your current value and your desired value. 

If error > 0 => You need to decrease PID value. 

If error < 0 => You need to increase PID value.

   */
   


  
  error_angle_pitch =  (desired_angle_pitch) - (angle_pitch)   // The pitch angle will have a specific amount of error associated with it. 

  error_angle_roll =  (desired_angle_roll) - (angle_roll) ;     // The roll angle will have a specific amount of error associated with it.

  error = (error_angle_pitch) + (error_angle_roll);     // The total error from both pitch and roll will be the sum of the errors.

  delta_error = error - previous_error;   // This is so that we can find the difference in error when we're finding the derivative of error.



  
  
  // Calculate pid_p = kp * error;

  /*
Tuesday May 15th, 2018:
   
Ammar:

Do we:

1) Add up the TOTAL error from the pitch AND roll angle and make it = error_total? 

OR 

2) Have a pid_P_pitch  AND a pid_P_roll?

From my understanding, it should only be ONE proportional controller.

   */


   
  pid_p = kp * error;
  








  /*

Tuesday May 15th, 2018:

Ammar:

Source: https://sites.google.com/a/eng.ucsd.edu/quadcopterclass/labs/lab-7-flight-control

This one is a little trickier.  The obvious answer is to just sum the error term forever.  
The problem here is that if you happen to hold your test platform steady with your hand, the integral will grow rapidly. 
For instance, if you hold it in a position where the error term is positive for 10 seconds, the integral term will come to a large positive value.  
When you release the platform, the platform will move sharply to a position with negative error (because it has been trying for 10 seconds to reduce the positive error).  
It will then take a while (maybe another 10 seconds) to "unwind" the integral by adding in the negative error.

There are couple of solution to this:
You can have the integral decay over time.  For instance, instead computing sum = sum + e each iteration, you could do sum = 3*sum/4 + e.
You can also just bound the integral at some value.  Some trial and error may be required to find a reasonable bound.



   
   */



  
  // Calculate pid_i and only use for values between -3 and 3 degrees?

  // integral = integral + (error*iteration_time) if and only if:

  // -3 < angle_pitch < 3 
 
  // -3 < angle_roll < 3

/*

Tuesday May 15th, 2018:

Ammar:


So we only want to use the integral control if the error is very small. 

We should be careful with this because the integral term lets the controller handle errors that are accumulating over time. 
This is good when we need to get rid of the steady state error.
The problem is is that if we  have a large KI (which I don't think we will) we would be trying to correct the error over time. That means, it could interfere with the response for dealing with
changes in the current. I've read online that this is one of the causes of instability in the PID controller.


 */

 
  if (angle_roll < 3  || angle_roll > -3 ) {
  
  pid_i = ki + (error*cur_time);
    
  }


  else if (angle_pitch < 3 || angle_pitch <  -3) {
   
  pid_i = ki + (error*cur_time);
  }

  
  else {
    pid_i = 0;
  }






  // Calculate pid_d


  /*

Tuesday May 15th, 2018:

Ammar:

Source: https://sites.google.com/a/eng.ucsd.edu/quadcopterclass/labs/lab-7-flight-control

The easiest way to compute the derivative is by dividing the change in error by the change in time.  Just remember:
To measure time in seconds.
Be sure you have the sign right on e(t) and de/dt.  If you get it backwards nothing will work.
To use the actual elapse time since your last measurement.
It is also possible to use the raw output from the gyroscope (since you are mostly integrated it to get the estimate of your pitch angle).



So, in this case,

Our Difference in Error = Current Error - previous_error

Difference in Time = dt (where dt is in seconds)

   
   */


pid_d = (error - previous_error) / (dt);    // Finding derivate of error


  




  // Calculate PID

  PID = (pid_p) + (pid_i) + (pid_d);




  

  // Make sure that PID does not reach more than let's say 60 positions so 90+60 = 150 or 90-60 = 30


// I wasn't too sure what I was supposed to do for this part. What do you mean by "reach" more than 60 positions?


    
  
  
  


  


/*
Tuesday May 15th, 2018:


Ammar:

I use servo.write() to write a value to the servo. This value would control the shaft of the servo motor.

This value would set the angle of the shaft.

Setting different values in the servo.write() argument means different things:


1) Setting "0" means full-speed in its direction 

2) Setting "180" means full-speed in the opposite direction 

3) Setting "90" or somewhere close to it would set the shaft to the mid-point which means almost not moving.


 */

  // Adjust position by adding PID to it


  cur_servo_angle = servo.read()  // Read the current angle of the servo (the value passed to the last call to servo.write() ). The argument inside read() is going to be the signal pin for the servo motor.

  new_servo_angle = cur_servo_angle + PID; // Calculate the new_servo_angle by adding PID to the cur_servo_angle. Also, just to make sure I understand this, why are we adding PID to the servo angle? Won't we have to subtract in some cases?

  /*

  Ok I see what we're doing here. We want to find the current servo angle and then find the new servo angle. 

  Nut, we don't want to adjust the servo to the new servo angle right away because we don't know if this new angle is going to be past our threshold value for the servo motor. 

  So, for example, if we get a huge angle that's bigger than like 160 or something like that, we don't want to bug out our system by writing those values to the servo motor shaft.

   
   */
  
  








  






  // Ensure again that new position does not violate anything


  /*

   Ammar:

  From what I remember and understand from our meeting and conversations, you mean you don't want the servo to go past 180 or something like.

  I'm just going to cap it at like 160 degrees so we can ensure it won't go past its max angle.


  What I don't understand, if it does violate, what should we do?

  For now, I'll just say don't only add half of the PID amount and not the full PID amount. Else, make the new servo angle the one we just calculated.


  
   */




// First check if the new_servo_angle that we calculated is past 160 degrees.

  if (  new_servo_angle > 160 ) {


  new_servo_angle = cur_servo_angle + ( (1/2) * PID); //I'll only add half of the PID just for now because I think that adding the full PID value would make the new angle go past our threshold. 
    
  }

  else {

  new_servo_angle = new_servo_angle;    // Whatever new_servo_angle we just calculated, just make that the new_angle.
    
  }

  
  








  

  // Write new position to servo


  myservo.attach(); // We have to put in the specific servo motor we want to write to. So inside attach(), the argument is going to be the signal pin of the servo motor we want to change.
  new_servo_angle = servo.write(cur_servo_angle + PID); // adjusting position of servo by adding actual PID value to current servo angle.








  

  // set previous error = error


  previous_error = error;
  




}


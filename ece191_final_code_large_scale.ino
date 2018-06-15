/* This code is designed for the small prototype. The tuning is also optimized for the prototype.*/

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <servo.h>//Using servo library to control ESC
#include <Adafruit_MPL3115A2.h> // Altitude sensor library
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h> // Gyro and Accelerometer library

// Make servo and ESC objects
Servo myservo_pitch;
Servo myservo_roll;
Servo esc;

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000, make object for sensor
double calAccX = 0, calAccY = 0; // Calibratation initialization values accelerometer
double calGyroX = 0, calGyroY = 0, calGyroZ = 0; // Calibratation initialization values gyroscope
double angle_pitch = 0, angle_roll = 0, angle_yaw = 0; // intitial pitch, roll and yaw angles
double prev_time = 0; // for time recording intiatilize to zero
double dt; // for timing how long a loop takes
double cur_time = 0; // for time recording
double filterConstant = .98; // filter constant for complimentary filter .98 works pretty well! Could try other values though

double PID_pitch = 0, PID_roll = 0; // PID values for pitch and roll angles set to zero
double previous_error_pitch = 0, previous_error_roll = 0; // for saving previous error values for PID set to zero
double integration_cuttoff_angle = 1; // for integration cuttoff to only sum values of error is 1 degree

double kp = .07;  // desirable kp value for prototype system
double ki = 0; // set to zero because integral part is turned off (not needed)
double kd = 0.03;   // desirable kd value for prototype system


double initial_pos = 90; // initial values of servo, 180/2 for full range of motion
double desired_angle_pitch = 0, desired_angle_roll = 0; // desired angle for the drone to be at
int servoLowerBound = 40, servoUpperBound = 140; // the bounds on the servos to make sure the flap does not hit the side of the drone
double cur_servo_angle_pitch = initial_pos, cur_servo_angle_roll = initial_pos; // set current servo angle to be the initial position
double new_servo_angle_pitch, new_servo_angle_roll; // declare new servo angles for pitch and roll
double pid_i_pitch = 0, pid_i_roll = 0; // initialize integral part of pitch and roll to be zero, they will act like a sum if the integral component is used

// Function to set-up LSM9DS0 sensor
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

/*Function to calibrate sensors, this only finds a baseline of what the values from the sensor are and 
 * will fail based on the tolerance from calibrationValue. It takes in a number that will be the amount
 * of times the sensor will grab data for calibration. Larger values of numData will yield better results.
*/
boolean calibrateSensor(int numData)
{
  sensors_event_t accel, mag, gyro, temp;
  int i;
  double calibrationValue = .5; // this can change depending on how much tolerance the user is willing to have
  double gyroX, gyroY, gyroZ, accX, accY; // initialize variables
  
  for (i = 0; i < numData; i += 1){

    lsm.getEvent(&accel, &mag, &gyro, &temp); // get data from sensor
    // sum up all sensor readings for these values
    calAccX += accel.acceleration.x; 
    calAccY += accel.acceleration.y;
    calGyroX += gyro.gyro.x;
    calGyroY += gyro.gyro.y;
    calGyroZ += gyro.gyro.z;

  }
  
  // divide the sums of calibration by the number of data points taken to get an average
  calGyroX /= numData;
  calGyroY /= numData;
  calGyroZ /= numData;
  calAccX /= numData;
  calAccY /= numData;

  // test calibration by getting one more event
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // subtract new measurement from calibration
  gyroX = gyro.gyro.x - calGyroX;
  gyroY = gyro.gyro.y - calGyroY;
  gyroZ = gyro.gyro.z - calGyroZ;
  accX = accel.acceleration.x - calAccX;
  accY = accel.acceleration.y - calAccY;  

  // check to see if the new measurement violates the calibrationValue
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
  if(accX > calibrationValue || accX < -1*calibrationValue) {
    Serial.print("Bad calibration value: ");
    Serial.println(accX);
    return false;
  }
  if(accY > calibrationValue || accY < -1*calibrationValue) {
    Serial.print("Bad calibration value: ");
    Serial.println(accY);
    return false;
  }
  return true;
}

/* Turns on motor based on a speed*/
void turnOnMotor(int motorSpeed) {
  esc.writeMicroseconds(motorSpeed);
}

/* Updates the current angles based on measurements from the sensors. Records an event from
 * sensors and then does mathematical operations to conver it to an angle. It then filters 
 * these values with the complimentary filter.
 * Website reference: https://www.youtube.com/watch?v=4BoIE8YQwM8&t=3s
*/
void updateAngles() {
  
  sensors_event_t accel, mag, gyro, temp;
  double accelX, accelY, accelZ;
  double gyroX, gyroY, gyroZ;
  double convertTimeRadians;
  double acc_mag_vector;
  double angle_pitch_acc, angle_roll_acc;
  double convertToDegrees;
  
  lsm.getEvent(&accel, &mag, &gyro, &temp);
 
  accelX = accel.acceleration.x - calAccX;
  accelY = accel.acceleration.y - calAccY;
  accelZ = accel.acceleration.z;
  
  gyroX = gyro.gyro.x - calGyroX;
  gyroY = gyro.gyro.y - calGyroY;
  gyroZ = gyro.gyro.z - calGyroZ;

  // Gyro Integration
  prev_time = cur_time;
  cur_time = millis();
  dt = (cur_time - prev_time) / (1000.0);
  angle_pitch += (gyroX*dt); 
  angle_roll += gyroY*dt; 

  // Adjust for yaw rotation, look at video at top of function
  convertTimeRadians = dt * PI/ 180.0;
  angle_pitch += angle_roll * sin(gyroZ * convertTimeRadians);               //If the IMU has yawed transfer the roll angle to the pitch angel
  angle_roll -= angle_pitch * sin(gyroZ * convertTimeRadians);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  // Accelerometer angle calculations
  acc_mag_vector = sqrt((accelX*accelX)+(accelY*accelY)+(accelZ*accelZ));  //Calculate the total accelerometer vector
  convertToDegrees = 1 / (PI / 180.0);
  angle_pitch_acc = asin((double)accelY/acc_mag_vector)* convertToDegrees;       //Calculate the pitch angle
  angle_roll_acc = asin((double)accelX/acc_mag_vector)* -convertToDegrees;       //Calculate the roll angle

  // Complimentary Filter
  angle_pitch = angle_pitch * filterConstant + angle_pitch_acc * (1.0-filterConstant);     //Correct the drift of the gyro pitch angle with the accelerometer pitch angle
  angle_roll = angle_roll * filterConstant + angle_roll_acc * -(1.0-filterConstant);        //Correct the drift of the gyro roll angle with the accelerometer roll angle

}

/* This function calculates PID values for pitch and roll.*/
void calculatePID() {
  double delta_error_pitch = 0, delta_error_roll = 0;
  double error_angle_pitch, error_angle_roll;
  double pid_p_pitch, pid_p_roll, pid_d_pitch, pid_d_roll;

  // Calculate Error
  error_angle_pitch =  (desired_angle_pitch) - (angle_pitch);    
  error_angle_roll =  (desired_angle_roll) - (angle_roll) ;     

  // Calculate Net Error from Previous Measurement
  delta_error_pitch = error_angle_pitch - previous_error_pitch;   // This is so that we can find the difference in error when we're finding the derivative of error.
  delta_error_roll = error_angle_roll - previous_error_roll;

  // Calculate P
  pid_p_pitch = kp * error_angle_pitch;
  pid_p_roll = kp * error_angle_roll;

  /* In case Integral term is desired in future
  // Calculate I
  if (angle_roll < integration_cuttoff_angle  || angle_roll > -integration_cuttoff_angle ) {
    pid_i_roll = .75*pid_i_roll + ki*(error_angle_roll*cur_time);
  }
  else {
    pid_i_roll = 0;
  }
  if (angle_pitch < integration_cuttoff_angle || angle_pitch >  -integration_cuttoff_angle) {
    pid_i_pitch = .75*pid_i_pitch + ki*(error_angle_pitch*cur_time);
  }
  else {
  }
 */

  // Calculate D
  pid_d_pitch = kd*(error_angle_pitch - previous_error_pitch) / (dt);    // Finding derivate of error
  pid_d_roll = kd*(error_angle_roll - previous_error_roll) / (dt);

  // Add P, I, D terms together
  PID_pitch = (pid_p_pitch) + (pid_i_pitch) + (pid_d_pitch);
  PID_roll = (pid_p_roll) + (pid_i_roll) + (pid_d_roll);

  // Set previous error to be current error
  previous_error_pitch = error_angle_pitch;
  previous_error_roll = error_angle_roll;
}

// Moves servos according to algorithm
void moveServos() {
  new_servo_angle_pitch = cur_servo_angle_pitch + PID_pitch; // Calculate the new_servo_angle by adding PID to the cur_servo_angle. Also, just to make sure I understand this, why are we adding PID to the servo angle? Won't we have to subtract in some cases?
  new_servo_angle_roll = cur_servo_angle_roll + PID_roll; // Calculate the new_servo_angle by adding PID to the cur_servo_angle. Also, just to make sure I understand this, why are we adding PID to the servo angle? Won't we have to subtract in some cases?
  

// First check if the new_servo_angle that we calculated is past 160 degrees.
  checkServoBounds();
  
  myservo_pitch.write(new_servo_angle_pitch); // adjusting position of servo by adding actual PID value to current servo angle.
  myservo_roll.write(new_servo_angle_roll); // adjusting position of servo by adding actual PID value to current servo angle.

  // Updates current angle
  cur_servo_angle_pitch = new_servo_angle_pitch;
  cur_servo_angle_roll = new_servo_angle_roll;
}

// Checks to see if the servo violates and upper or lower bounds set to prevent flaps hitting the side of the drone
void checkServoBounds() {

  // Checks both upper bounds
  if (  new_servo_angle_pitch > servoUpperBound ) {
    new_servo_angle_pitch = servoUpperBound; 
  }
  else {
    new_servo_angle_pitch = new_servo_angle_pitch;
  }

  if (  new_servo_angle_roll > servoUpperBound ) {
    new_servo_angle_roll = servoUpperBound; 
  }
  else {
    new_servo_angle_roll = new_servo_angle_roll;
  }

  // Checks both lower bounds
  if (  new_servo_angle_pitch < servoLowerBound ) {
    new_servo_angle_pitch = servoLowerBound; 
  }
  else {
    new_servo_angle_pitch = new_servo_angle_pitch;  
  }

  if (  new_servo_angle_roll < servoLowerBound ) {
    new_servo_angle_roll = servoLowerBound; 
  }
  else {
    new_servo_angle_roll = new_servo_angle_roll;    
  }
}

/*Gets an altitude reading*/
void Altitude() {
  if (! baro.begin()) {
    Serial.println("Couldnt find sensor");
    return;
  }
  
  float pascals = baro.getPressure();
  // Our weather page presents pressure in Inches (Hg)
  // Use http://www.onlineconversion.com/pressure.htm for other units
  Serial.print(pascals/3377); Serial.println(" Inches (Hg)");

  float altm = baro.getAltitude();
  Serial.print(altm); Serial.println(" meters");


  // This sensor also reads the temperature. We don't need to read the temperature.
  /*
  float tempC = baro.getTemperature();
  Serial.print(tempC); Serial.println("*C");
  */

  delay(250); // This delay was given in the example code of the altitude sensor. We can take this out. 
  // But, for testing purposes, we'll leave it in just to see if the sensor is working properly. 
  // It may need calibration/tuning just like the LSM9DS0 sensor.
}




/* We want to keep the drone in between a specific boundary of altitude.*/
void checkAltitude() {

/*
 We don't want the drone to exceed an altitude of 1 meter. 
 To keep it under 1 meter altitude, the "val" variable (the propeller's speed), needs to be under a specific value. 
 The faster the propeller is spinning, the drone will be at a higher atltiude.
 The slower the propeller is spinning, the drone will be at a lower altitude.
*/

if ( altm > 1) // If altitude > 1 meter => Make the motor speed slower so that the drone drops in altitude.
{

  // Make the val sent to the motor slower so that the drone drops in altitude.
  val = 1300;
}

else 
{
  motorSpeed = motorSpeed;
}


void printResults() {
  //Serial.print("Pitch Angle: ");
  Serial.print(angle_roll);
  Serial.print(", ");
  
 // Serial.print("Previous error: ");
  Serial.print(previous_error_roll);
  Serial.print(", ");

 // Serial.print("New Servo Angle: ");
  Serial.println(new_servo_angle_roll);

}


void setup() {
  boolean goodCalibration = false;
  myservo_roll.attach(11);  // attaches the servo on pin 5 to the servo object
  esc.attach(8); // attaches the servo to pin 8 of the esc
  esc.writeMicroseconds(1000); // turn on motor to zero power
    
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

  // Do initial servo adjustments
  myservo_roll.write(initial_pos);
  myservo_roll.write(initial_pos + 10);
  delay(500);
  myservo_roll.write(initial_pos);
 // myservo_roll.write(initial_pos);
  delay(3000);
  cur_time = millis();
}

void loop() {
  
  turnOnMotor(1800); // speed range of between 1000-2100, 1000 won't turn on, 2100 is max speed.
  Altitude(); // Call Altitude Sensor to start setup and start measuring altitude
  updateAngles();  // Calculate the pitch and roll angles from LSM9DS0 sensor (gyro+accelerometer built in).
  calculatePID();  // Calculate the total PID value based on the current and desired pitch/roll angles.
  moveServos();    // Based on the PID value, make decision to move servos (or not).
  printResults();  // Print out final results.
  checkAltitude(); // Keep checking altitude to make sure drone does not exceed > 1 meter.
  
}

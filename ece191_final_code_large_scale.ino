#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#include <servo.h>//Using servo library to control ESC
#include <servo.h>//Using servo library to control ESC
#include <Adafruit_Sensor.h>
#include <Adafruit_MPL3115A2.h> // Altitude sensor library
#include <Adafruit_LSM9DS0.h>   // Gyro and Accelerometer librar


Servo myservo_pitch;
Servo myservo_roll;
Servo esc;

Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2(); // Altitude Sensor, Uses I2C - connect SCL to the SCL pin, SDA to SDA pin
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // LSM9DS0 sensor, Use I2C, ID #1000
double calAccX = 0, calAccY = 0;
double calGyroX = 0, calGyroY = 0, calGyroZ = 0;
double angle_pitch = 0, angle_roll = 0, angle_yaw = 0;
double prev_time = 0;
double dt;
double cur_time = 0;
double filterConstant = .98;

double PID_pitch = 0, PID_roll = 0;
double previous_error_pitch = 0, previous_error_roll = 0;
double integration_cuttoff_angle = 1;

double kp = .03;  //3
double ki = 0; //.01
double kd = 0;   // 2

// Initial values or servos
double initial_pos = 90; // 180/2 for full range of motion
double desired_angle_pitch = 1, desired_angle_roll = 1; // to stay steady
int servoLowerBound = 40, servoUpperBound = 140;
double cur_servo_angle_pitch = initial_pos, cur_servo_angle_roll = initial_pos;
double new_servo_angle_pitch, new_servo_angle_roll; 

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
  double calibrationValue = .5;
  double gyroX, gyroY, gyroZ, accX, accY;
  
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
  accX = accel.acceleration.x - calAccX;
  accY = accel.acceleration.y - calAccY;  
  
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

void turnOnMotor(int motorSpeed) {
  esc.writeMicroseconds(motorSpeed);
}

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

}

void calculatePID() {
  double delta_error_pitch = 0, delta_error_roll = 0;
  double error_angle_pitch, error_angle_roll;
  double pid_p_pitch, pid_p_roll, pid_i_pitch, pid_i_roll, pid_d_pitch, pid_d_roll;
  
  error_angle_pitch =  (desired_angle_pitch) - (angle_pitch);   // The pitch angle will have a specific amount of error associated with it. 
  error_angle_roll =  (desired_angle_roll) - (angle_roll) ;     // The roll angle will have a specific amount of error associated with it.
  
  delta_error_pitch = error_angle_pitch - previous_error_pitch;   // This is so that we can find the difference in error when we're finding the derivative of error.
  delta_error_roll = error_angle_roll - previous_error_roll;

  pid_p_pitch = kp * error_angle_pitch;
  pid_p_roll = kp * error_angle_roll;

  /*
  if (angle_roll < integration_cuttoff_angle  || angle_roll > -integration_cuttoff_angle ) {
    pid_i_roll = ki + (error_angle_roll*cur_time);
  }
  else {
    pid_i_roll = 0;
  }
  if (angle_pitch < integration_cuttoff_angle || angle_pitch >  -integration_cuttoff_angle) {
    pid_i_pitch = ki + (error_angle_pitch*cur_time);
  }
  else {
    pid_i_pitch= 0;
  }*/
  pid_i_roll=0;
  pid_i_pitch= 0;

  pid_d_pitch = kd*(error_angle_pitch - previous_error_pitch) / (dt);    // Finding derivate of error
  pid_d_roll = kd*(error_angle_roll - previous_error_roll) / (dt);

  PID_pitch = (pid_p_pitch) + (pid_i_pitch) + (pid_d_pitch);
  
  PID_roll = (pid_p_roll) + (pid_i_roll) + (pid_d_roll);
  
  previous_error_pitch = error_angle_pitch;
  previous_error_roll = error_angle_roll;
}

void moveServos() {
 // cur_servo_angle_pitch = myservo_pitch.read();  // Read the current angle of the servo (the value passed to the last call to servo.write() ). The argument inside read() is going to be the signal pin for the servo motor.
  new_servo_angle_pitch = cur_servo_angle_pitch + PID_pitch; // Calculate the new_servo_angle by adding PID to the cur_servo_angle. Also, just to make sure I understand this, why are we adding PID to the servo angle? Won't we have to subtract in some cases?
//  cur_servo_angle_roll = myservo_roll.read();  // Read the current angle of the servo (the value passed to the last call to servo.write() ). The argument inside read() is going to be the signal pin for the servo motor.
  new_servo_angle_roll = cur_servo_angle_roll + PID_roll; // Calculate the new_servo_angle by adding PID to the cur_servo_angle. Also, just to make sure I understand this, why are we adding PID to the servo angle? Won't we have to subtract in some cases?
  

// First check if the new_servo_angle that we calculated is past 160 degrees.
  checkServoBounds();
  
//  myservo_pitch.write(new_servo_angle_pitch); // adjusting position of servo by adding actual PID value to current servo angle.
  myservo_roll.write(new_servo_angle_roll); // adjusting position of servo by adding actual PID value to current servo angle.
  cur_servo_angle_pitch = new_servo_angle_pitch;
  cur_servo_angle_roll = new_servo_angle_roll;
}

void checkServoBounds() {
  if (  new_servo_angle_pitch > servoUpperBound ) {
    new_servo_angle_pitch = servoUpperBound; //I'll only add half of the PID just for now because I think that adding the full PID value would make the new angle go past our threshold.  
  }
  else {
    new_servo_angle_pitch = new_servo_angle_pitch;    // Whatever new_servo_angle we just calculated, just make that the new_angle.
  }

  if (  new_servo_angle_roll > servoUpperBound ) {
    new_servo_angle_roll = servoUpperBound; //I'll only add half of the PID just for now because I think that adding the full PID value would make the new angle go past our threshold.  
  }
  else {
    new_servo_angle_roll = new_servo_angle_roll;    // Whatever new_servo_angle we just calculated, just make that the new_angle.
  }


  if (  new_servo_angle_pitch < servoLowerBound ) {
    new_servo_angle_pitch = servoLowerBound; //I'll only add half of the PID just for now because I think that adding the full PID value would make the new angle go past our threshold.  
  }
  else {
    new_servo_angle_pitch = new_servo_angle_pitch;    // Whatever new_servo_angle we just calculated, just make that the new_angle.
  }

  if (  new_servo_angle_roll < servoLowerBound ) {
    new_servo_angle_roll = servoLowerBound; //I'll only add half of the PID just for now because I think that adding the full PID value would make the new angle go past our threshold.  
  }
  else {
    new_servo_angle_roll = new_servo_angle_roll;    // Whatever new_servo_angle we just calculated, just make that the new_angle. 
  }
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





void checkAltitude() // We want to keep the drone in between a specific boundary of altitude.

{

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






void setup() {
  boolean goodCalibration = false;
   Serial.begin(9600);
  Serial.println("Adafruit_MPL3115A2 test!");
  myservo_roll.attach(11);  // attaches the servo on pin 5 to the servo object
  esc.attach(8); // attaches the servo to pin 8 of the esc
  esc.writeMicroseconds(1000);
    
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



}


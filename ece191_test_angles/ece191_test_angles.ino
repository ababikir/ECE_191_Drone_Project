#include <Servo.h>
#include <servo.h>//Using servo library to control ESC
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

Servo esc;
Servo myservo;


Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
double calAccX = 0, calAccY = 0;
double calGyroX = 0, calGyroY = 0, calGyroZ = 0;
double angle_pitch = 0, angle_roll = 0, angle_yaw = 0;
double prev_time = 0;
double dt;
double cur_time = 0;
double filterConstant = .98;
int i = 40;



/* Functions */
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

boolean calibrateSensor(int numData)
{
  sensors_event_t accel, mag, gyro, temp;
  int i;
  double calibrationValue = 2.0;
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
  esc.attach(8); //Specify the esc signal pin,Here as D8
  esc.writeMicroseconds(1000); //initialize the signal to 1000
  myservo.attach(11);
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
  
  int val;
  
  val = 1800;
  esc.writeMicroseconds(val); //using val as the signal to esc
  
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
  
  Serial.println(angle_roll, 6);
  //Serial.print(", ");
  //Serial.println(cur_time);
  //delay(50); // 50 ms delay
 
  if (i > 130) {
    i = 50; }
  i = i + 1;
  delay(100);
  myservo.write(i);
}

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
double calAccX = 0, calAccY = 0;
double calGyroX = 0, calGyroY = 0, calGyroZ = 0;
double angle_pitch = 0, angle_roll = 0, angle_yaw = 0;
double prev_time = 0;
double dt;


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

void calibrateSensor(int numData)
{
  int i;
  for (i = 0; i < numData; i += 1){
    sensors_event_t accel, mag, gyro, temp;
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

}


void setup() {
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

  calibrateSensor(2000);

}

void loop() {
  
  sensors_event_t accel, mag, gyro, temp;
  double accelX, accelY, accelZ;
  double gyroX, gyroY, gyroZ;
  double time_1;
  
  lsm.getEvent(&accel, &mag, &gyro, &temp);
  dt = (millis() - prev_time) / (1000.0);

 
  accelX = accel.acceleration.x - calAccX;
  accelY = accel.acceleration.y - calAccY;
  
  gyroX = gyro.gyro.x - calGyroX;
  gyroY = gyro.gyro.y - calGyroY;
  gyroZ = gyro.gyro.z - calGyroZ;
  //dt = (millis() - prev_time) / (1000.0);
  //dt = .01;
  // Gyro Integration
  prev_time = millis();
  angle_pitch = angle_pitch + (gyroX*dt); 
  angle_roll += gyroY*dt; 
  //prev_time = millis();
  // Adjust for yaw rotation
  //angle_pitch += angle_roll * sin(gyroZ * dt);               //If the IMU has yawed transfer the roll angle to the pitch angel
  //angle_roll -= angle_pitch * sin(gyroZ * 0.000001066);               //If the IMU has yawed transfer the pitch angle to the roll angel
  
  
  
  Serial.println(angle_pitch, 6);
  //delay(50); // 50 ms delay
    
}

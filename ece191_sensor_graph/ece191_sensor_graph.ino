#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000
float calAccX = 0, calAccY = 0, calAccZ = 0;
float calGyroX = 0, calGyroY = 0, calGyroZ = 0;

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
    calGyroX += gyro.gyro.x;
  }
  calGyroX /= numData;
  calAccX /= numData;
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

  calibrateSensor(1000);

}

void loop() {
  
  sensors_event_t accel, mag, gyro, temp;
  float accelX;
  float gyroX;

  
  lsm.getEvent(&accel, &mag, &gyro, &temp); 
  // Uncomment for acceleration measurement
  
  accelX = accel.acceleration.x - calAccX;
  Serial.println(accelX);
  

  // Uncomment for gryo measurement
  /*
  gyroX = gyro.gyro.x - calGyroX;
  Serial.println(gyroX);
  */
  delay(50); // 50 ms delay
  
}

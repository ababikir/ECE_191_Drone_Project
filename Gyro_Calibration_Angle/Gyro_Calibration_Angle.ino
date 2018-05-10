#include <SPI.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>

// Create Adafruit_LSM9DS0 object
Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);  // Use I2C, ID #1000

//Global variables for gyroscope data.
double anglez = 0;
double angley = 0; 
double anglex = 0;
double dt = 0;
double prev_time = 0;
double biasz = 0;
double biasx = 0; 
double biasy = 0;

//Configuring the gyroscope for a specific range
void configureIMU(void) {
    /* Initialise the sensor */
  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

  for(int i = 0; i < 1000; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    biasz += gyro.gyro.z;
    biasy += gyro.gyro.y;
    biasx += gyro.gyro.x;
  }

  biasz = biasz / 1000.;
  biasy = biasy / 1000.;
  biasx = biasx / 1000.;
}

//Reads gyroscope and returns current angle in radians in reference to initial angle to be 0 radians.
double gyroRead() {
    sensors_event_t accel, mag, gyro, temp;

    lsm.getEvent(&accel, &mag, &gyro, &temp);

    dt = (millis() - prev_time) / (1000.0);
    anglez = (anglez) + ( (gyro.gyro.z - biasz) * (dt) );
    angley = (angley) + ( (gyro.gyro.y - biasy) * (dt) );
    anglex = (anglex) + ( (gyro.gyro.x - biasx) * (dt) );

    
    prev_time = millis();

    //return (int(anglez)) * (PI / 180);
    //return (int(angley)) * (PI / 180);
    //return (int(anglex)) * (PI / 180);
    Serial.println(anglex);
}

void setup(void)
{
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
  configureIMU();
}

void loop(void)
{
  gyroRead();
}


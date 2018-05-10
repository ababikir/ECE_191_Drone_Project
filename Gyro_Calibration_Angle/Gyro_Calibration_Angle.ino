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
  if(!lsm.begin()) {
      while(1);
  }
  
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);

  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);

  for(int i = 0; i < 100; i++) {
    sensors_event_t accel, mag, gyro, temp;
    lsm.getEvent(&accel, &mag, &gyro, &temp);
    biasz += gyro.gyro.z;
    biasy += gyro.gyro.y;
    biasx += gyro.gyro.x;
  }

  biasz = biasz / 100.;
  biasy = biasy / 100.;
  biasx = biasx / 100.;
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

    return (int(anglez)) * (PI / 180);
    return (int(angley)) * (PI / 180);
    return (int(anglex)) * (PI / 180);


}

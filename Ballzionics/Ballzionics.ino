#include "quaternionFilters.h"
#include "MPU9250.h"

#define I2CClock 400000
#define I2CPort Wire
#define MPU9250_ADDRESS MPU9250_ADDRESS_AD0   // Use either this line or the next to select which I2C address your device is using
//#define MPU9250_ADDRESS MPU9250_ADDRESS_AD1

#define OPENLOG false

MPU9250 myIMU(MPU9250_ADDRESS, I2CPort, I2CClock);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial3.begin(9600);

  while(!Serial){};

  Serial3.println("Initializing IMU");
  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
  Serial.print(F("MPU9250 I AM 0x"));
  Serial.print(c, HEX);
  Serial.print(F(" I should be 0x"));
  Serial.println(0x71, HEX);

  if (c == 0x71) { // WHO_AM_I should always be 0x71
    Serial.println(F("MPU9250 is online..."));

    // Start by performing self test and reporting values
    myIMU.MPU9250SelfTest(myIMU.selfTest);
    Serial.print(F("x-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[0],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[1],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: acceleration trim within : "));
    Serial.print(myIMU.selfTest[2],1); Serial.println("% of factory value");
    Serial.print(F("x-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[3],1); Serial.println("% of factory value");
    Serial.print(F("y-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[4],1); Serial.println("% of factory value");
    Serial.print(F("z-axis self test: gyration trim within : "));
    Serial.print(myIMU.selfTest[5],1); Serial.println("% of factory value");

    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);

    myIMU.initMPU9250();
    // Initialize device for active mode read of acclerometer, gyroscope, and
    // temperature
    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of
    // communication
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM 0x");
    Serial.print(d, HEX);
    Serial.print(" I should be 0x");
    Serial.println(0x48, HEX);

    if (d != 0x48) {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....\n");  

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
    
    Serial.println("Magnetometer Sensitivity Adjustment Values:");
    Serial.print("X-Axis: ");
    Serial.print(myIMU.factoryMagCalibration[0], 2);
    Serial.print(", Y-Axis: ");
    Serial.print(myIMU.factoryMagCalibration[1], 2);
    Serial.print(", Z-Axis: ");
    Serial.println(myIMU.factoryMagCalibration[2], 2);

    Serial.println("\nAccelerometer Sensitivity Adjustment Values:");
    Serial.print("X-Axis: ");
    Serial.print(myIMU.accelBias[0], 2);
    Serial.print(", Y-Axis: ");
    Serial.print(myIMU.accelBias[1], 2);
    Serial.print(", Z-Axis: ");
    Serial.println(myIMU.accelBias[2], 2);

    Serial.println("\nGyroscope Sensitivity Adjustment Values:");
    Serial.print("X-Axis: ");
    Serial.print(myIMU.gyroBias[0], 2);
    Serial.print(", Y-Axis: ");
    Serial.print(myIMU.gyroBias[1], 2);
    Serial.print(", Z-Axis: ");
    Serial.println(myIMU.gyroBias[2], 2);
  } // if (c == 0x71)

  else {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication Failed - Stop
    Serial.println(F("Communication Failed"));
    Serial.flush();
    abort();
  }
  Serial.println("\nSetup Complete\n");

  Serial.println(" \t   Accelerometer\t\tGyroscope \t      Magnetometer");
  Serial.println("Time      x   \t y   \t z   \t  x    \t  y  \t   z   \t  x   \t  y   \t  z");
  Serial.println("==============================================================================");
}

void loop() {
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  
  if (myIMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    myIMU.readAccelData(myIMU.accelCount);  // Read the x/y/z adc values
    
    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    myIMU.ax = (float)myIMU.accelCount[0] * myIMU.aRes; // - myIMU.accelBias[0];
    myIMU.ay = (float)myIMU.accelCount[1] * myIMU.aRes; // - myIMU.accelBias[1];
    myIMU.az = (float)myIMU.accelCount[2] * myIMU.aRes; // - myIMU.accelBias[2];

    myIMU.readGyroData(myIMU.gyroCount);  // Read the x/y/z adc values

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    myIMU.gx = (float)myIMU.gyroCount[0] * myIMU.gRes;
    myIMU.gy = (float)myIMU.gyroCount[1] * myIMU.gRes;
    myIMU.gz = (float)myIMU.gyroCount[2] * myIMU.gRes;

    myIMU.readMagData(myIMU.magCount);  // Read the x/y/z adc values

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections
    // Get actual magnetometer value, this depends on scale being set
    myIMU.mx = (float)myIMU.magCount[0] * myIMU.mRes
               * myIMU.factoryMagCalibration[0] - myIMU.magBias[0];
    myIMU.my = (float)myIMU.magCount[1] * myIMU.mRes
               * myIMU.factoryMagCalibration[1] - myIMU.magBias[1];
    myIMU.mz = (float)myIMU.magCount[2] * myIMU.mRes
               * myIMU.factoryMagCalibration[2] - myIMU.magBias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  
  // Must be called before updating quaternions!
  myIMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // the magnetometer; the magnetometer z-axis (+ down) is opposite to z-axis
  // (+ up) of accelerometer and gyro! We have to make some allowance for this
  // orientationmismatch in feeding the output to the quaternion filter. For the
  // MPU-9250, we have chosen a magnetic rotation that keeps the sensor forward
  // along the x-axis just like in the LSM9DS0 sensor. This rotation can be
  // modified to allow any convenient orientation convention. This is ok by
  // aircraft orientation standards! Pass gyro rate as rad/s
  MahonyQuaternionUpdate(myIMU.ax, myIMU.ay, myIMU.az, myIMU.gx * DEG_TO_RAD,
                         myIMU.gy * DEG_TO_RAD, myIMU.gz * DEG_TO_RAD, myIMU.my,
                         myIMU.mx, myIMU.mz, myIMU.deltat);

  
  myIMU.delt_t = millis() - myIMU.count;
  if (myIMU.delt_t > 50) {
    if (OPENLOG) {
      Serial3.print(float(millis())/1000); Serial3.print(",");
      Serial3.print(myIMU.ax); Serial3.print(",");
      Serial3.print(myIMU.ay); Serial3.print(",");
      Serial3.print(myIMU.az); Serial3.print(",");
      Serial3.print(myIMU.gx, 3); Serial3.print(",");
      Serial3.print(myIMU.gy, 3); Serial3.print(",");
      Serial3.print(myIMU.gz, 3); Serial3.print(",");
      Serial3.print(myIMU.mx); Serial3.print(",");
      Serial3.print(myIMU.my); Serial3.print(",");
      Serial3.print(myIMU.mz); Serial3.println("");
    }
      Serial.print(float(millis())/1000); Serial.print(",\t");
      Serial.print(myIMU.ax); Serial.print(",\t");
      Serial.print(myIMU.ay); Serial.print(",\t");
      Serial.print(myIMU.az); Serial.print(",\t");
      Serial.print(myIMU.gx, 3); Serial.print(",\t");
      Serial.print(myIMU.gy, 3); Serial.print(",\t");
      Serial.print(myIMU.gz, 3); Serial.print(",\t");
      Serial.print(myIMU.mx); Serial.print(",\t");
      Serial.print(myIMU.my); Serial.print(",\t");
      Serial.print(myIMU.mz); Serial.println("");
    
    myIMU.count = millis();
    //digitalWrite(myLed, !digitalRead(myLed));  // toggle led
  } // if (myIMU.delt_t > 500)
}

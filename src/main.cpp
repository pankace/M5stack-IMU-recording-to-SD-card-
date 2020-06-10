#include <M5Stack.h>                   // also this whichone do i use ahhhhh
#include <Wire.h>                      // this is for the imu
#include "utility/quaternionFilters.h" // this is for kalman filter
#include "utility/MPU9250.h"           // IMU that is beaing used
#define processing_out false
#define AHRS true        // Set to false for basic data read
#define SerialDebug true // Set to true to get Serial output for debugging
#define LCD              // THB im not really sure i would want this or do I?
//                       // I'm not including anything that uses the lcd or maybe i shoud ? actually i did does it work?
//                       // plz tell me i don't have a testing unit

MPU9250 IMU;
char msg[100];

//**********************************************************************************//
// sdcard start
void appendFile(fs::FS &fs, const char *path, const char *message)
{
  //Serial.printf("Appending to file: %s\r\n", path);
  File file = fs.open(path, FILE_APPEND);
  if (!file)
  {
    Serial.println("- failed to open file for appending");
    return;
  }
  if (file.print(message))
  {
    //Serial.println("- message appended");
  }
  else
  {
    Serial.println("- append failed");
  }
}
void writeFile(fs::FS &fs, const char *path, const char *message)
{
  Serial.printf("Writing file: %s\n", path);

  File file = fs.open(path, FILE_WRITE);
  if (!file)
  {
    Serial.println("Failed to open file for writing");
    return;
  }
  if (file.print(message))
  {
    Serial.println("File written");
  }
  else
  {
    Serial.println("Write failed");
  }
}
//**********************************************************************************//

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  M5.begin();
  M5.Lcd.setBrightness(200); // Screen thinggey to show that it is working
  M5.Lcd.fillScreen(GREEN);  // Will be green if on ? idk dont have one to test on

  delay(30000);

  //save files Cool Eh?
  writeFile(SD, "/acceleration.txt", "milligs,aX,aY,aZ");
  writeFile(SD, "/gyro.txt", "degree/sec,gX,gY,gZ");
  writeFile(SD, "/mag.txt", "millis,mX,mY,mZ");
  writeFile(SD, "/quaternion.txt", "millis,q0,qX,qY,qZ");
  writeFile(SD, "/ypr.txt", "rate/hz,Yaw,Pitch,Roll");
  writeFile(SD, "/diagnostics.txt", "cool diagnostics stuff delta must remain below 2% for results to be valid");
  // writeFile(SD, "/temperature.txt", "temperature");  //posible useless

  // if (c == 0x71) // WHO_AM_I should always be 0x68 what elsse so you think it would be huh?
  {
    Serial.println("MPU9250 is online...");
    // Start by performing self test and reporting values
    IMU.MPU9250SelfTest(IMU.SelfTest);
    Serial.print("x-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[0], 1);
    Serial.println("% of factory value");
    Serial.print("y-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[1], 1);
    Serial.println("% of factory value");
    Serial.print("z-axis self test: acceleration trim within : ");
    Serial.print(IMU.SelfTest[2], 1);
    Serial.println("% of factory value");
    Serial.print("x-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[3], 1);
    Serial.println("% of factory value");
    Serial.print("y-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[4], 1);
    Serial.println("% of factory value");
    Serial.print("z-axis self test: gyration trim within : ");
    Serial.print(IMU.SelfTest[5], 1);
    Serial.println("% of factory value");

    IMU.calibrateMPU9250(IMU.gyroBias, IMU.accelBias); // Calibrate gyro and accelerometers, load biases in bias registers
    IMU.initMPU9250();                                 // Initialize device for active mode read of acclerometer, gyroscope and temperature i put temperature in becasue i wanted

    Serial.println("MPU9250 initialized for active data mode....");

    // Read the WHO_AM_I register of the magnetometer, this is a good test of communication
    byte d = IMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);
    Serial.print("AK8963 ");
    Serial.print("I AM ");
    Serial.print(d, HEX);
    Serial.print(" I should be ");
    Serial.println(0x48, HEX);

    // Get magnetometer calibration from AK8963 ROM
    IMU.initAK8963(IMU.magCalibration);
    // Initialize device for active mode read of magnetometer
    Serial.println("AK8963 initialized for active data mode....");

    if (Serial)
    {
      //  Serial.println("Calibration values: ");
      Serial.print("X-Axis sensitivity adjustment value ");
      Serial.println(IMU.magCalibration[0], 2);
      Serial.print("Y-Axis sensitivity adjustment value ");
      Serial.println(IMU.magCalibration[1], 2);
      Serial.print("Z-Axis sensitivity adjustment value ");
      Serial.println(IMU.magCalibration[2], 2);
    }
  }
}

char chr;

void loop()
{
  //diagnostics lol
  snprintf(msg, 50, "\r\n%d,%lf,%lf,%lf,%lf",
           IMU.SelfTest, IMU.gyroBias, IMU.accelBias, IMU.magCalibration);
  appendFile(SD, "/diagnostics.txt", msg);

  // I think this is evrything that would be intresting to know from a clinostat
  // If intPin goes high, all data registers have new data
  // On interrupt, check if data ready interrupt
  if (IMU.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)
  {
    IMU.readAccelData(IMU.accelCount); // Read the x/y/z adc values
    IMU.getAres();

    // Now we'll calculate the accleration value into actual g's
    // This depends on scale being set
    IMU.ax = (float)IMU.accelCount[0] * IMU.aRes; // - accelBias[0];
    IMU.ay = (float)IMU.accelCount[1] * IMU.aRes; // - accelBias[1];
    IMU.az = (float)IMU.accelCount[2] * IMU.aRes; // - accelBias[2];

    IMU.readGyroData(IMU.gyroCount); // Read the x/y/z adc values
    IMU.getGres();

    // Calculate the gyro value into actual degrees per second
    // This depends on scale being set
    IMU.gx = (float)IMU.gyroCount[0] * IMU.gRes;
    IMU.gy = (float)IMU.gyroCount[1] * IMU.gRes;
    IMU.gz = (float)IMU.gyroCount[2] * IMU.gRes;

    IMU.readMagData(IMU.magCount); // Read the x/y/z adc values
    IMU.getMres();
    // User environmental x-axis correction in milliGauss, should be automatically calculated
    IMU.magbias[0] = +470.;
    // User environmental x-axis correction in milliGauss TODO axis??
    IMU.magbias[1] = +120.;
    // User environmental x-axis correction in milliGauss
    IMU.magbias[2] = +125.;

    // Calculate the magnetometer values in milliGauss
    // Include factory calibration per data sheet and user environmental
    // corrections Get actual magnetometer value, this depends on scale being set
    IMU.mx = (float)IMU.magCount[0] * IMU.mRes * IMU.magCalibration[0] -
             IMU.magbias[0];
    IMU.my = (float)IMU.magCount[1] * IMU.mRes * IMU.magCalibration[1] -
             IMU.magbias[1];
    IMU.mz = (float)IMU.magCount[2] * IMU.mRes * IMU.magCalibration[2] -
             IMU.magbias[2];
  } // if (readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01)

  // Must be called before updating quaternions! i hate this code!
  IMU.updateTime();

  // Sensors x (y)-axis of the accelerometer is aligned with the y (x)-axis of
  // MadgwickQuaternionUpdate(ax, ay, az, gx*PI/180.0f, gy*PI/180.0f, gz*PI/180.0f,  my,  mx, mz);
  MahonyQuaternionUpdate(IMU.ax, IMU.ay, IMU.az, IMU.gx * DEG_TO_RAD,
                         IMU.gy * DEG_TO_RAD, IMU.gz * DEG_TO_RAD, IMU.my,
                         IMU.mx, IMU.mz, IMU.deltat);

  if (!AHRS)
  {
    IMU.delt_t = millis() - IMU.count;
    if (IMU.delt_t > 500)
    {
      if (SerialDebug)
      {
        // Print acceleration values in milligs!
        Serial.print("X-acceleration: ");
        Serial.print(1000 * IMU.ax);
        Serial.print(" mg ");
        Serial.print("Y-acceleration: ");
        Serial.print(1000 * IMU.ay);
        Serial.print(" mg ");
        Serial.print("Z-acceleration: ");
        Serial.print(1000 * IMU.az);
        Serial.println(" mg ");

        // Print gyro values in degree/sec
        Serial.print("X-gyro rate: ");
        Serial.print(IMU.gx, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Y-gyro rate: ");
        Serial.print(IMU.gy, 3);
        Serial.print(" degrees/sec ");
        Serial.print("Z-gyro rate: ");
        Serial.print(IMU.gz, 3);
        Serial.println(" degrees/sec");

        // Print mag values in degree/sec
        Serial.print("X-mag field: ");
        Serial.print(IMU.mx);
        Serial.print(" mG ");
        Serial.print("Y-mag field: ");
        Serial.print(IMU.my);
        Serial.print(" mG ");
        Serial.print("Z-mag field: ");
        Serial.print(IMU.mz);
        Serial.println(" mG");

        IMU.tempCount = IMU.readTempData(); // Read the adc values
        // Temperature in degrees Centigrade

        IMU.temperature = ((float)IMU.tempCount) / 333.87 + 21.0;
        // Print temperature in degrees Centigrade
        Serial.print("Temperature is ");
        Serial.print(IMU.temperature, 1);
        Serial.println(" degrees C");
        Serial.println("");
      }

      IMU.count = millis();
      // digitalWrite(myLed, !digitalRead(myLed));  // toggle led
    } // if (IMU.delt_t > 500)
  }   // if (!AHRS)
  else
  {

    IMU.delt_t = millis() - IMU.count;

    // if (IMU.delt_t > 500) //use this if data logging is trash
    if (IMU.delt_t > 100)
    {
      if (SerialDebug)
      {
        Serial.print("ax = ");
        Serial.print((int)1000 * IMU.ax);
        Serial.print(" ay = ");
        Serial.print((int)1000 * IMU.ay);
        Serial.print(" az = ");
        Serial.print((int)1000 * IMU.az);
        Serial.println(" mg");
        //  writeFile(SD, "acceleration values in milligs", "millis,IMU.ax,IMU.ay,IMU.az");
        snprintf(msg, 50, "\r\n%d',%lf,%lf,%lf", millis, 1000 * IMU.ax, 1000 * IMU.ay, 1000 * IMU.az);
        appendFile(SD, "/acceleration.txt", msg);

        Serial.print("gx = ");
        Serial.print(IMU.gx, 2);
        Serial.print(" gy = ");
        Serial.print(IMU.gy, 2);
        Serial.print(" gz = ");
        Serial.print(IMU.gz, 2);
        Serial.println(" deg/s");
        //  writeFile(SD, "gyro values in degree/sec", "degree/sec,IMU.gx,IMU.gy,IMU.gz");
        snprintf(msg, 50, "\r\n%d,%lf,%lf,%lf", millis, IMU.gz, IMU.gy, IMU.gz);
        appendFile(SD, "/gyro.txt", msg);

        Serial.print("mx = ");
        Serial.print((int)IMU.mx);
        Serial.print(" my = ");
        Serial.print((int)IMU.my);
        Serial.print(" mz = ");
        Serial.print((int)IMU.mz);
        Serial.println(" mG");
        //  writeFile(SD, " mag values in d\millis", "degree/sec,IMU.mx,IMU.my,IMU.mz");
        snprintf(msg, 50, "\r\n%d,%lf,%lf,%lf", millis, (int)IMU.mz, (int)IMU.my, (int)IMU.mz);
        appendFile(SD, "/mag.txt", msg);

        Serial.print("q0 = ");
        Serial.print(*getQ());
        Serial.print(" qx = ");
        Serial.print(*(getQ() + 1));
        Serial.print(" qy = ");
        Serial.print(*(getQ() + 2));
        Serial.print(" qz = ");
        Serial.println(*(getQ() + 3));
        //  writeFile(SD, " quaternion values in millis", "degree/sec,IMU.mx,IMU.my,IMU.mz");
        snprintf(msg, 50, "\r\n%d,%lf,%lf,%lf,%lf", millis, *getQ(), *(getQ() + 1), *(getQ() + 2), *(getQ() + 3));
        appendFile(SD, "/quaternion.txt", msg);
      }

      // Define output variables from updated quaternion---these are Tait-Bryan angles.

      IMU.yaw = atan2(2.0f * (*(getQ() + 1) * *(getQ() + 2) + *getQ() *
                                                                  *(getQ() + 3)),
                      *getQ() * *getQ() + *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) - *(getQ() + 3) * *(getQ() + 3));
      IMU.pitch = -asin(2.0f * (*(getQ() + 1) * *(getQ() + 3) - *getQ() *
                                                                    *(getQ() + 2)));
      IMU.roll = atan2(2.0f * (*getQ() * *(getQ() + 1) + *(getQ() + 2) *
                                                             *(getQ() + 3)),
                       *getQ() * *getQ() - *(getQ() + 1) * *(getQ() + 1) - *(getQ() + 2) * *(getQ() + 2) + *(getQ() + 3) * *(getQ() + 3));
      IMU.pitch *= RAD_TO_DEG;
      IMU.yaw *= RAD_TO_DEG;
      // By declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      IMU.yaw -= 8.5;
      IMU.roll *= RAD_TO_DEG;

      if (SerialDebug)
      {
        Serial.print("Yaw, Pitch, Roll: ");
        Serial.print(IMU.yaw, 2);
        Serial.print(", ");
        Serial.print(IMU.pitch, 2);
        Serial.print(", ");
        Serial.println(IMU.roll, 2);

        Serial.print("rate = ");
        Serial.print((float)IMU.sumCount / IMU.sum, 2);
        Serial.println(" Hz");
        Serial.println("");
        //  writeFile(SD, "Yaw,Pitch,Roll in rate/hz", "hz,IMU.yaw,IMU.pitch,IMU.roll");
        snprintf(msg, 50, "\r\n%d,%lf,%lf,%lf,%lf", millis, (float)IMU.sumCount / IMU.sum, IMU.yaw, IMU.pitch, IMU.roll);
        appendFile(SD, "/ypr.txt", msg);
      }

      IMU.count = millis();
      IMU.sumCount = 0;
      IMU.sum = 0;

#if (processing_out)

      Serial.print(((IMU.yaw)));
      Serial.print(";");
      Serial.print(((IMU.pitch)));
      Serial.print(";");
      Serial.print(((IMU.roll)));
      Serial.print(";");
      Serial.print(26.5);
      Serial.print(";");
      Serial.print(0.01);
      Serial.print(";");
      Serial.print(0.02);
      Serial.println();
#endif

    } // if (IMU.delt_t > 500)
  }   // if (AHRS)
}

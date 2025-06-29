/****************************************************************
 * Example11_DMP_Bias_Save_Restore_ESP32.ino
 * ICM 20948 Arduino Library Demo
 * Initialize the DMP based on the TDK InvenSense ICM20948_eMD_nucleo_1.0 example-icm20948
 * Periodically save the biases in ESP32 EEPROM
 * Restore the biases at startup - if valid
 * Paul Clark, November 14th, 2022
 * Based on original code by the SlimeVR-Tracker contributors:
 * https://github.com/SlimeVR/SlimeVR-Tracker-ESP/blob/83550d21ef748678704b3a11c0f7afb6f44258e4/src/sensors/icm20948sensor.cpp#L30
 * https://github.com/PaulZC/SlimeVR-Tracker-ESP/blob/83550d21ef748678704b3a11c0f7afb6f44258e4/src/sensors/icm20948sensor.cpp#L40-L41
 * 
 * ** This example is based on InvenSense's _confidential_ Application Note "Programming Sequence for DMP Hardware Functions".
 * ** We are grateful to InvenSense for sharing this with us.
 * 
 * ** Important note: by default the DMP functionality is disabled in the library
 * ** as the DMP firmware takes up 14301 Bytes of program memory.
 * ** To use the DMP, you will need to:
 * ** Edit ICM_20948_C.h
 * ** Uncomment line 29: #define ICM_20948_USE_DMP
 * ** Save changes
 * ** If you are using Windows, you can find ICM_20948_C.h in:
 * ** Documents\Arduino\libraries\SparkFun_ICM-20948_ArduinoLibrary\src\util
 *
 * Please see License.md for the license information.
 *
 * Distributed as-is; no warranty is given.
 ***************************************************************/

#include "ICM_20948.h" // Click here to get the library: http://librarymanager/All#SparkFun_ICM_20948_IMU

//#define USE_SPI       // Uncomment this to use SPI

#define SERIAL_PORT Serial

#define SPI_PORT SPI // Your desired SPI port.       Used only when "USE_SPI" is defined
#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined

#define WIRE_PORT Wire // Your desired Wire port.      Used when "USE_SPI" is not defined
// The value of the last bit of the I2C address.
// On the SparkFun 9DoF IMU breakout the default is 1, and when the ADR jumper is closed the value becomes 0
#define AD0_VAL 1

#ifdef USE_SPI
ICM_20948_SPI myICM; // If using SPI create an ICM_20948_SPI object
#else
ICM_20948_I2C myICM; // Otherwise create an ICM_20948_I2C object
#endif

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

#include <EEPROM.h>

// Define a storage struct for the biases. Include a non-zero header and a simple checksum
struct biasStore
{
  int32_t header = 0x42;
  int32_t biasGyroX = 0;
  int32_t biasGyroY = 0;
  int32_t biasGyroZ = 0;
  int32_t biasAccelX = 0;
  int32_t biasAccelY = 0;
  int32_t biasAccelZ = 0;
  int32_t biasCPassX = 0;
  int32_t biasCPassY = 0;
  int32_t biasCPassZ = 0;
  int32_t sum = 0;
};

// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void updateBiasStoreSum(biasStore *store) // Update the bias store checksum
{
  int32_t sum = store->header;
  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;
  store->sum = sum;
}

bool isBiasStoreValid(biasStore *store) // Returns true if the header and checksum are valid
{
  int32_t sum = store->header;

  if (sum != 0x42)
    return false;

  sum += store->biasGyroX;
  sum += store->biasGyroY;
  sum += store->biasGyroZ;
  sum += store->biasAccelX;
  sum += store->biasAccelY;
  sum += store->biasAccelZ;
  sum += store->biasCPassX;
  sum += store->biasCPassY;
  sum += store->biasCPassZ;

  return (store->sum == sum);
}

void printBiases(biasStore *store)
{
  SERIAL_PORT.print(F("Gyro X: "));
  SERIAL_PORT.print(store->biasGyroX);
  SERIAL_PORT.print(F(" Gyro Y: "));
  SERIAL_PORT.print(store->biasGyroY);
  SERIAL_PORT.print(F(" Gyro Z: "));
  SERIAL_PORT.println(store->biasGyroZ);
  SERIAL_PORT.print(F("Accel X: "));
  SERIAL_PORT.print(store->biasAccelX);
  SERIAL_PORT.print(F(" Accel Y: "));
  SERIAL_PORT.print(store->biasAccelY);
  SERIAL_PORT.print(F(" Accel Z: "));
  SERIAL_PORT.println(store->biasAccelZ);
  SERIAL_PORT.print(F("CPass X: "));
  SERIAL_PORT.print(store->biasCPassX);
  SERIAL_PORT.print(F(" CPass Y: "));
  SERIAL_PORT.print(store->biasCPassY);
  SERIAL_PORT.print(F(" CPass Z: "));
  SERIAL_PORT.println(store->biasCPassZ);

}
// -=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

void setup()
{

  delay(1000);

  SERIAL_PORT.begin(115200); // Start the serial console
  SERIAL_PORT.println(F("ICM-20948 Example"));

#ifdef USE_SPI
  SPI_PORT.begin();
#else
  WIRE_PORT.begin();
  WIRE_PORT.setClock(400000);
#endif

  //myICM.enableDebugging(); // Uncomment this line to enable helpful debug messages on Serial

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
#ifdef USE_SPI
    myICM.begin(CS_PIN, SPI_PORT);
#else
    myICM.begin(WIRE_PORT, AD0_VAL);
#endif

    SERIAL_PORT.print(F("Initialization of the sensor returned: "));
    SERIAL_PORT.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      SERIAL_PORT.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  SERIAL_PORT.println(F("Device connected."));

  bool success = true; // Use success to show if the DMP configuration was successful

  // Initialize the DMP. initializeDMP is a weak function. In this example we overwrite it to change the sample rate (see below)
  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);

  // DMP sensor options are defined in ICM_20948_DMP.h
  //    INV_ICM20948_SENSOR_ACCELEROMETER               (16-bit accel)
  //    INV_ICM20948_SENSOR_GYROSCOPE                   (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_RAW_ACCELEROMETER           (16-bit accel)
  //    INV_ICM20948_SENSOR_RAW_GYROSCOPE               (16-bit gyro + 32-bit calibrated gyro)
  //    INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED (16-bit compass)
  //    INV_ICM20948_SENSOR_GYROSCOPE_UNCALIBRATED      (16-bit gyro)
  //    INV_ICM20948_SENSOR_STEP_DETECTOR               (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_STEP_COUNTER                (Pedometer Step Detector)
  //    INV_ICM20948_SENSOR_GAME_ROTATION_VECTOR        (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ROTATION_VECTOR             (32-bit 9-axis quaternion + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_ROTATION_VECTOR (32-bit Geomag RV + heading accuracy)
  //    INV_ICM20948_SENSOR_GEOMAGNETIC_FIELD           (32-bit calibrated compass)
  //    INV_ICM20948_SENSOR_GRAVITY                     (32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_LINEAR_ACCELERATION         (16-bit accel + 32-bit 6-axis quaternion)
  //    INV_ICM20948_SENSOR_ORIENTATION                 (32-bit 9-axis quaternion + heading accuracy)

  // Enable the DMP orientation sensor
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);

  // Enable any additional sensors / features
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);
  //success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED) == ICM_20948_Stat_Ok);

  // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  //success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Cpass_Calibr, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  // Check success
  if (success)
  {
    SERIAL_PORT.println(F("DMP enabled."));
  }
  else
  {
    SERIAL_PORT.println(F("Enable DMP failed!"));
    SERIAL_PORT.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }

  // Read existing biases from EEPROM
  if (!EEPROM.begin(128)) // Allocate 128 Bytes for EEPROM storage. ESP32 needs this.
  {
    SERIAL_PORT.println(F("EEPROM.begin failed! You will not be able to save the biases..."));
  }

  biasStore store;

  EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
  if (isBiasStoreValid(&store))
  {
    SERIAL_PORT.println(F("Bias data in EEPROM is valid. Restoring it..."));
    success &= (myICM.setBiasGyroX(store.biasGyroX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroY(store.biasGyroY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasGyroZ(store.biasGyroZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelX(store.biasAccelX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelY(store.biasAccelY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasAccelZ(store.biasAccelZ) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassX(store.biasCPassX) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassY(store.biasCPassY) == ICM_20948_Stat_Ok);
    success &= (myICM.setBiasCPassZ(store.biasCPassZ) == ICM_20948_Stat_Ok);

    if (success)
    {
      SERIAL_PORT.println(F("Biases restored."));
      printBiases(&store);
    }
    else
      SERIAL_PORT.println(F("Bias restore failed!"));
  }

  SERIAL_PORT.println(F("The biases will be saved in two minutes."));
  SERIAL_PORT.println(F("Before then:"));
  SERIAL_PORT.println(F("* Rotate the sensor around all three axes"));
  SERIAL_PORT.println(F("* Hold the sensor stationary in all six orientations for a few seconds"));
}

void loop()
{
  static unsigned long startTime = millis(); // Save the biases when the code has been running for two minutes
  static bool biasesStored = false;
  
  // Read any DMP data waiting in the FIFO
  // Note:
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFONoDataAvail if no data is available.
  //    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOIncompleteData if a frame was present but was incomplete
  //    readDMPdataFromFIFO will return ICM_20948_Stat_Ok if a valid frame was read.
  //    readDMPdataFromFIFO will return ICM_20948_Stat_FIFOMoreDataAvail if a valid frame was read _and_ the FIFO contains more (unread) data.
  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail)) // Was valid data available?
  {
    //SERIAL_PORT.print(F("Received data! Header: 0x")); // Print the header in HEX so we can see what data is arriving in the FIFO
    //if ( data.header < 0x1000) SERIAL_PORT.print( "0" ); // Pad the zeros
    //if ( data.header < 0x100) SERIAL_PORT.print( "0" );
    //if ( data.header < 0x10) SERIAL_PORT.print( "0" );
    //SERIAL_PORT.println( data.header, HEX );

    if ((data.header & DMP_header_bitmap_Quat9) > 0) // We have asked for orientation data so we should receive Quat9
    {
      // Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
      // In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
      // The quaternion data is scaled by 2^30.

      //SERIAL_PORT.printf("Quat9 data is: Q1:%ld Q2:%ld Q3:%ld Accuracy:%d\r\n", data.Quat9.Data.Q1, data.Quat9.Data.Q2, data.Quat9.Data.Q3, data.Quat9.Data.Accuracy);

      // Scale to +/- 1
      double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
      double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
      double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30

/*
      SERIAL_PORT.print(F("Q1:"));
      SERIAL_PORT.print(q1, 3);
      SERIAL_PORT.print(F(" Q2:"));
      SERIAL_PORT.print(q2, 3);
      SERIAL_PORT.print(F(" Q3:"));
      SERIAL_PORT.print(q3, 3);
      SERIAL_PORT.print(F(" Accuracy:"));
      SERIAL_PORT.println(data.Quat9.Data.Accuracy);
*/

      // The ICM 20948 chip has axes y-forward, x-right and Z-up - see Figure 12:
      // Orientation of Axes of Sensitivity and Polarity of Rotation
      // in DS-000189-ICM-20948-v1.6.pdf  These are the axes for gyro and accel and quat
      //
      // For conversion to roll, pitch and yaw for the equations below, the coordinate frame
      // must be in aircraft reference frame.
      //  
      // We use the Tait Bryan angles (in terms of flight dynamics):
      // ref: https://en.wikipedia.org/w/index.php?title=Conversion_between_quaternions_and_Euler_angles
      //
      // Heading – ψ : rotation about the Z-axis (+/- 180 deg.)
      // Pitch – θ : rotation about the new Y-axis (+/- 90 deg.)
      // Bank – ϕ : rotation about the new X-axis  (+/- 180 deg.)
      //
      // where the X-axis points forward (pin 1 on chip), Y-axis to the right and Z-axis downward.
      // In the conversion example above the rotation occurs in the order heading, pitch, bank. 
      // To get the roll, pitch and yaw equations to work properly we need to exchange the axes

      // Note when pitch approaches +/- 90 deg. the heading and bank become less meaningfull because the
      // device is pointing up/down. (Gimbal lock)

      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      double qw = q0; // See issue #145 - thank you @Gord1
      double qx = q2;
      double qy = q1;
      double qz = -q3;

      // roll (x-axis rotation)
      double t0 = +2.0 * (qw * qx + qy * qz);
      double t1 = +1.0 - 2.0 * (qx * qx + qy * qy);
      double roll = atan2(t0, t1) * 180.0 / PI;

      // pitch (y-axis rotation)
      double t2 = +2.0 * (qw * qy - qx * qz);
      t2 = t2 > 1.0 ? 1.0 : t2;
      t2 = t2 < -1.0 ? -1.0 : t2;
      double pitch = asin(t2) * 180.0 / PI;

      // yaw (z-axis rotation)
      double t3 = +2.0 * (qw * qz + qx * qy);
      double t4 = +1.0 - 2.0 * (qy * qy + qz * qz);
      double yaw = atan2(t3, t4) * 180.0 / PI;

      SERIAL_PORT.print(F("Roll: "));
      SERIAL_PORT.print(roll, 1);
      SERIAL_PORT.print(F("\tPitch: "));
      SERIAL_PORT.print(pitch, 1);
      SERIAL_PORT.print(F("\tYaw: "));
      SERIAL_PORT.println(yaw, 1);
    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    if (!biasesStored) // Should we store the biases?
    {
      if (millis() > (startTime + 120000)) // Is it time to store the biases?
      {
        SERIAL_PORT.println(F("\r\n\r\n\r\nSaving bias data..."));

        biasStore store;
          
        bool success = (myICM.getBiasGyroX(&store.biasGyroX) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasGyroY(&store.biasGyroY) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasGyroZ(&store.biasGyroZ) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasAccelX(&store.biasAccelX) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasAccelY(&store.biasAccelY) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasAccelZ(&store.biasAccelZ) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasCPassX(&store.biasCPassX) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasCPassY(&store.biasCPassY) == ICM_20948_Stat_Ok);
        success &= (myICM.getBiasCPassZ(&store.biasCPassZ) == ICM_20948_Stat_Ok);

        updateBiasStoreSum(&store);
      
        if (success)
        {
          biasesStored = true; // Only attempt this once
        
          EEPROM.put(0, store); // Write biases to EEPROM, starting at address 0
          EEPROM.commit(); // ESP32/SAMD/STM32 needs this
  
          EEPROM.get(0, store); // Read existing EEPROM, starting at address 0
          if (isBiasStoreValid(&store))
          {
            SERIAL_PORT.println(F("Biases stored."));
            printBiases(&store);
            SERIAL_PORT.println(F("\r\n\r\n\r\n"));
          }
          else
            SERIAL_PORT.println(F("Bias store failed!\r\n\r\n\r\n"));
        }
        else
        {
          SERIAL_PORT.println(F("Bias read failed!\r\n\r\n\r\n"));
        }
      }
    }
      
    delay(10);
  }
}

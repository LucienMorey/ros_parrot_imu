#include "ICM_20948.h"
#include "SPI.h"

#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined


ICM_20948_SPI myICM;

void setup() {
  Serial.begin(115200);

  SPI.begin();

  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(CS_PIN, SPI);

    Serial.print(F("Initialization of the sensor returned: "));
    Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  Serial.println(F("Device connected!"));

  bool success = true;

  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_RAW_ACCELEROMETER) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  

  
  // Enable the FIFO
  success &= (myICM.enableFIFO() == ICM_20948_Stat_Ok);

  // Enable the DMP
  success &= (myICM.enableDMP() == ICM_20948_Stat_Ok);

  // Reset DMP
  success &= (myICM.resetDMP() == ICM_20948_Stat_Ok);

  // Reset FIFO
  success &= (myICM.resetFIFO() == ICM_20948_Stat_Ok);

  if (success)
  {
    Serial.println(F("DMP enabled!"));
  }
  else
  {
    Serial.println(F("Enable DMP failed!"));
    Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }

}

void loop() {
  // put your main code here, to run repeatedly:

  icm_20948_DMP_data_t data;
  myICM.readDMPdataFromFIFO(&data);

  if ((myICM.status == ICM_20948_Stat_Ok) || (myICM.status == ICM_20948_Stat_FIFOMoreDataAvail))
  {
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
      double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));

      Serial.print(F("Q1:"));
      Serial.print(q1, 3);
      Serial.print(F(" Q2:"));
      Serial.print(q2, 3);
      Serial.print(F(" Q3:"));
      Serial.print(q3, 3);
      Serial.print(F(" Accuracy:"));
      Serial.println(data.Quat9.Data.Accuracy);

    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // We have asked for orientation data so we should receive Quat9
    {
      float accel_x = (float)data.Raw_Accel.Data.X;
      float accel_y = (float)data.Raw_Accel.Data.Y;
      float accel_z = (float)data.Raw_Accel.Data.Z;

      Serial.print(F("{\"accel_x\":"));
      Serial.print(accel_x, 3);
      Serial.print(F(", \"accel_y\":"));
      Serial.print(accel_y, 3);
      Serial.print(F(", \"accel_z\":"));
      Serial.print(accel_z, 3);
      Serial.println(F("}"));

    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // We have asked for orientation data so we should receive Quat9
    {
      float gyro_x = (float)data.Raw_Gyro.Data.X;
      float gyro_y = (float)data.Raw_Gyro.Data.Y;
      float gyro_z = (float)data.Raw_Gyro.Data.Z;

      Serial.print(F("{\"gyro_x\":"));
      Serial.print(gyro_x, 3);
      Serial.print(F(", \"gyro_y\":"));
      Serial.print(gyro_y, 3);
      Serial.print(F(", \"gyro_z\":"));
      Serial.print(gyro_z, 3);
      Serial.println(F("}"));

    }
  }

  if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  {
    delay(10);
  }

}

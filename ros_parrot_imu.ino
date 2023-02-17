#include <micro_ros_arduino.h>

#include "ICM_20948.h"
#include "SPI.h"
#include "constants.h"



#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>

rcl_publisher_t publisher;
sensor_msgs__msg__Imu msg;
// rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
// rcl_timer_t timer;

#define CS_PIN 2     // Which pin you connect CS to. Used only when "USE_SPI" is defined
#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}


ICM_20948_SPI myICM;

void setup() {
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH); 

  delay(2000);

  SPI.begin();

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "micro_ros_arduino_node_publisher"));



  bool initialized = false;
  while (!initialized)
  {

    // Initialize the ICM-20948
    // If the DMP is enabled, .begin performs a minimal startup. We need to configure the sample mode etc. manually.
    myICM.begin(CS_PIN, SPI);

    // Serial.print(F("Initialization of the sensor returned: "));
    // Serial.println(myICM.statusString());
    if (myICM.status != ICM_20948_Stat_Ok)
    {
      // Serial.println(F("Trying again..."));
      delay(500);
    }
    else
    {
      initialized = true;
    }
  }

  // Serial.println(F("Device connected!"));

  bool success = true;

  success &= (myICM.initializeDMP() == ICM_20948_Stat_Ok);
  
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ORIENTATION) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_GYROSCOPE) == ICM_20948_Stat_Ok);
  success &= (myICM.enableDMPSensor(INV_ICM20948_SENSOR_ACCELEROMETER) == ICM_20948_Stat_Ok);

    // Configuring DMP to output data at multiple ODRs:
  // DMP is capable of outputting multiple sensor data at different rates to FIFO.
  // Setting value can be calculated as follows:
  // Value = (DMP running rate / ODR ) - 1
  // E.g. For a 5Hz ODR rate when DMP is running at 55Hz, value = (55/5) - 1 = 10.
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Quat9, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Accel, 0) == ICM_20948_Stat_Ok); // Set to the maximum
  success &= (myICM.setDMPODRrate(DMP_ODR_Reg_Gyro, 0) == ICM_20948_Stat_Ok); // Set to the maximum

  ICM_20948_fss_t myFSS;
  myFSS.a = gpm8;
  myFSS.g = dps250;
  myICM.setFullScale((ICM_20948_Internal_Acc | ICM_20948_Internal_Gyr), myFSS);

  
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
    // Serial.println(F("DMP enabled!"));
  }
  else
  {
    // Serial.println(F("Enable DMP failed!"));
    // Serial.println(F("Please check that you have uncommented line 29 (#define ICM_20948_USE_DMP) in ICM_20948_C.h..."));
    while (1)
      ; // Do nothing more
  }

  memset(&msg, 0, sizeof(sensor_msgs__msg__Imu));

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
      msg.orientation.x = ((double)data.Quat9.Data.Q1) * ORIENTATION_CONVERSION_FACTORS::QUATERNION_9_DOF; // Convert to double. Divide by 2^30
      msg.orientation.y = ((double)data.Quat9.Data.Q2) * ORIENTATION_CONVERSION_FACTORS::QUATERNION_9_DOF; // Convert to double. Divide by 2^30
      msg.orientation.z = ((double)data.Quat9.Data.Q3) * ORIENTATION_CONVERSION_FACTORS::QUATERNION_9_DOF; // Convert to double. Divide by 2^30
      msg.orientation.w = sqrt(1.0 - ((msg.orientation.x * msg.orientation.x) + (msg.orientation.y * msg.orientation.y) + (msg.orientation.z * msg.orientation.z)));

      // Serial.print(F("Q1:"));
      // Serial.print(msg.orientation.w, 3);
      // Serial.print(F(" Q2:"));
      // Serial.print(msg.orientation.x, 3);
      // Serial.print(F(" Q3:"));
      // Serial.print(msg.orientation.y, 3);
      // Serial.print(F(" Accuracy:"));
      // Serial.println(data.Quat9.Data.Accuracy);

    }

    if ((data.header & DMP_header_bitmap_Accel) > 0) // We have asked for orientation data so we should receive Quat9
    {
      msg.linear_acceleration.x = (float)data.Raw_Accel.Data.X * ACCEL_CONVERSION_FACTORS::ACEEL_8G * GRAVITY;
      msg.linear_acceleration.y = (float)data.Raw_Accel.Data.Y * ACCEL_CONVERSION_FACTORS::ACEEL_8G * GRAVITY;
      msg.linear_acceleration.z = (float)data.Raw_Accel.Data.Z * ACCEL_CONVERSION_FACTORS::ACEEL_8G * GRAVITY;

      // Serial.print(F("{\"accel_x\":"));
      // Serial.print(msg.linear_acceleration.x, 3);
      // Serial.print(F(", \"accel_y\":"));
      // Serial.print(msg.linear_acceleration.y, 3);
      // Serial.print(F(", \"accel_z\":"));
      // Serial.print(msg.linear_acceleration.z, 3);
      // Serial.println(F("}"));

    }

    if ((data.header & DMP_header_bitmap_Gyro) > 0) // We have asked for orientation data so we should receive Quat9
    {
      msg.angular_velocity.x = (float)data.Raw_Gyro.Data.X * GYRO_CONVERSION_FACTORS::GYRO_250_DPS * PI/180.0;
      msg.angular_velocity.y = (float)data.Raw_Gyro.Data.Y * GYRO_CONVERSION_FACTORS::GYRO_250_DPS * PI/180.0;
      msg.angular_velocity.z = (float)data.Raw_Gyro.Data.Z * GYRO_CONVERSION_FACTORS::GYRO_250_DPS * PI/180.0;

      // Serial.print(F("{\"gyro_x\":"));
      // Serial.print(msg.angular_velocity.x, 3);
      // Serial.print(F(", \"gyro_y\":"));
      // Serial.print(msg.angular_velocity.y, 3);
      // Serial.print(F(", \"gyro_z\":"));
      // Serial.print(msg.angular_velocity.z, 3);
      // Serial.println(F("}"));

    }
  }

  // if (myICM.status != ICM_20948_Stat_FIFOMoreDataAvail) // If more data is available then we should read it right away - and not delay
  // {
  //   delay(10);
  // }
  int32_t temp = 0;
  myICM.getBiasAccelX(&temp);
  msg.linear_acceleration_covariance[0] = temp;
  myICM.getBiasAccelY(&temp);
  msg.linear_acceleration_covariance[1] = temp;
  myICM.getBiasAccelZ(&temp);
  msg.linear_acceleration_covariance[2] = temp;
  myICM.getBiasGyroX(&temp);
  msg.linear_acceleration_covariance[3] = temp;
  myICM.getBiasGyroY(&temp);
  msg.linear_acceleration_covariance[4] = temp;
  myICM.getBiasGyroZ(&temp);
  msg.linear_acceleration_covariance[5] = temp;

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

}


/*
 * Project IMU6DOF
 * Description: Obtain Accel and Gyro data from BNO055 with DS1307 rtc
 * Author: Annis Nusseibeh
 * Date: July 22, 2020
 */

#define DS1307_ADDRESS 0x68

#include <Particle.h>
#include "DS1307.h"
#include "HC_SR04.h"
#include "ADAFRUIT_BNO055_PHOTON.h"
#include "ros.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "sensor_msgs/Range.h"

// Date and time functions using a DS1307 RTC connected via I2C
//
// WIRE IT UP!
//
// DS1307               SPARK CORE
//--------------------------------------------------------------------
// VCC                - Vin (5V only, does not work on 3.3)
// Serial Clock (SCL) - D1 (needs 2.2k to 10k pull up resistor to Vin)
// Serial Data  (SDA) - D0 (needs 2.2k to 10k pull up resistor to Vin)
// Ground             - GND
//--------------------------------------------------------------------

RTC_DS1307 rtc;

#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
int counter = 0;

#define MAXCM 300.0
#define MINCM 2.0
float cm = 0;
int trigPin = D4;
int echoPin = D5;
int ISRPin = D6;

HC_SR04 rangeFinder = HC_SR04(trigPin, echoPin, MINCM, MAXCM);

Adafruit_BNO055 bno = Adafruit_BNO055(55);

uint32_t seconds = 0;
uint32_t nanoseconds = 0;
uint16_t DS3231_counter = 0;
bool pflag = FALSE;

// Define ROS stuff
ros::NodeHandle nh;
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField mag_msg;
geometry_msgs::Vector3Stamped cal_msg;
sensor_msgs::Range range_msg;

#define BAUDRATE 115200
#define LOOPRATE 100

ros::Publisher imu_publisher("bno055/imu", &imu_msg);
ros::Publisher cal_publisher("bno055/cal", &cal_msg);
ros::Publisher range_publisher("hcsr04/range", &range_msg);

// Variables used in loop
uint32_t loop_start_sec;
uint32_t loop_start_nsec;
DateTime now;
time_t now2;
uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
sensors_event_t event;
imu::Quaternion quat;
imu::Vector<3> gyro;
imu::Vector<3> accel;
uint32_t bno055_time_nsec;
uint32_t bno055_time_sec;
uint32_t hcsr04_time_nsec;
uint32_t hcsr04_time_sec;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  rtc.begin();
  
  // Set D6 interrupt pin
  pinMode(ISRPin, INPUT);
  attachInterrupt(ISRPin, DS3231ClockISR, RISING);

  // Set the rosserial baudrate
  nh.getHardware()->setBaud(BAUDRATE);

  // Setup the ros message publishers
  nh.initNode();
  nh.advertise(imu_publisher);
  nh.advertise(cal_publisher);
  nh.advertise(range_publisher);

  // Set covariance matrices to 0 (default is 0, so nothing needs to be done)
  // TODO: Figure out real covariance matrices from data sheet or Allan variance tests

  // Set range message sensor information
  range_msg.radiation_type = 0;
  range_msg.field_of_view = 15 * 3.14159265359 /  180;
  range_msg.min_range = MINCM / 100;
  range_msg.max_range = MAXCM / 100;

  if (Particle.connected()){
    Serial.println("Photon is connected to the cloud.");
    time_t lastSyncTimestamp;
    unsigned long cur = millis();
    Serial.print("Last sync was over a day ago at: ");
    Serial.println(Time.timeStr(lastSyncTimestamp)); 
    Serial.println("Attempting to update clock...");
    Particle.syncTime();
    waitUntil(Particle.syncTimeDone);
    if (Particle.timeSyncedLast() >= cur){
      Serial.print("Time updated to : ");
      Serial.println(Time.timeStr());
      Serial.println("Updating RTC time.");
      time_t set_rtc = Time.now();
      seconds = set_rtc;
      nanoseconds = 0;
      DS3231_counter = 0;
      rtc.adjust(DateTime(set_rtc));
    }
    else{
      Serial.println("Something went wrong. Could not sync time.");
    }
  }
  else{
    DateTime curr_rtc = rtc.now();
    seconds = curr_rtc.unixtime();
    nanoseconds = 0;
    DS3231_counter = 0;
    Time.setTime(curr_rtc.unixtime());
    Serial.print("Particle not connected to the cloud. Setting time from RTC: ");
    Serial.print(curr_rtc.year(), DEC);
    Serial.print('/');
    Serial.print(curr_rtc.month(), DEC);
    Serial.print('/');
    Serial.print(curr_rtc.day(), DEC);
    Serial.print(' ');
    Serial.print(curr_rtc.hour(), DEC);
    Serial.print(':');
    Serial.print(curr_rtc.minute(), DEC);
    Serial.print(':');
    Serial.print(curr_rtc.second(), DEC);
    Serial.println();
  }

  /* Initialize the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1) ;
  }

  bno.setExtCrystalUse(true);

  Serial.println();
}

void loop() {
  now = rtc.now();
  now2 = Time.now();
  loop_start_sec = seconds;
  loop_start_nsec = nanoseconds;

  /* Get a new sensor event */
  bno.getEvent(&event);

  /* Display calibration status for each sensor. */
  bno.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  cal_msg.header.stamp.sec = seconds;
  cal_msg.header.stamp.nsec = nanoseconds;

  cal_msg.vector.x = (float)gyro_cal;
  cal_msg.vector.y = (float)accel_cal;
  cal_msg.vector.z = (float)mag_cal;

  // Get IMU and mag data to publish to ROS
  quat = bno.getQuat();
  gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);

  // Fill IMU message
  bno055_time_nsec = nanoseconds;
  bno055_time_sec = seconds;
  imu_msg.header.stamp.sec = bno055_time_sec;
  imu_msg.header.stamp.nsec = bno055_time_nsec;
  imu_msg.orientation.x = quat.x();
  imu_msg.orientation.y = quat.y();
  imu_msg.orientation.z = quat.z();
  imu_msg.orientation.w = quat.w();
  imu_msg.angular_velocity.x = gyro.x();
  imu_msg.angular_velocity.y = gyro.y();
  imu_msg.angular_velocity.z = gyro.z();
  imu_msg.linear_acceleration.x = accel.x();
  imu_msg.linear_acceleration.y = accel.y();
  imu_msg.linear_acceleration.z = accel.z();

  // Get range data to publish to cloud and ROS
  // cm = rangeFinder.getDistanceCM();

  // hcsr04_time_nsec = nanoseconds;
  // hcsr04_time_sec = seconds;
  // range_msg.header.stamp.sec = hcsr04_time_nsec;
  // range_msg.header.stamp.nsec = hcsr04_time_sec;
  // range_msg.range = (cm == -1) ? cm : cm / 100;

  // Publish ROS messages at 10Hz, cloud messages at 0.2Hz
  imu_publisher.publish(&imu_msg);
  cal_publisher.publish(&cal_msg);
  // range_publisher.publish(&range_msg);

  nh.spinOnce();
  delay(10-((seconds-loop_start_sec)+(nanoseconds-loop_start_nsec)*1e-9)*1e3);
}

// DS3231 32KHz clock interrupt service routine
void DS3231ClockISR(){
  nanoseconds += 30517.11247081801119978;   //32.7685kHz
  if (nanoseconds >= 999999000){
    nanoseconds = 0;
    pflag = TRUE;
    seconds++;
  }
}
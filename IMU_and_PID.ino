/*
 By Sam Travers
 Hardware setup:
 MPU9250 Breakout --------- Arduino
 VDD ---------------------- 3.3V
 SDA ----------------------- A4
 SCL ----------------------- A5
 GND ---------------------- GND
 */

#include "quaternionFilters.h"
#include "MPU9250.h"


#include <Adafruit_GFX.h>

#define AHRS true         // Set to false for basic data read
#define SerialDebug true  // Set to true to get Serial output for debugging

// Pin definitions
int intPin = 12;  // These can be changed, 2 and 3 are the Arduinos ext int pins
int myLed  = 13;  // Set up pin 13 led for toggling
float pitch, roll, yaw, pitch_sum, roll_sum, pitch_adjustment = 0.0, roll_adjustment = 0.0 ;
float pitch_rate, yaw_rate, roll_rate;

//PID variables
float pid_p_gain_roll = 1;
float pid_i_gain_roll = 0.00;
float pid_d_gain_roll = 0;
int pid_max_roll = 400;

float pid_p_gain_pitch = 1;
float pid_i_gain_pitch = 0.00;
float pid_d_gain_pitch = 0;
int pid_max_pitch = 400;

float pid_p_gain_yaw = 1;
float pid_i_gain_yaw = 0.00;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint=0, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint=0, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint=0, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

//receiver variables
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;

//esc variables
int esc_1, esc_2, esc_3, esc_4;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long loop_timer;
int throttle = 1200;

int cal_int;

MPU9250 myIMU;

void setup()
{
  DDRD |= B11110000;                                                        //Configure digital poort 4, 5, 6 and 7 as output.
  Wire.begin();
  // TWBR = 12;  // 400 kbit/sec I2C speed
  Serial.begin(9600);

  // Read the WHO_AM_I register, this is a good test of communication
  byte c = myIMU.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);

  if (c == 0x73) // WHO_AM_I should always be 0x71
  {
    // Calibrate gyro and accelerometers, load biases in bias registers
    myIMU.calibrateMPU9250(myIMU.gyroBias, myIMU.accelBias);
    myIMU.initMPU9250();
   
    byte d = myIMU.readByte(AK8963_ADDRESS, WHO_AM_I_AK8963);

    if (d != 0xFF)
    {
      // Communication failed, stop here
      Serial.println(F("Communication failed, abort!"));
      Serial.flush();
      abort();
    }

    // Get magnetometer calibration from AK8963 ROM
    myIMU.initAK8963(myIMU.factoryMagCalibration);

    // Get sensor resolutions, only need to do this once
    myIMU.getAres();
    myIMU.getGres();
    myIMU.getMres();
  } // if (c == 0x71)
  else
  {
    Serial.print("Could not connect to MPU9250: 0x");
    Serial.println(c, HEX);

    // Communication failed, stop here
    Serial.println(F("Communication failed, abort!"));
    Serial.flush();
    abort();
  }

  // Level the IMU. The first stage of the following sequence
  // runs the MPU9250 (myIMU) for 30 entries. This initial run removes 
  // an exaderated peak in pitch and roll values found to occur when 
  // the MPU9250 if first turned on.  
  Serial.print("MAKE SURE THE DRONE IS LEVEL. CALIBRATION STAGE 1.");
  Serial.println(" ");
  delay(5000);
  for(int x = 0; x < 60; x++){
    values(pitch, roll, yaw);
    Serial.print("Pitch_initial: ");
    Serial.print(pitch);
    Serial.print("\tRoll_initial: ");
    Serial.println(roll);
  }
      
  // The second stage sums pitch and roll values together for 50 values. 
  Serial.print("CALIBRATION STAGE 2."); 
  Serial.println(" ");
  for(int x = 0; x < 50; x++){
    values(pitch, roll, yaw);
    pitch_sum += pitch;
    roll_sum += roll;
    Serial.print("Pitch_sum: ");
    Serial.print(pitch_sum);
    Serial.print("\tRoll_rum: ");
    Serial.println(roll_sum);
  }
     
  // The third and final stage produces an average adjustment required
  // to bring both pitch and roll values close to zero. WITHOUT this piece
  // of code, either pitch or roll may produce a skewed value such as this 
  // example: |  pitch: 0.00 roll: -3.45  | while the drone sits level.
  Serial.print("CALIBRATION STAGE 3.");
  Serial.println(" ");
  pitch_adjustment = pitch_sum/50;
  roll_adjustment = roll_sum/50;
  Serial.print("Pitch_adjustment: ");
  Serial.print(pitch_adjustment);
  Serial.print("\tRoll_adjusment: ");
  Serial.println(roll_adjustment);
  
  for (cal_int = 0; cal_int < 750 ; cal_int ++){  
    PORTD |= B11110000;                                                     //Set digital poort 4, 5, 6 and 7 high.
    delayMicroseconds(1000);                                                //Wait 1000us.
    PORTD &= B00001111;                                                     //Set digital poort 4, 5, 6 and 7 low.
    delay(3); 
  }
  loop_timer = micros();                                                    //Set the timer for the next loop.
}

void values(float& pitch, float& roll, float& yaw)
{
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

//rates
    roll_rate = myIMU.gx;
    pitch_rate = myIMU.gy;
    yaw_rate = myIMU.gz;

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
  if (!AHRS)
  {
    myIMU.delt_t = millis() - myIMU.count;
    
  } // if (!AHRS)
  else
  {
    // Serial print and/or display at 0.5 s rate independent of data rates
    myIMU.delt_t = millis() - myIMU.count;
   

  
// Define output variables from updated quaternion---these are Tait-Bryan
// angles, commonly used in aircraft orientation. In this coordinate system,
// the positive z-axis is down toward Earth. Yaw is the angle between Sensor
// x-axis and Earth magnetic North (or true North if corrected for local
// declination, looking down on the sensor positive yaw is counterclockwise.
// Pitch is angle between sensor x-axis and Earth ground plane, toward the
// Earth is positive, up toward the sky is negative. Roll is angle between
// sensor y-axis and Earth ground plane, y-axis up is positive roll. These
// arise from the definition of the homogeneous rotation matrix constructed
// from quaternions. Tait-Bryan angles as well as Euler angles are
// non-commutative; that is, the get the correct orientation the rotations
// must be applied in the correct order which for this configuration is yaw,
// pitch, and then roll.
// For more see
// http://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
// which has additional links.
      myIMU.yaw   = atan2(2.0f * (*(getQ()+1) * *(getQ()+2) + *getQ()
                    * *(getQ()+3)), *getQ() * *getQ() + *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) - *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch = -asin(2.0f * (*(getQ()+1) * *(getQ()+3) - *getQ()
                    * *(getQ()+2)));
      myIMU.roll  = atan2(2.0f * (*getQ() * *(getQ()+1) + *(getQ()+2)
                    * *(getQ()+3)), *getQ() * *getQ() - *(getQ()+1)
                    * *(getQ()+1) - *(getQ()+2) * *(getQ()+2) + *(getQ()+3)
                    * *(getQ()+3));
      myIMU.pitch *= RAD_TO_DEG;
      myIMU.yaw   *= RAD_TO_DEG;

      // Declination of SparkFun Electronics (40°05'26.6"N 105°11'05.9"W) is
      //   8° 30' E  ± 0° 21' (or 8.5°) on 2016-07-19
      // - http://www.ngdc.noaa.gov/geomag-web/#declination
      myIMU.yaw  -= 0.7;
      myIMU.roll *= RAD_TO_DEG;

      pitch = myIMU.pitch - pitch_adjustment;
      roll = myIMU.roll - roll_adjustment;
      yaw = myIMU.yaw;

      myIMU.count = millis();
      myIMU.sumCount = 0;
      myIMU.sum = 0;
    } // if (myIMU.delt_t > 500)
  } // void vales


void calculate_pid(){
  //Roll calculations
  pid_error_temp = roll_rate - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = pitch_rate - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = yaw_rate - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void loop(){
  values(pitch, roll, yaw);
  calculate_pid();
  Serial.print("\tPitch Rate:\t"); Serial.print(pitch_rate, 2); Serial.print("\tPID:\t"); Serial.print(pid_output_pitch, 2); Serial.print("\tRoll Rate:\t"); Serial.print(roll_rate, 2); Serial.print("\tPID:\t"); Serial.print(pid_output_roll, 2); Serial.print("\tYaw Rate:\t"); Serial.print(yaw_rate, 2); Serial.print("\tPID:\t"); Serial.println(pid_output_yaw, 2);
  //Serial.print("\tRoll:  "); Serial.print(roll, 2); Serial.print("\tRoll Rate:\t"); Serial.print(roll_rate, 2); Serial.print("\tPID:\t"); Serial.println(pid_output_roll, 2);
  //Serial.print("\tYaw: "); Serial.print(yaw, 2); Serial.print("\tYaw Rate: "); Serial.println(yaw_rate, 2);

  //if(receiver_input_channel_1 > 1508) pid_roll_setpoint = (receiver_input_channel_1 - 1508)/3.0;
  //else if(receiver_input_channel_1 < 1492) pid_roll_setpoint = (receiver_input_channel_1 - 1492)/3.0;
  esc_1 = throttle + pid_output_roll + (pid_output_pitch*2/3); //Calculate the pulse for esc 1 (front-left - CCW)
  esc_2 = throttle - pid_output_roll + (pid_output_pitch*2/3); //Calculate the pulse for esc 2 (front-right - CW)
  esc_3 = throttle - pid_output_roll - (pid_output_pitch*4/3); //Calculate the pulse for esc 3 (rear - CCW)
  esc_4 = pid_output_yaw; //Calculate the pulse for esc 4 (servo - CW)

  if (esc_1 < 1100)esc_1 = 1100;                                         //Keep the motors running.
  if (esc_2 < 1100)esc_2 = 1100;                                         //Keep the motors running.
  if (esc_3 < 1100)esc_3 = 1100;                                         //Keep the motors running.
  if (esc_4 < 1100)esc_4 = 1100;                                         //Keep the motors running.

  if(esc_1 > 1400)esc_1 = 1400;                                           //Limit the esc-1 pulse to 2000us.
  if(esc_2 > 1400)esc_2 = 1400;                                           //Limit the esc-2 pulse to 2000us.
  if(esc_3 > 1400)esc_3 = 1400;                                           //Limit the esc-3 pulse to 2000us.
  if(esc_4 > 1400)esc_4 = 1400;                                           //Limit the esc-4 pulse to 2000us.

  //The refresh rate is 250Hz. That means the esc's need there pulse every 4ms.
  while(micros() - loop_timer < 4000);                                      //We wait until 4000us are passed.
  loop_timer = micros();                                                    //Set the timer for the next loop.

  PORTD |= B11110000;                                                       //Set digital outputs 4,5,6 and 7 high.
  timer_channel_1 = esc_1 + loop_timer;                                     //Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_2 = esc_2 + loop_timer;                                     //Calculate the time of the faling edge of the esc-2 pulse.
  timer_channel_3 = esc_3 + loop_timer;                                     //Calculate the time of the faling edge of the esc-3 pulse.
  timer_channel_4 = esc_4 + loop_timer;                                     //Calculate the time of the faling edge of the esc-4 pulse.

  while(PORTD >= 16){                                                       //Stay in this loop until output 4,5,6 and 7 are low.
    esc_loop_timer = micros();                                              //Read the current time.
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11101111;                //Set digital output 4 to low if the time is expired.
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11011111;                //Set digital output 5 to low if the time is expired.
    if(timer_channel_3 <= esc_loop_timer)PORTD &= B10111111;                //Set digital output 6 to low if the time is expired.
    if(timer_channel_4 <= esc_loop_timer)PORTD &= B01111111;                //Set digital output 7 to low if the time is expired.
  }
}


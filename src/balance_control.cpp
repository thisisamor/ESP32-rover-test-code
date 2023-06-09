/*

    balance_control.cpp

    By: EE2 Group 5


*/

/*
  Useful resources:
    [https://randomnerdtutorials.com/esp32-mpu-6050-accelerometer-gyroscope-arduino/]


  Adafruit_Sensor.h Notes [https://learn.adafruit.com/using-the-adafruit-unified-sensor-driver/how-does-it-work]: 
  The "sensors_event_t" type is used to encapsulate a specific sensor reading, called an 'event', and contains a data from the sensor from a specific moment in time.
  The getEvent(sensors_event_t*) method will read a new set of values from you sensor (a sensor 'event'), convert them to the appropriate SI units and scale, and then assign the results to a specific sensors_event_t object. Since "sensor_event_t" objects use 36 bytes of memory each, they should be passed into function as a reference (&) to save memory.

*/

#include <Arduino.h>
#include "balance_control.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Wire.h>
#include <vector>

//---------------------------Function Definitions----------------------------

// FIXME: Calculation results can become very inaccurate when acceleration components used in calculation 
// are orthogonal to the gravity vector. Large inaccuracies are only present when a component is almost 
// perfectly orthogonal to the gravity vector. This may need to be considered when writing code using calculated pitch, roll, and yaw values.

// Returns the pitch in degrees. Here we define pitch as the rotation about the y-axis.
float getAccPitch(sensors_event_t& a){
  return atan2(-a.acceleration.x, a.acceleration.z)*RAD_TO_DEG;
}

// Returns the roll in degrees. Here we define roll as the rotation about the x-axis.
float getAccRoll(sensors_event_t& a){
  return atan2(a.acceleration.y, a.acceleration.z)*RAD_TO_DEG;
}

// Returns the yaw in degrees. Here we define yaw as the rotation about the z-axis.
float getAccYaw(sensors_event_t& a){
  return atan2(-a.acceleration.y, a.acceleration.x)*RAD_TO_DEG;
}

// Converts an angle value in degrees to its equivalent value between 180 and -180.
float normalizeAngle(float angle) {
  float remainder = fmod(angle, 360.0);
  // If the remainder is greater than 180, subtract 360 to get the equivalent negative value
  if (remainder > 180.0) {
    remainder -= 360.0;
  }
  // If the remainder is less than -180, add 360 to get the equivalent positive value
  if (remainder < -180.0) {
    remainder += 360.0;
  }
  return remainder;
}


//--------------------------- MPU 6050 Method Definitions------------------------------

MPU_6050::MPU_6050(float aCompFilterParam, Cartesian aAccAngles, Cartesian aGryoAngles, Cartesian aFilteredAngles, Cartesian aGyroCalValues, Cartesian aGyroAngleVelCal){
  mpu = Adafruit_MPU6050();
  accAngles = aAccAngles;
  gyroAngles = aGryoAngles;
  filteredAngles = aFilteredAngles;
  gyroCalValues = aGyroCalValues;
  gyroAngVelCal = aGyroAngleVelCal;
  compFilterParam = aCompFilterParam;
  previousTime = 0.0;
  currentTime = 0.0;
}

MPU_6050::MPU_6050(const MPU_6050& anMPU){
  *this = anMPU;
}

MPU_6050::~MPU_6050(){}

MPU_6050& MPU_6050::operator=(const MPU_6050& anMPU){
  mpu = Adafruit_MPU6050(anMPU.mpu);    // Calling the Adafruit_MPU6050 copy constructor.
  a = anMPU.a;
  g = anMPU.g;
  temp = anMPU.temp;
  accAngles = anMPU.accAngles;
  gyroAngles = anMPU.gyroAngles;
  filteredAngles = anMPU.filteredAngles;
  gyroCalValues = anMPU.gyroCalValues;
  gyroAngVelCal = anMPU.gyroAngVelCal;
  compFilterParam = anMPU.compFilterParam;
  previousTime = anMPU.previousTime;
  currentTime = anMPU.currentTime;
  return *this;
}


// Activates and configures the MPU 6050. Returns "false" if failed, otherwise "true".
// NOTE: Remember that inside member methods of a class, "this->" is implied when referencing other class members.
bool MPU_6050::activate() {
  // Activating the MPU 6050. 
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip.");
    return false;
  }
  Serial.println("MPU6050 Found.");

  // Configuring the accelerometer.
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);

  // Configuring the gyro.
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_44_HZ);

  return true;
}

// Calibrates the Gyroscope. Should be run when the MPU 6050 is in a stationary position.  Returns "false" if failed, otherwise "true".
bool MPU_6050::gyro_calibrate(){
  Serial.println("Calibrating gyro, place on level surface and do not move.");
  gyroCalValues = Cartesian{0, 0, 0};   // Reset the gyro calibration values to 0.

  // Take 3000 readings for each coordinate.
  for (int cal_int = 0; cal_int < 3000; cal_int ++){
    if(cal_int % 200 == 0) Serial.print(".");
    mpu.getEvent(&a, &g, &temp);     // Get new sensor event values without updating data members.
    gyroCalValues.x += g.gyro.x;
    gyroCalValues.y += g.gyro.y;
    gyroCalValues.z += g.gyro.z;
  }

  // Average the values
  gyroCalValues.x /= 3000;
  gyroCalValues.y /= 3000;
  gyroCalValues.z /= 3000;

  // The MPU should start running right after calibration, meaning that we now set our initial "currentTime".
  currentTime = micros();

  return true;
}

// Measures sensor event values and updates relevant data members. Returns "false" if failed, otherwise "true".
bool MPU_6050::readSensor(){
  mpu.getEvent(&a, &g, &temp);     // Get new sensor event values.
  updateData();
  return true;
}

// Updates relevant data members in the class with new values read from the sensor. Returns "false" if failed, otherwise "true".
bool MPU_6050::updateData(){
  // Updating time variables.
  previousTime = currentTime;
  currentTime = micros();    // micros() returns the time in microseconds since the current program started running.

  // Updating calibrated angular velocity values.
  gyroAngVelCal = calcAngVelCal();

  // Updating the accelerometer calculated angle values.
  accAngles = calcAccAngles();

  // Updating the gyroscope calculated angle values.
  gyroAngles = calcGyroAngles();

  // Updating the Complementary Filter calculated angle values.
  filteredAngles = calcFilteredAngles();

  return true;
}

// Calculates the Calibrated Gyroscope Angular Velocity values.
Cartesian MPU_6050::calcAngVelCal(){
  Cartesian adjustedGyro;
  // Define new gyroscope angular velocities in degrees/s with the calibration offset values subtracted.
  adjustedGyro.x = (g.gyro.x - gyroCalValues.x)*RAD_TO_DEG;
  adjustedGyro.y = (g.gyro.y - gyroCalValues.y)*RAD_TO_DEG;
  adjustedGyro.z = (g.gyro.z - gyroCalValues.z)*RAD_TO_DEG;

  return adjustedGyro;
}

// Calculates the orientation of the MPU 6050 using only accelerometer data.
Cartesian MPU_6050::calcAccAngles(){
  Cartesian angles;
  angles.x = getAccRoll(a);    // Roll is the rotation about the x-axis.
  angles.y = getAccPitch(a);   // Pitch is the rotation about the y-axis.
  angles.z = getAccYaw(a);     // Yaw is the rotation about the z-axis.
  return angles;
}

// Calculates the orientation of the MPU 6050 using only gyroscope data.
Cartesian MPU_6050::calcGyroAngles(){
  Cartesian angles = gyroAngles;    // Initialize with the current gyro estimated angles.
  float dt;
  Cartesian adjustedGyroVel = calcAngVelCal();

  // Calculating the change in time elapsed since the last measurement in seconds. This change in time is used for the integration approximation.
  dt = (currentTime - previousTime)/1000000.0;

  // Gyroscope estimated angle values. These are integrals over time and will tend to drift.
  angles.x += adjustedGyroVel.x*dt;     
  angles.y += adjustedGyroVel.y*dt;
  angles.z += adjustedGyroVel.z*dt;

  // Changing angles to be within -180 and 180 degrees.
  angles.x = normalizeAngle(angles.x);
  angles.y = normalizeAngle(angles.y);
  angles.z = normalizeAngle(angles.z);

  return angles;
}

// Calculates the orientation of the MPU 6050 using accelerometer and gyroscope data with a Complementary Filter.
Cartesian MPU_6050::calcFilteredAngles(){
  Cartesian angles;
  float dt;
  Cartesian adjustedGyroVel = calcAngVelCal();
  Cartesian accelAngles = calcAccAngles();

  // Calculating the change in time elapsed since the last measurement in seconds. This change in time is used for the integration approximation.
  dt = (currentTime - previousTime)/1000000.0;

  // Complementary filter. (NOTE: A complementary filter is a computationally inexpensive sensor fusion technique that consists of a low-pass and a high-pass filter).
  // In this case the low-pass and high-pass filter are modeled as scalar multiplications to different terms.
  // This way we can combine the gyro and accelerometer measurements to get more accurate pitch, roll, and yaw values.
  angles.x = (compFilterParam)*(filteredAngles.x + adjustedGyroVel.x*dt) + (1-compFilterParam)*(accelAngles.x);   // The first term in this equation represents the previous filtered angle plus the integral of angular velocity from the gyro. The second term is the estimated accelerometer angle.
  angles.y = (compFilterParam)*(filteredAngles.y + adjustedGyroVel.y*dt) + (1-compFilterParam)*(accelAngles.y);
  angles.z = (compFilterParam)*(filteredAngles.z + adjustedGyroVel.z*dt) + (1-compFilterParam)*(accelAngles.z);

  return angles;
}





// there are 4 types of motor control: 
// 1. forward
// 2. turn left
// 3. turn right
// 4. balance only (stop)

// necessary output parameters: 
// 1. motor revolution 

// possible input parameters: 
// it might be helpful if we can control the velocity 
// (eg: rover moves faster on known path, slower when exploring)

/*
float alpha = angle.x;
//pole placement

void controltorque (float xd, float alphad, float x, float alpha){
// Define the system matrices A_d, B_d, C_d, and K
float A_d[6][6] = {{a11, a12, a13, a14, a15, a16},
                   {a21, a22, a23, a24, a25, a26},
                   {a31, a32, a33, a34, a35, a36},
                   {a41, a42, a43, a44, a45, a46},
                   {a51, a52, a53, a54, a55, a56},
                   {a61, a62, a63, a64, a65, a66}};

float B_d[6][2] = {{b11, b12},
                   {b21, b22},
                   {b31, b32},
                   {b41, b42},
                   {b51, b52},
                   {b61, b62}};

float C_d[2][6] = {{1, 0, 0, 0, 0, 0},
                   {0, 0, 1, 0, 0, 0}};

float K[3][2] = {{k11, k12},
                 {k21, k22}};

//inv(C_d / (eye(6) - (A_d - B_d * K)) * B_d)
float F[3][2] = {{f11, f12},
                 {f21, f22}};

// Define xi, yd, and u as arrays
float xi[6] = {x, 0, alpha, 0, theta, 0}; //What coordinate system are we using to give the position x?
//float thetad = 0; //the angle where the rover is stable (to be changed)
float yd[2][1] = {xd, alphad};
float y[2][1] = {x, alpha};
float u, yi;

// Calculate u = -K * xi' + F * yd
for (int i = 0; i < 2; i++) {
  u[i][0] = 0;
  for (int j = 0; j < 6; j++) {
    u[i][0] -= K[i][j] * xi[0][j];
  }
  for (int j = 0; j < 2; j++) {
    u[i][0] += F[i][j] * yd[0][j];
  }
}

// Calculate y_xi = C_d*xi' - y;
for (int i = 0; i < 2; i++) {
  y_xi[0][i] = 0;
  for (int j = 0; j < 6; j++) {
    y_xi[0][i] += C_d[i][j] * xi[0][j];
  }
  y_xi[0][i] -= y[0][i];
}

// Calculate xi = A_d*xi + B_d*u + L*y_xi
float temp[1][6]; // Temporary storage for intermediate calculation
for (int i = 0; i < 6; i++) {
  temp[0][i] = 0;
  for (int j = 0; j < 6; j++) {
    temp[0][i] += A_d[i][j] * xi[0][j];
  }
  for (int j = 0; j < 2; j++) {
    temp[0][i] += B_d[i][j] * u[j][0];
    temp[0][i] += L[i][j] * y_xi[0][j];
  }
}
for (int i = 0; i < 6; i++) {
  xi[0][i] = temp[0][i];
}
return u;

}

void controldelay(float u){
  float stepperdelay = //link to the datasheet to give appropriate value;
}

//PID Controller
// Constants for inner loop PID controller
const float Kp_inner = ...;
const float Ki_inner = ...;
const float Kd_inner = ...;

// Constants for outer loop PID controller
const float Kp_outer = ...;
const float Ki_outer = ...;
const float Kd_outer = ...;

// Variables for inner loop PID controller
float setpoint_inner = 0.0;
float input_inner = 0.0;
float output_inner = 0.0;
float error_inner = 0.0;
float prev_error_inner = 0.0;
float integral_inner = 0.0;
float derivative_inner = 0.0;

// Variables for outer loop PID controller
float setpoint_outer = 0.0;
float input_outer = 0.0;
float output_outer = 0.0;
float error_outer = 0.0;
float prev_error_outer = 0.0;
float integral_outer = 0.0;
float derivative_outer = 0.0;

// Compute the output of the inner loop PID controller
void computeInnerLoopPID() {
  error_inner = setpoint_inner - input_inner;
  integral_inner += error_inner;
  derivative_inner = error_inner - prev_error_inner;
  output_inner = Kp_inner * error_inner + Ki_inner * integral_inner + Kd_inner * derivative_inner;
  prev_error_inner = error_inner;
}

// Compute the output of the outer loop PID controller
void computeOuterLoopPID() {
  error_outer = setpoint_outer - output_inner;
  integral_outer += error_outer;
  derivative_outer = error_outer - prev_error_outer;
  output_outer = Kp_outer * error_outer + Ki_outer * integral_outer + Kd_outer * derivative_outer;
  prev_error_outer = error_outer;
}

// Main loop
void pidcontroller() {
  // Read inputs for inner and outer loops
  // Update setpoints for inner and outer loops
  
  // Inner loop computation
  input_inner = ...; // Read input for inner loop
  computeInnerLoopPID();
  
  // Outer loop computation
  input_outer = output_inner; // Set input for outer loop as output of inner loop
  computeOuterLoopPID();
  
  // Apply output to the actuators or system
  
  // Wait for next iteration
}

*/


// ------------- Motor control ---------------------------

// stepsPerRevolution 200
// oneStepDegeree 1.8

float stepperdelay = 2000; //will use the controldelay function subsequently to determine the torque for control

// one revolution (2*pi*r = 175mm)
void motor_control_1revo(int STPL, int STPR, int DIRL, int DIRR) 
{
  // Set the spinning direction counterclockwise:
  digitalWrite(DIRL, LOW);
  digitalWrite(DIRR, HIGH);
    
  //Spin the stepper motor 5 revolutions fast:
  while (1){
    // These four lines result in 1 step:
    digitalWrite(STPL, HIGH);
    digitalWrite(STPR, HIGH);
    delayMicroseconds(stepperdelay);
    digitalWrite(STPL, LOW);
    digitalWrite(STPR, LOW);
    delayMicroseconds(stepperdelay);
  }
  delay(3000);
}

// n revolution
void motor_control(int STPL, int STPR, int DIRL, int DIRR, int n) 
{
  digitalWrite(DIRL, HIGH);
  digitalWrite(DIRR, LOW);

  for(int i=0; i<200*n; i++)
  {
    digitalWrite(STPL, HIGH);
    digitalWrite(STPR, HIGH);
    delayMicroseconds(stepperdelay);
    digitalWrite(STPL, LOW);
    digitalWrite(STPR, LOW);
    delayMicroseconds(stepperdelay);
  }
}

// for testing algorithm only
// please dont change this for now <33
// read commands
void motor_control_custom(int STPL, int STPR, int DIRL, int DIRR, std::vector<int> command, int steps)
{
  // command [0] = DIRL
  // command [1] = DIRR
  // command [2] = STPL
  // command [3] = STPR
  digitalWrite(DIRL, command[0]);
  digitalWrite(DIRR, command[1]);

  for(int i=0; i<steps; i++)
  {
    digitalWrite(STPL, command[2]);
    digitalWrite(STPR, command[3]);
    delayMicroseconds(stepperdelay);
    digitalWrite(STPL, LOW);
    digitalWrite(STPR, LOW);
    delayMicroseconds(stepperdelay);
  }
}

// perform a series of motor command
void motor_control_command_list (int STPL, int STPR, int DIRL, int DIRR, std::vector<std::vector<int>> command)
{
  // command [0] = DIRL
  // command [1] = DIRR
  // command [2] = STPL
  // command [3] = STPR
  // command [4] = steps
  int i = 0; 
  while ( i < command.size() )
  {
    std::vector<int> tmp = command[i]; 
    digitalWrite(DIRL, tmp[0]);
    digitalWrite(DIRR, tmp[1]);

    for(int i=0; i<tmp[4]; i++)
    {
      digitalWrite(STPL, tmp[2]);
      digitalWrite(STPR, tmp[3]);
      delayMicroseconds(stepperdelay);
      digitalWrite(STPL, LOW);
      digitalWrite(STPR, LOW);
      delayMicroseconds(stepperdelay);
    }
    i ++; 
  }
}


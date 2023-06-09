/*

    balance_control.h

    By: EE2 Group 5

*/

// NOTE: These headers should be used to ensure that if this header file is included multiple times
// in the program, double declaration will be avoided. This is done by only including one version of
// the code between "#ifndef" and "#endif". [https://stackoverflow.com/questions/1653958/why-are-ifndef-and-define-used-in-c-header-files]
#ifndef balance_control_h
#define balance_control_h

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <math.h>
#include <Wire.h>
#include <vector>



//-----------------------Function Declarations---------------------------

float getAccPitch(sensors_event_t& a);
float getAccRoll(sensors_event_t& a);
float getAccYaw(sensors_event_t& a);

float normalizeAngle(float angle);

void motor_control_1revo(int STPL, int STPR, int DIRL, int DIRR); 
void motor_control(int STPL, int STPR, int DIRL, int DIRR, int n); 
void motor_control_custom(int STPL, int STPR, int DIRL, int DIRR, std::vector<int> command, int steps); 
void motor_control_command_list (int STPL, int STPR, int DIRL, int DIRR, std::vector<std::vector<int>> command); 
//------------------------Struct/Class Declarations-----------------------------

struct Cartesian {
    float x;
    float y;
    float z;
};

class MPU_6050{
public:
    // Default constructor (Remember that you do not need a default argument for every data member in a class. Additionally, when using the default constructor, variables with default arguments can be omitted as parameters and their default value will be assumed.)
    // It is also important to note that if you want to initialize the object with different values for certain default arguments, you need to enter a value for each default argument preceding the one you want to specify in the constructor input parameters. Hence, put the variables you are most likely to specify at the beginning of you input parameters.
    MPU_6050(float aCompFilterParam = 0.98, Cartesian aAccAngles=Cartesian{0, 0, 0}, Cartesian aGryoAngles=Cartesian{0, 0, 0},
            Cartesian aFilteredAngles=Cartesian{0, 0, 0},  Cartesian aGyroCalValues=Cartesian{0, 0, 0},
            Cartesian aGyroAngleVelCal=Cartesian{0, 0, 0});
    MPU_6050(const MPU_6050& anMPU);        // Copy constructor
    ~MPU_6050();        // Destructor

    MPU_6050& operator=(const MPU_6050& anMPU);     // Assignment operator

    // Methods

    bool activate();
    bool gyro_calibrate();
    bool readSensor();
    bool updateData();
    Cartesian calcAngVelCal();
    Cartesian calcAccAngles();
    Cartesian calcGyroAngles();
    Cartesian calcFilteredAngles();

    // Data Members

    Adafruit_MPU6050 mpu;       // Adafruit MPU 6050 object.
    sensors_event_t a, g, temp;     // Accelerometer, gyroscope, and temperature sensor last measured sensor event values.
    Cartesian accAngles;       // Accelerometer calculated angle values.
    Cartesian gyroAngles;      // Gyroscope calculated angle values.
    Cartesian filteredAngles;  // Calculated angle values after the Complementary Filter.
    Cartesian gyroCalValues;   // Stores the x, y, and z gyro calibration values.
    Cartesian gyroAngVelCal;   // Stores the calibrated angular velocity values in degrees/s.
    float compFilterParam;          // Parameter that defines the LPF and HPF for the angle calculation Complementary Filter.
    unsigned long previousTime;     // Stores the previous time when a sensor event was read. Used for integration calculations.
    unsigned long currentTime;      // Stores the current time when a sensor event was read. Used for itegration calculations.
};

#endif
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>
#include <Encoder.h>
#define CONTROL_INTERVAL_MS 50L
#define PUB_PERIOD 50L
//Declare PID variables
extern "C"
{
#include <PID.h>
}
#define PID_KP_LEFT 3.0f
#define PID_KI_LEFT 2.0f
#define PID_KD_LEFT 0.0f
#define PID_KP_RIGHT 3.0f
#define PID_KI_RIGHT 2.0f
#define PID_KD_RIGHT 0.0f
#define PID_TAU 0.05f
#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX 255.0f
#define PID_LIM_MIN_INT -125.0f
#define PID_LIM_MAX_INT 125.0f
#define SAMPLE_TIME_S 0.05f
#include <ros.h>
#include <math.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

//Include IMU related library
#include <I2Cdev.h>
#include <MPU6050.h>
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include <Wire.h>
#endif

//Define Encoders pin (need to be interrupt pins)
#define ENCA1 2
#define ENCA2 3
#define ENCB1 17
#define ENCB2 18
//Define ticks per revolution for converting ticks to rev
#define TICKS_PER_REV 1600
//Declare pin for motors
#define PWM_L 8
#define INA_L 12
#define INB_L 13
#define PWM_R 5
#define INA_R 6
#define INB_R 7
//Define Wheel Constant
#define WHEEL_NUM 2
#define WHEEL_RAD 0.125
#define WHEEL_DIST 0.547
Encoder encLeft(ENCA1,ENCA2), encRight(ENCB1,ENCB2);
PIDController pidLeft = {PID_KP_LEFT,
                     PID_KI_LEFT, PID_KD_LEFT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
PIDController pidRight = {PID_KP_RIGHT,
                     PID_KI_RIGHT, PID_KD_RIGHT,
                     PID_TAU,
                     PID_LIM_MIN, PID_LIM_MAX,
                     PID_LIM_MIN_INT, PID_LIM_MAX_INT,
                     SAMPLE_TIME_S};
//Time variables
unsigned long prevT = 0;
unsigned long currT = 0;
unsigned long prevT_state = 0;
unsigned long currT_state = 0;

//IMU variables
MPU6050 accelgyro;
int16_t ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001

#define ACCEL_SCALE 1 / 16384 // LSB/g
#define GYRO_SCALE 1 / 131 // LSB/(deg/s)
// #define MAG_SCALE 0.3 // uT/LSB


//Variables
int32_t posPrev[WHEEL_NUM] = {0, 0};
int32_t pos[WHEEL_NUM] = {0, 0};

int deltaPos[WHEEL_NUM] = {0, 0};
float rad[WHEEL_NUM] = {0.0, 0.0};
float velocity[WHEEL_NUM] = {0.0, 0.0};
float goal[WHEEL_NUM] = {0.0, 0.0};
int pwr[WHEEL_NUM] = {0, 0};
float odom_pose[3] = {0.0, 0.0, 0.0};
float last_theta = 0.0;
float callback_time = 0.0;
float prevT_imu = 0.0;

// ROS publishers and subscribers
ros::NodeHandle nh;
std_msgs::Float32MultiArray raw_vel;
ros::Publisher raw_vel_pub("/walkie/raw_vel", &raw_vel);
std_msgs::Int32MultiArray raw_pos;
ros::Publisher raw_pos_pub("/walkie/raw_pos", &raw_pos);

geometry_msgs::Vector3 raw_acc;
ros::Publisher raw_acc_pub("/walkie/imu/raw_linear_acc", &raw_acc);
geometry_msgs::Vector3 raw_gyr;
ros::Publisher raw_gyr_pub("/walkie/imu/raw_ang_vel", &raw_gyr);
// geometry_msgs::Vector3 raw_mag;
// ros::Publisher raw_mag_pub("/walkie/imu/raw_mag", &raw_mag);


//Declare Function
void GoalCb(const geometry_msgs::Twist&);

//Subscribe cmd_vel
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("/walkie/cmd_vel", GoalCb );
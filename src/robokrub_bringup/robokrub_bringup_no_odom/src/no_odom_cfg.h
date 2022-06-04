#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Arduino.h>

// //Setting the timer used for interrupts
// #define USE_TIMER_1     false
// #define USE_TIMER_2     true
// #define USE_TIMER_3     false
// #define USE_TIMER_4     false
// #define USE_TIMER_5     false

#define CONTROL_INTERVAL_MS 50L
#define PUB_PERIOD 50L

//Declare PID variables
extern "C"
{
#include <PID.h>
}

#define PID_KP_LEFT 0.4f
#define PID_KI_LEFT 1.0f
#define PID_KD_LEFT 0.0f

#define PID_KP_RIGHT 0.2f
#define PID_KI_RIGHT 1.2f
#define PID_KD_RIGHT 0.01f

#define PID_TAU 0.05f

#define PID_LIM_MIN -255.0f
#define PID_LIM_MAX 255.0f

#define PID_LIM_MIN_INT -125.0f
#define PID_LIM_MAX_INT 125.0f

#define SAMPLE_TIME_S 0.05f

//Import encoder library and servo for motor control
#include <Encoder.h>
#include <Servo.h>

//Import ros-related libraries
#include <ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
// #include <SoftwareSerial.h>
#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

//Define Wheel Constant
#define WHEEL_NUM 2
#define WHEEL_RAD 0.125
#define WHEEL_DIST 0.547

//Define Encoders pin (need to be interrupt pins)
#define ENCA1 2 
#define ENCA2 3

#define ENCB1 18
#define ENCB2 19

//Define ticks per revolution for converting ticks to rev
#define TICKS_PER_REV 1600

//Declare pin for motors
#define PWM_L 9
#define INA_L 10
#define INB_L 11

#define PWM_R 5
#define INA_R 6
#define INB_R 7

//Define pin of the servos
#define PITCH_PIN 9
#define YAW_PIN 10

//Limit of servo
#define YAW_UPPER_LIMIT 180
#define YAW_LOWER_LIMIT 0
#define PITCH_UPPER_LIMIT 180
#define PITCH_LOWER_LIMIT 0
#define YAW_INIT_STATE 10
#define PITCH_INIT_STATE 150



//Declare Servo objects for each wheels
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

//Declare variables for odom calculation
double Vx = 0 ;
double Vy = 0 ;
double W = 0 ;
double x = 0.0;
double y = 0.0;
double th = 0.0;

int pos[WHEEL_NUM] = {0, 0}; 
int prevPosOdom[WHEEL_NUM] = {0, 0};
int deltaPos[WHEEL_NUM] = {0, 0};
float rad[WHEEL_NUM] = {0.0, 0.0}; 

float velocity[WHEEL_NUM] = {0.0, 0.0};
float goal[WHEEL_NUM] = {0.0, 0.0};
int pwr[WHEEL_NUM] = {0, 0};

//Time variables
unsigned long prevT = 0;
unsigned long currT = 0;

//Declare Function
void GoalCb(const geometry_msgs::Twist&);
void setMotor(int ,int, int, int);

//ros publisher and transform broadcaster
ros::NodeHandle  nh;
ros::Subscriber<geometry_msgs::Twist> cmd_vel_sub("cmd_vel", GoalCb );

//debug msg
std_msgs::String debug_msg;
ros::Publisher debug_pub("debug",&debug_msg);

unsigned long callback_time = 0;
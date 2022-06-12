// ref: https://github.com/linorobot/linorobot/blob/master/src/lino_base.cpp

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <tf2_ros/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>

#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>

int WHEEL_NUM = 2;
float WHEEL_RAD = 0.125;
float WHEEL_DIST = 0.547;

#define TICKS_PER_REV 1530;

class WalkieBase
{
private:
  ros::Subscriber raw_vel_sub, raw_pos_sub;
  ros::Publisher odom_pub, joint_states_pub;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  ros::NodeHandle nh;
  sensor_msgs::JointState joint_states;
  nav_msgs::Odometry odom;
  tf2::Quaternion odom_quat;

  double odom_pose[3] = { 0.0, 0.0, 0.0 };

  char odom_child_frame[30] = "base_footprint";
  char odom_header_frame[5] = "odom";

  char base_link_name[10] = "base_link";

  std::vector<double> joint_states_pos = { 0.0, 0.0, 0.0, 0.0 };
  std::vector<double> joint_states_vel = { 0.0, 0.0, 0.0, 0.0 };

  std::vector<std::string> joint_states_name = { "realsense_yaw_joint", "realsense_pitch_joint",
                                                 "left_back_wheel_joint", "right_back_wheel_joint" };

  double vel_left=0.0, vel_right=0.0, pos_left=0.0, pos_right=0.0;
  double last_theta = 0.0;

  int32_t prev_pos_left=0, prev_pos_right=0, delta_pos_left=0, delta_pos_right=0;
  double rad_left=0.0, rad_right=0.0;
public:
  WalkieBase()
  {
    raw_vel_sub = nh.subscribe("/walkie/raw_vel", 1000, &WalkieBase::velCallback, this);
    raw_pos_sub = nh.subscribe("/walkie/raw_pos", 1000, &WalkieBase::posCallback, this);

    odom_pub = nh.advertise<nav_msgs::Odometry>("/walkie/odom", 50);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/walkie/joint_states", 50);

    joint_states.header.frame_id = base_link_name;
    joint_states.name = joint_states_name;
    
  }
  void velCallback(const std_msgs::Float32MultiArray::ConstPtr& msg)
  {
    vel_left = (double) msg->data[0];
    vel_right = (double) msg->data[1];
  }

  void posCallback(const std_msgs::Int32MultiArray::ConstPtr& msg)
  {
    pos_left = msg->data[0];
    pos_right = msg->data[1];
  }
 
  void update_odom()
  { 

    //Calculate velocity for the robot
    double Vx = ((vel_left+vel_right)/2); //Velocity forward
    double Vw = ((vel_right-vel_left)/WHEEL_DIST); //Angular velocity

    // float dw = Vw * 0.05 ;// second
    // odom_pose[2] += dw;

    // float Vxx = (Vx * cos(Vw));
    // float Vxy = (-1*Vx * sin(Vw));

    // float delta_x = (Vxx*cos(odom_pose[2])-Vxy*sin(odom_pose[2])) * 0.05;
    // float delta_y = (Vxx*sin(odom_pose[2])+Vxy*cos(odom_pose[2])) * 0.05;

    // odom_pose[0] += delta_x;
    // odom_pose[1] += delta_y;
    
    double delta_s = WHEEL_RAD * (rad_left + rad_right) / 2.0;
    double theta = WHEEL_RAD * (rad_right - rad_left) / WHEEL_DIST;

    double delta_theta = theta - last_theta;

    // TODO change odom equation according to lino
    
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;
    

    odom_trans.header.frame_id = odom_header_frame;
    odom_trans.child_frame_id = odom_child_frame;

    odom.header.frame_id = odom_header_frame;
    odom.child_frame_id = odom_child_frame;

    odom_trans.transform.translation.x = odom_pose[0];
    odom_trans.transform.translation.y = odom_pose[1];
    odom_trans.transform.translation.z = 0.4021;

    odom_quat.setRPY(0, 0, odom_pose[2]);
    odom_trans.transform.rotation.x = odom_quat.x();
    odom_trans.transform.rotation.y = odom_quat.y();
    odom_trans.transform.rotation.z = odom_quat.z();
    odom_trans.transform.rotation.w = odom_quat.w();
    odom_trans.header.stamp = ros::Time::now();

    odom_broadcaster.sendTransform(odom_trans);

    odom.header.frame_id = odom_header_frame;
    odom.child_frame_id = odom_child_frame;

    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    odom.pose.pose.position.z = 0.4021;
    odom.pose.pose.orientation = odom_trans.transform.rotation;
    odom.header.stamp = ros::Time::now();

    odom.twist.twist.linear.x = Vx;
    odom.twist.twist.angular.z = Vw;

    odom_pub.publish(odom);
  }
  void update_joints()
  {
        //Radian per sec of each wheel
    delta_pos_left = pos_left - prev_pos_left;
    delta_pos_right = prev_pos_right - pos_right ;
    
    prev_pos_left = pos_left;
    prev_pos_right = pos_right;
    
    rad_left = delta_pos_left*2*M_PI/TICKS_PER_REV;
    rad_right = delta_pos_right*2*M_PI/TICKS_PER_REV;
    
    joint_states_pos[2] += rad_left;
    joint_states_pos[3] += rad_right;

    joint_states_vel[2] = vel_left;
    joint_states_vel[3] = vel_right;

    joint_states.position = joint_states_pos;
    joint_states.velocity = joint_states_vel;

    joint_states.header.stamp=ros::Time::now();

    joint_states_pub.publish(joint_states);
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "bringup");

  WalkieBase walkie;

  ros::Rate loop_rate(20);


  while (ros::ok())
  {

    walkie.update_odom();

    walkie.update_joints();

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

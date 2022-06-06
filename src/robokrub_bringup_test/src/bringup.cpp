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

int WHEEL_NUM = 2;
float WHEEL_RAD = 0.125;
float WHEEL_DIST = 0.547;

class WalkieBase
{
private:
  ros::Subscriber vel_left_sub, vel_right_sub, pos_left_sub, pos_right_sub;
  ros::Publisher odom_pub, joint_states_pub;
  tf2_ros::TransformBroadcaster odom_broadcaster;
  geometry_msgs::TransformStamped odom_trans;
  ros::NodeHandle nh;
  sensor_msgs::JointState joint_states;
  nav_msgs::Odometry odom;
  tf2::Quaternion odom_quat;

  float odom_pose[3] = { 0.0, 0.0, 0.0 };

  char odom_child_frame[30] = "base_footprint";
  char odom_header_frame[5] = "odom";

  char base_link_name[10] = "base_link";

  float joint_states_pos[4] = { 0.0, 0.0, 0.0, 0.0 };
  float joint_states_vel[4] = { 0.0, 0.0, 0.0, 0.0 };

  std::vector<std::string> joint_states_name = { "realsense_yaw_joint", "realsense_pitch_joint",
                                                 "left_back_wheel_joint", "right_back_wheel_joint" };

  double vel_left, vel_right, pos_left, pos_right;
  double last_theta;

public:
  WalkieBase()
  {
    vel_left_sub = nh.subscribe("/walkie/vel_left", 1000, &WalkieBase::velLeftCallback, this);
    vel_right_sub = nh.subscribe("/walkie/vel_right", 1000, &WalkieBase::velRightCallback, this);
    pos_left_sub = nh.subscribe("/walkie/pos_left", 1000, &WalkieBase::posLeftCallback, this);
    pos_right_sub = nh.subscribe("/walkie/pos_right", 1000, &WalkieBase::posRightCallback, this);
    odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 50);
    joint_states_pub = nh.advertise<sensor_msgs::JointState>("/joint_states", 50);

    joint_states.header.frame_id = base_link_name;
    joint_states.name = joint_states_name;
  }
  void velLeftCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    vel_left = (double)msg->data;
  }
  void velRightCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    vel_right = (double)msg->data;
  }
  void posLeftCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    pos_left = (double)msg->data;
  }
  void posRightCallback(const std_msgs::Float32::ConstPtr& msg)
  {
    pos_right = (double)msg->data;
  }
  void update_odom()
  {
    odom_trans.header.frame_id = odom_header_frame;
    odom_trans.child_frame_id = odom_child_frame;

    double delta_s = WHEEL_RAD * (pos_left + pos_right) / 2.0;
    double theta = WHEEL_RAD * (pos_right - pos_left) / WHEEL_DIST;

    double delta_theta = theta - last_theta;
    last_theta = theta;

    // TODO change odom equation according to lino
    odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
    odom_pose[2] += delta_theta;

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

    odom.twist.twist.linear.x = delta_s;
    odom.twist.twist.angular.z = delta_theta;

    odom_pub.publish(odom);
  }
  void update_joints()
  {
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

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}

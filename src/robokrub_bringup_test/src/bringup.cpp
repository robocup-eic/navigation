#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/String.h>
#include <ros/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Quaternion.h>

#include <ros/time.h>
#include <math.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>

int WHEEL_NUM = 2;
float WHEEL_RAD = 0.125;
float WHEEL_DIST = 0.547;

class Odom{
  private:
    ros::Subscriber vel_left_sub, vel_right_sub, pos_left_sub, pos_right_sub;
    ros::Publisher odom_pub, joint_states_pub;
    ros::NodeHandle nh;
    sensor_msgs::JointState joint_states;
    nav_msgs::Odometry odom;
    float odom_pose[3] = {0.0, 0.0, 0.0};

    char odom_child_frame[30] = "base_footprint";
    char odom_header_frame[5] = "odom";

    char base_link_name[10] = "base_link";

    float joint_states_pos[4] = {0.0, 0.0, 0.0, 0.0};
    float joint_states_vel[4] = {0.0, 0.0, 0.0, 0.0};

    char *joint_states_name[4] = {(char*)"realsense_yaw_joint", (char*)"realsense_pitch_joint",(char*)"left_back_wheel_joint", (char*)"right_back_wheel_joint"};
    
    double vel_left, vel_right, pos_left, pos_right;

  public:
    Odom(){
      vel_left_sub = nh.subscribe("/walkie/vel_left", 1000, &Odom::velLeftCallback, this);
      vel_right_sub = nh.subscribe("/walkie/vel_right", 1000, &Odom::velRightCallback, this);
      pos_left_sub = nh.subscribe("/walkie/pos_left", 1000, &Odom::posLeftCallback, this);
      pos_right_sub = nh.subscribe("/walkie/pos_right", 1000, &Odom::posRightCallback, this);

      joint_states.header.frame_id = base_link_name;
      joint_states.name = joint_states_name;

      joint_states.name_length     = 4;
      joint_states.position_length = 4;
      joint_states.velocity_length = 4;
      joint_states.effort_length   = 4;
    }
    void velLeftCallback(const std_msgs::Float32::ConstPtr& msg){
        vel_left = (double) msg->data;
    }
    void velRightCallback(const std_msgs::Float32::ConstPtr& msg){
        vel_right = (double) msg->data;
    }
    void posLeftCallback(const std_msgs::Float32::ConstPtr& msg){
        pos_left = (double) msg->data;
    }
    void posRightCallback(const std_msgs::Float32::ConstPtr& msg){
        pos_right = (double) msg->data;
    }
    void update_odom(){
        
        odom.header.frame_id = odom_header_frame;
        odom.child_frame_id = odom_child_frame;   

        
        odom_trans.header.frame_id = odom_header_frame;    
        odom_trans.child_frame_id = odom_child_frame;   
    
        float delta_s = WHEEL_RAD * (rad[0]+rad[1])/2.0;
        float theta = WHEEL_RAD * (rad[1]-rad[0])/WHEEL_DIST;

        float delta_theta = theta-last_theta;
        
        odom_pose[0] += delta_s * cos(odom_pose[2] + (delta_theta / 2.0));
        odom_pose[1] += delta_s * sin(odom_pose[2] + (delta_theta / 2.0));
        odom_pose[2] += delta_theta;
        
        
        
        odom_trans.transform.translation.x = odom_pose[0];
        odom_trans.transform.translation.y = odom_pose[1];
        odom_trans.transform.translation.z = 0.4021;
        odom_trans.transform.rotation = tf::createQuaternionFromYaw(odom_pose[2]); 
        odom_trans.header.stamp = nh.now();

        odom.pose.pose.position.x = odom_pose[0];
        odom.pose.pose.position.y = odom_pose[1];
        odom.pose.pose.position.z = 0.4021;
        odom.pose.pose.orientation = odom_trans.transform.rotation;
        odom.header.stamp = nh.now();

        odom.twist.twist.linear.x = velocity[0];
        odom.twist.twist.angular.z = velocity[1];
    }
    void update_joints(){
        
    }
    


};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "bringup");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  


  
  ros::Rate loop_rate(20);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

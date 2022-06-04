#include <Arduino.h>
#include "no_odom_cfg.h"

void setup() {
  // put your setup code here, to run once:


}

//Function for commanding motor
void setMotor(int pwr, int digiPin1, int digiPin2, int analogPin)
{
  if(pwr<0){
    digitalWrite(digiPin1,HIGH);
    digitalWrite(digiPin2,LOW);
    analogWrite(analogPin,-pwr);
  }
  else{
    digitalWrite(digiPin1,LOW);
    digitalWrite(digiPin2,HIGH);
    analogWrite(analogPin,pwr);
  }
}

//Callback for cmd_vel subscription
void GoalCb(const geometry_msgs::Twist& twist_msg){

  callback_time = millis();
  
  float x = twist_msg.linear.x;
  float z = twist_msg.angular.z;

  //Calculate goal_veclocity(rpm) fro both left and right wheel
  float goal_left = x-z*WHEEL_DIST/2;
  float goal_right = x+z*WHEEL_DIST/2;

  goal_left = goal_left*60.0/(2*M_PI*WHEEL_RAD);
  goal_right = goal_right*60.0/(2*M_PI*WHEEL_RAD);


  //Limit the maximum speed of the robot
  if(goal_left>80.0){
    goal_left = 80.0;
  }
  else if(goal_left<-80.0){
    goal_left = -80.0;
  }

  if(goal_right>80.0){
    goal_right=80.0;
  }
  else if(goal_right<-80.0){
    goal_right=-80.0;
  }

  //Assign it to the array variable
  goal[0] = goal_left;
  goal[1] = goal_right;
}

void loop() {
  // put your main code here, to run repeatedly:

}
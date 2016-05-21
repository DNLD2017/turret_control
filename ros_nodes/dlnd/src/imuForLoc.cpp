#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "dlnd/poseMsg.h"

#include <math.h>
#include <string>
#include <stdio.h>
#include <stdlib.h>
#include "iostream"

std::string imuid = "default";
double imux = 0;
double imuy = 0;
double imuz = 0;
double imuw = 0;

void listenerImu( const sensor_msgs::Imu::ConstPtr& msg)
{  
  imuid = msg->header.frame_id.c_str();
  imux = msg->orientation.x;
  imuy = msg->orientation.y;
  imuz = msg->orientation.z;
  imuw = msg->orientation.w;
  ROS_INFO("I heard id: [%s]",msg->header.frame_id.c_str());
  ROS_INFO("I heard quaternion x: [%f]",msg->orientation.x);
  ROS_INFO("I heard quaternion y: [%f]",msg->orientation.y);
  ROS_INFO("I heard quaternion z: [%f]",msg->orientation.z);
  ROS_INFO("I heard quaternion w: [%f]",msg->orientation.w);

}
int main(int argc, char **argv){

  ros::init(argc,argv,"imuForLoc");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/turret1/imu",1000,listenerImu);
  ros::Publisher chatter_pub = n.advertise<dlnd::poseMsg>("/turret1/posbr",1000);
  ros::Rate loop_rate(10);


  while(ros::ok()){
      dlnd::poseMsg msg;
      msg.id = atoi(imuid.c_str() );
      msg.orientation.x = imux;
      msg.orientation.y = imuy;
      msg.orientation.z = imuz;
      msg.orientation.w = imuw;
      chatter_pub.publish(msg);
      ros::spinOnce();
      loop_rate.sleep();
  }
return 0;
}

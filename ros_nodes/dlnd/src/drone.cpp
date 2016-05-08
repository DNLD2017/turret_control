#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include <visualization_msgs/Marker.h>

int main(int argc, char **argv){
    ros::init(argc,argv,"drone");

    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<visualization_msgs::Marker>("drone",100);

    ros::Rate loop_rate(100);

    while(ros::ok()){
        visualization_msgs::Marker msg;
        msg.header.frame_id = "/my_frame";
        msg.header.stamp = ros::Time::now();
        msg.ns = "drone";
        msg.action = visualization_msgs::Marker::ADD;
        msg.pose.orientation.w = 1.0;
        msg.id = 0;
        msg.type = visualization_msgs::Marker::POINTS;
        msg.scale.x = 0.2;
        msg.scale.y = 0.2;
        msg.color.g = 1.0f;
        msg.color.a = 1.0;
        geometry_msgs::Point point;
        point.x = 10*cos(ros::Time::now().toSec());
        point.y = 5*sin(1.35*ros::Time::now().toSec());
        point.z = 7*cos(ros::Time::now().toSec())+sin(0.5*ros::Time::now().toSec())+10;
        msg.points.push_back(point);
        chatter_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;

}

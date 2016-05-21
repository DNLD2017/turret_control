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
        msg.action = visualization_msgs::Marker::ADD;
        msg.type = visualization_msgs::Marker::MESH_RESOURCE;
        msg.scale.x = 0.0003;
        msg.scale.y = 0.0003;
        msg.scale.z = 0.0003;
        msg.color.g = 1.0f;
        msg.color.a = 0.5;
        msg.mesh_resource = "package://dlnd/blender/drone.dae";
        msg.pose.position.x = 1.5*cos(ros::Time::now().toSec())+4;
        msg.pose.position.y = 1.5*sin(ros::Time::now().toSec());
        msg.pose.position.z = -1;
        msg.pose.orientation.w = 1.0;
        msg.pose.orientation.x = 1.0;
        chatter_pub.publish(msg);
        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;

}

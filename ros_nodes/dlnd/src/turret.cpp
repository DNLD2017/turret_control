#include "ros/ros.h"
#include "dlnd/poseMsg.h"
#include "dlnd/camMsg.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <cstdlib>

float x_drone;
float y_drone;
float z_drone;
float x;
float y;
float z;
int id;

void CB_drone(const visualization_msgs::Marker::ConstPtr& msg){
    x_drone = msg->points.front().x;
    y_drone = msg->points.front().y;
    z_drone = msg->points.front().z;
}

int main(int argc, char **argv){

    ros::init(argc,argv,"turret");

    ros::NodeHandle n;

    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("position",100);
    ros::Publisher posbr_pub = n.advertise<dlnd::poseMsg>("posbr",100);
    ros::Publisher cambr_pub = n.advertise<dlnd::camMsg>("cambr",100);
    ros::Subscriber drone_sub = n.subscribe<visualization_msgs::Marker>("/drone", 100, CB_drone);

    ros::Rate loop_rate(100);

    x_drone = 0;
    y_drone = 0;
    z_drone = 0;
    n.getParam("x", x);
    n.getParam("y", y);
    n.getParam("z", z);
    n.getParam("id", id);

    while(ros::ok()){
        geometry_msgs::PoseStamped pos;
        pos.header.frame_id = "/my_frame";
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = x;
        pos.pose.position.y = y;
        pos.pose.position.z = z;
        tf::Quaternion q;
        q.setRPY(0,atan2(-z_drone+z, sqrt(pow(y_drone-y,2)+pow(x_drone-x,2))),atan2(y_drone-y, x_drone-x));
        pos.pose.orientation.w = q.getW();
        pos.pose.orientation.x = q.getX();
        pos.pose.orientation.y = q.getY();
        pos.pose.orientation.z = q.getZ();
        pos_pub.publish(pos);

        dlnd::poseMsg turret;
        turret.id = id;
        geometry_msgs::Quaternion qg;
        tf::quaternionTFToMsg(q, qg);
        turret.pos = qg;
        posbr_pub.publish(turret);

        dlnd::camMsg cam;
        cam.id = id;
        cam.epsi = (rand()-0.5)*3.1415926/360;
        cam.etheta = (rand()-0.5)*3.1415926/360;
        cambr_pub.publish(cam);

        ros::spinOnce();

        loop_rate.sleep();
    }
    return 0;

}

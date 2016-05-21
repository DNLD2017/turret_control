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
float yaw, pitch, roll;
int id;
bool fix;

void CB_drone(const visualization_msgs::Marker::ConstPtr& msg){
    x_drone = msg->pose.position.x;
    y_drone = msg->pose.position.y;
    z_drone = msg->pose.position.z;
}

int main(int argc, char **argv){

    ros::init(argc,argv,"turret");
    srand(time(NULL));
    fix = false;

    ros::NodeHandle n;

    ros::Publisher pos_pub = n.advertise<geometry_msgs::PoseStamped>("position",100);
    ros::Publisher posbr_pub = n.advertise<dlnd::poseMsg>("posbr",100);
    ros::Publisher cambr_pub = n.advertise<dlnd::camMsg>("cambr",100);
    ros::Publisher cam3d_pub = n.advertise<visualization_msgs::Marker>("cam3d",100);
    ros::Publisher pot3d_pub = n.advertise<visualization_msgs::Marker>("pot3d",100);
    ros::Subscriber drone_sub = n.subscribe<visualization_msgs::Marker>("/drone", 100, CB_drone);

    ros::Rate loop_rate(100);

    x_drone = 0;
    y_drone = 0;
    z_drone = 0;
    n.getParam("x", x);
    n.getParam("y", y);
    n.getParam("z", z);
    n.getParam("id", id);

    if(n.getParam("yaw", yaw) && n.getParam("pitch", pitch) && n.getParam("roll", roll)){
        fix = true;
    }

    while(ros::ok()){
        geometry_msgs::PoseStamped pos;
        pos.header.frame_id = "/my_frame";
        pos.header.stamp = ros::Time::now();
        pos.pose.position.x = x;
        pos.pose.position.y = y;
        pos.pose.position.z = z;
        tf::Quaternion q;
        double a,b,c;
        a=(( rand()/(double)RAND_MAX ) * 4-2)*3.1415926/180*0.5;
        b=(( rand()/(double)RAND_MAX ) * 4-2)*3.1415926/180*0.5;
        c=(( rand()/(double)RAND_MAX ) * 4-2)*3.1415926/180*0.5;
        if (!fix){
            roll = 0;
            pitch = atan2(z_drone-z+a, sqrt(pow(y_drone-y,2)+pow(x_drone-x,2)))+b;
            yaw = atan2(y_drone-y, x_drone-x)+c;
        }
        q.setRPY(roll, -pitch, yaw);
        pos.pose.orientation.w = q.getW();
        pos.pose.orientation.x = q.getX();
        pos.pose.orientation.y = q.getY();
        pos.pose.orientation.z = q.getZ();
        pos_pub.publish(pos);

        visualization_msgs::Marker rvizCam, rvizPot;
        rvizCam.header.frame_id = "/my_frame";
        rvizPot.header.frame_id = "/my_frame";
        rvizCam.header.stamp = ros::Time::now();
        rvizPot.header.stamp = ros::Time::now();
        rvizCam.type = visualization_msgs::Marker::MESH_RESOURCE;
        rvizPot.type = visualization_msgs::Marker::MESH_RESOURCE;
        rvizCam.mesh_resource = "package://dlnd/blender/camera.dae";
        rvizPot.mesh_resource = "package://dlnd/blender/poteau.dae";
        rvizCam.pose.position = pos.pose.position;
        rvizPot.pose.position = pos.pose.position; (rvizPot.pose.position.z)-=1.8;
        rvizCam.pose.orientation = pos.pose.orientation;
        rvizCam.color.a = 1.0;
        rvizCam.color.r = 1.0;
        rvizPot.color.a = 1.0;
        rvizPot.color.r = 1.0;
        rvizCam.scale.x = 0.2;
        rvizCam.scale.y = 0.2;
        rvizCam.scale.z = 0.2;
        rvizPot.scale.x = 0.2;
        rvizPot.scale.y = 0.2;
        rvizPot.scale.z = 1.0;
        cam3d_pub.publish(rvizCam);
        pot3d_pub.publish(rvizPot);


        q.setRPY(roll, pitch, yaw);
        dlnd::poseMsg turret;
        turret.id = id;
        geometry_msgs::Quaternion qg;
        tf::quaternionTFToMsg(q, qg);
        turret.pos = qg;
        posbr_pub.publish(turret);

        dlnd::camMsg cam;
        cam.id = id;
        if (fix){
            cam.epsi = atan2(y_drone-y, x_drone-x)+c-yaw;
            cam.etheta = atan2(z_drone-z+a, sqrt(pow(y_drone-y,2)+pow(x_drone-x,2)))+b-pitch;
        }
        else{
            cam.epsi = (( rand()/(double)RAND_MAX )-0.5)*3.1415926/360;
            cam.etheta = (( rand()/(double)RAND_MAX )-0.5)*3.1415926/360;
        }

        cambr_pub.publish(cam);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;

}

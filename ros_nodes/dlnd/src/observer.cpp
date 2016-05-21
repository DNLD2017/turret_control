#include "ros/ros.h"
#include "vector"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "visualization_msgs/Marker.h"
#define D_VCT std::vector<double>

D_VCT Xd,X;

void callback_drone(const visualization_msgs::Marker::ConstPtr& msg){
    std::cout << "hello" << std::endl;
    Xd[0] = msg->pose.position.x;
    Xd[1] = msg->pose.position.y;
    Xd[2] = msg->pose.position.z;
}

void callback_estim(const visualization_msgs::Marker::ConstPtr& msg){
    std::cout << "hello" << std::endl;
    X[0] = msg->pose.position.x;
    X[1] = msg->pose.position.y;
    X[2] = msg->pose.position.z;
}

int main(int argc, char **argv){
    ros::init(argc,argv,"observer");
    ros::NodeHandle n;
    ros::Subscriber drone_sub = n.subscribe<visualization_msgs::Marker>("/drone",1000,callback_drone);
    ros::Subscriber est_sub = n.subscribe<visualization_msgs::Marker>("/drone_estimation",1000,callback_estim);
    ros::Publisher pub = n.advertise<std_msgs::Float64MultiArray>("/errors", 1000);

    ros::Rate loop_rate(100);
    std_msgs::Float64MultiArray msg;
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = 3;
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "errors";

    D_VCT XXd(3,0);
    D_VCT XX(3,0);
    D_VCT::iterator i1 = XXd.begin();
    D_VCT::iterator i2 = XX.begin();
    Xd.assign(i1,XXd.end());
    X.assign(i2,XX.end());

    std::cout << Xd[0] << std::endl;

    std::cout << X[1] << std::endl;

    D_VCT Y(3,0);

    std::cout << "hello" << std::endl;

    while(ros::ok()){
        msg.data.clear();
        Y[0] = Xd[0]-X[0];
        Y[1] = Xd[1]-X[1];
        Y[2] = Xd[2]-X[2];
        std::cout << "hello" << std::endl;
        msg.data.insert(msg.data.end(),Y.begin(),Y.end());
        pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
    }
}

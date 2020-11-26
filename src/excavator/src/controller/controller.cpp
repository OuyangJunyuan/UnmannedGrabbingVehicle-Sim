//
// Created by ou on 2020/11/20.
//

#include "controller.h"

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/PointCloud2.h>


using namespace std;

ros::Publisher pubs[4];
void cb(const std_msgs::Float64MultiArray::ConstPtr msg){
    for (int i = 0; i < msg->data.size(); ++i) {
        if(i<=4)
            pubs[i].publish(msg->data[i]);
        cout<<i<<endl;
    }
}
int main(int argc,char **argv)
{
    ros::init(argc, argv, "joint_controller");
    ros::NodeHandle n;

    pubs[0]==n.advertise<std_msgs::Float64>("/WaistJoint_position_controller/command", 100);
    pubs[1]=n.advertise<std_msgs::Float64>("/ArmBaseJoint_position_controller/command", 100);
    pubs[2]=n.advertise<std_msgs::Float64>("/Arm12Joint_position_controller/command", 100);
    pubs[3]=n.advertise<std_msgs::Float64>("/Arm23Joint_position_controller/command", 100);

    ros::Subscriber sub=n.subscribe("/joint_cmd",10,cb);

    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



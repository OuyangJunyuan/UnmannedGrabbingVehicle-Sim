//
// Created by ou on 2020/11/27.
//
#include "pointcloudprocess.h"

int main(int argc,char **argv)
{
    ros::init(argc, argv,"LiDAR_PC_Process");

    PointCloudProcess pcprocess;

    ros::Rate loop_rate(100);
    while(ros::ok()){
        pcprocess.msgqueue.callAvailable();

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

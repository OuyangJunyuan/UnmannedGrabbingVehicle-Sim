//
// Created by ou on 2020/11/25.
//

#include "pointcloudprocess.h"

#include <ros/ros.h>
#include <ros/advertise_options.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/voxel_grid.h>
using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;
typedef pcl::PCLPointCloud2 PC2;

ros::NodeHandle *n;

void CallBack(sensor_msgs::PointCloud2::ConstPtr msg,int i){
    static sensor_msgs::PointCloud2 ROSfusedPC;
    static PCXYZ fusedPC;
    static double *lasttime=new double[sizeof(vTopicName.size())];
    /* get tf for initial value (rude)*/
    tf::StampedTransform transform_tf;
    try{
        pTFlisten->lookupTransform("base_link",vLiDARFrame[i], ros::Time(0), transform_tf);
        ROS_INFO("find tf %s in base_link",vLiDARFrame[i].c_str());
        TF2Eigen(transform_tf,vTf[i]);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    /* check validity */

    if(ros::Time::now().toSec()-lasttime[i]>=0.5){
        ROS_INFO("topic '%s' is timeout and ignored ",vTopicName[i].c_str());
        lasttime[i]=ros::Time::now().toSec();
        return ;
    }
    lasttime[i]=ros::Time::now().toSec();
    /* fusion */
    ROS_INFO("%d topic fused",i);
    ROS_INFO("time : %lf",lasttime[i]);
    PCXYZ newPC,tfPC;
    pcl::fromROSMsg(*msg,newPC);
    pcl::transformPointCloud(newPC,tfPC,vTf[i]);
    fusedPC+=tfPC;

    /* downsampling using vaxelgrid and send to topic */
    if(i==vTopicName.size()-1){
        pcl::VoxelGrid<PC2> voxel_grid;
        voxel_grid.setLeafSize(0.1f,0.1f,0.1f);
        //cvt to pc2
        PC2::Ptr pfusedPC2(new PC2),pfilteredPC2(new PC2);
        pcl::toPCLPointCloud2(fusedPC,*pfusedPC2);
        voxel_grid.setInputCloud(pfusedPC2);
        voxel_grid.filter(*pfilteredPC2);

        voxel_grid.filter(*pfilteredPC2);
        pcl::fromPCLPointCloud2(*pfilteredPC2,fusedPC);
        pcl::toROSMsg(fusedPC,ROSfusedPC);
        ROSfusedPC.header.frame_id="base_link";
        pub.publish(ROSfusedPC);
        ROS_INFO("fusedPC have %d points",fusedPC.size());
        fusedPC.clear();
    }
}

int main(int argc,char **argv)
{
    ros::init(argc, argv,"LiDAR_PC_Process");

    n=new ros::NodeHandle;
    n->getParam("LiDAR_topic",vTopicName);
    n->getParam("LiDAR_frame",vLiDARFrame);
    n->param<string>("Output_topic",outputTopicName,"fused_lidar");


    ros::CallbackQueue msgqueue;
    vTf.resize(vTopicName.size());
    pTFlisten = new tf::TransformListener;

    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            outputTopicName, 1,
            boost::bind(CntCB),
            boost::bind(CntCB),
            ros::VoidPtr(), &msgqueue);
    pub = n->advertise(ao);

    ros::Rate loop_rate(1000);
    while(ros::ok()){
        msgqueue.callAvailable();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}



//
// Created by ou on 2020/11/25.
//

#ifndef POINTCLOUD_PROCESS_POINTCLOUDPROCESS_H
#define POINTCLOUD_PROCESS_POINTCLOUDPROCESS_H
#include <chrono>
#include <ros/ros.h>
#include <ros/advertise_options.h>
#include <sensor_msgs/PointCloud2.h>


#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;
using namespace pcl;
class PointCloudProcess{
    typedef pcl::PointCloud<pcl::PointXYZ> PCXYZ;

private:
    ros::NodeHandle *n;
    ros::Publisher pub;
    vector<ros::Subscriber> vSubs;
    vector<string> vTopicName,vLiDARFrame;
    string outputTopicName;

    vector<Eigen::Isometry3f> vTf;
    tf::TransformListener *pTFlisten;


    PCXYZ::Ptr gCloud;
    float ROI_R_max,ROI_R_min;
    float clusterTolerance,minSize,maxSize;
    //    pcl::visualization::PCLVisualizer::Ptr viewer;

    void ConnectCallBack();
    void MsgCallBack(sensor_msgs::PointCloud2::ConstPtr msg,int i);

    void Segment();
public:
    ros::CallbackQueue msgqueue;
    PointCloudProcess();


};




#endif //POINTCLOUD_PROCESS_POINTCLOUDPROCESS_H

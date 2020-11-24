//
// Created by ou on 2020/11/20.
//

#include "PointCloudFusion.h"

#include <glog/logging.h>

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/cloud_viewer.h>


using namespace std;

tf::TransformListener *ptfh;
ros::Publisher pub1;

void chatterCallback(const sensor_msgs::PointCloud2 ::ConstPtr& msg)
{
    /* get tf  */
    tf::StampedTransform transform_tf;
    tf::Vector3 tf_t;
    try{
        ptfh->lookupTransform("base_link", "RubyLite1_frame", ros::Time(0), transform_tf);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
        return;
    }
    /* cvt transform type */
    tf_t = transform_tf.getRotation().getAxis();

    Eigen::Affine3f transform_eg = Eigen::Affine3f::Identity();
    transform_eg.translation() <<  transform_tf.getOrigin().x(), transform_tf.getOrigin().y(), transform_tf.getOrigin().z();
    transform_eg.rotate ( Eigen::AngleAxisf( transform_tf.getRotation().getAngle(), Eigen::Vector3f ( tf_t.x(),tf_t.y(),tf_t.z() ) ));
//    LOG(WARNING)<< transform_tf.getRotation().getAngle()<<" "<<tf_t.x()<<" "<< tf_t.y()<<" "<<tf_t.z()<<endl;
    /* init cloud */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>()), transformed_cloud (new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromROSMsg(*msg,*cloud);
    pcl::transformPointCloud(*cloud,*transformed_cloud,transform_eg);


    sensor_msgs::PointCloud2 globalcloud;
    pcl::toROSMsg(*transformed_cloud,globalcloud);
    globalcloud.header.frame_id="base_link";
    pub1.publish(globalcloud);


//    pcl::visualization::PCLVisualizer viewer ("Matrix transformation example");
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> source_cloud_color_handler (cloud, 0, 0, 255);
//    viewer.addPointCloud (cloud,source_cloud_color_handler, "original_cloud");
//
//
//    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> transform_cloud_color_handler (transformed_cloud, 255, 0, 0);
//    viewer.addPointCloud (transformed_cloud,transform_cloud_color_handler, "transform_cloud");
//    while (!viewer.wasStopped ()) { // 在按下 "q" 键之前一直会显示窗口
//        viewer.spinOnce ();
//    }

    // int POINT_STEP = 22;

//    int size=msg->data.size();
//    LOG(WARNING)<<"size : "<<size<<endl;
//    globalcloud.data.resize(size);
//
//
//    tf::Vector3 vector3;
//    tf::Vector3 vector31;
//    uint8_t *ptrtail = globalcloud.data.data()+globalcloud.data.size();
//    for(uint8_t *ptr = globalcloud.data.data(),*msgptr = (uint8_t *)(msg->data.data());ptr<ptrtail;ptr+=POINT_STEP,msgptr+=POINT_STEP){
//        vector3.setX(*((float*)(msgptr+0)));
//        vector3.setY(*((float*)(msgptr+4)));
//        vector3.setZ(*((float*)(msgptr+8)));
//        vector31=transform*vector3;
//        *((float*)(ptr + 0)) = vector31.x();
//        *((float*)(ptr + 4)) = vector31.y();
//        *((float*)(ptr + 8)) = vector31.z();
//        *((float*)(ptr + 12)) = *((float*)(msgptr+12));
//        *((uint16_t*)(ptr + 16)) = *((uint16_t*)(msgptr+16));
//        *((float*)(ptr + 18)) = 0.0; // time
//    }
//    globalcloud.point_step=POINT_STEP;
//    globalcloud.row_step=globalcloud.data.size();
//    globalcloud.height=1;
//    globalcloud.width= msg->row_step / POINT_STEP;
//    globalcloud.is_bigendian=false;
//    globalcloud.is_dense=true;
//    globalcloud.header.stamp=ros::Time::now();
//    pub1.publish(globalcloud);
}
int main(int argc,char **argv)
{

    google::InitGoogleLogging(argv[0]); // Initialize Google's logging library.
//    google::InstallFailureWriter(&FatalMessageDump); 配合使用，可以在程序出现严重错误时将详细的错误信息打印出来
    google::SetLogFilenameExtension("log_");// setting output  prefix-filename

    google::SetLogDestination(google::INFO, "/home/ou/workspace/ugv_ws/src/excavator/log/info");
    google::SetLogDestination(google::WARNING, "/home/ou/workspace/ugv_ws/src/excavator/log/warning");
    google::SetLogDestination(google::GLOG_ERROR, "./home/ou/workspace/ugv_ws/src/excavator/log/error");
    google::SetStderrLogging(google::WARNING); //level above argument will output on screen/terminal

    ros::init(argc, argv, "PointCloudFusion");
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("RubyLite1", 1000, chatterCallback);
    pub1 = n.advertise<sensor_msgs::PointCloud2>("PointCloudFused",1);

    tf::TransformListener listener;
    ptfh=&listener;


    ros::Publisher pub2=n.advertise<std_msgs::Float64>("/ArmBaseJoint_position_controller/command", 100);
    ros::Publisher pub3=n.advertise<std_msgs::Float64>("/Arm12Joint_position_controller/command", 100);
    ros::Publisher pub4=n.advertise<std_msgs::Float64>("/Arm23Joint_position_controller/command", 100);
    ros::Publisher pub5=n.advertise<std_msgs::Float64>("/WaistJoint_position_controller/command", 100);
    ros::Rate loop_rate(10);
    while(ros::ok()){
        static int cnt=0;
        cnt++;
        std_msgs::Float64 float64;
        float64.data=sin(cnt*M_PI*0.05)-0.5;
        pub2.publish(float64);
        pub3.publish(float64);
        pub4.publish(float64);
        pub5.publish(float64);
        ros::spinOnce();
        loop_rate.sleep();
    }
    google::ShutdownGoogleLogging();
    return 0;
}



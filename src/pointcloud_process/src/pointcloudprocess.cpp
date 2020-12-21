//
// Created by ou on 2020/11/25.
//

#include "pointcloudprocess.h"
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/kdtree/kdtree.h>
#include <pcl/octree/octree_search.h>
void TF2Eigen(tf::StampedTransform &_tf_trans,Eigen::Isometry3f &_eigen_trans){

    tf::Vector3 tf_t = _tf_trans.getRotation().getAxis();
    Eigen::Isometry3f eigen_trans = Eigen::Isometry3f::Identity();
    eigen_trans.translation() <<  _tf_trans.getOrigin().x(), _tf_trans.getOrigin().y(), _tf_trans.getOrigin().z();
    eigen_trans.rotate ( Eigen::AngleAxisf( _tf_trans.getRotation().getAngle(), Eigen::Vector3f ( tf_t.x(),tf_t.y(),tf_t.z() ) ));
    _eigen_trans=eigen_trans;
}

PointCloudProcess::PointCloudProcess(){
    n=new ros::NodeHandle;
    n->getParam("PointCloudFusion/LiDAR_topic",vTopicName);
    n->getParam("PointCloudFusion/LiDAR_frame",vLiDARFrame);
    n->param<string>("PointCloudFusion/Output_topic",outputTopicName,"fused_lidar");
    n->param<float>("PointCloudFilter/ROI_R_max",ROI_R_max,50);
    n->param<float>("PointCloudFilter/ROI_R_min",ROI_R_min,0);
    n->param<float>("PointCloudSegment/clusterTolerance",clusterTolerance,1);
    n->param<float>("PointCloudSegment/minSize",minSize,20);
    n->param<float>("PointCloudSegment/maxSize",maxSize,10000);


    ros::AdvertiseOptions ao = ros::AdvertiseOptions::create<sensor_msgs::PointCloud2>(
            outputTopicName, 1,
            boost::bind(&PointCloudProcess::ConnectCallBack,this),
            boost::bind(&PointCloudProcess::ConnectCallBack,this),
            ros::VoidPtr(), &msgqueue);
    pub = n->advertise(ao);


    vTf.resize(vTopicName.size());
    pTFlisten = new tf::TransformListener;

    gCloud=PCXYZ::Ptr(new PCXYZ);
}


void PointCloudProcess::ConnectCallBack(){
    cout<<pub.getNumSubscribers()<<endl;
    if(pub.getNumSubscribers()==1){
        vSubs.resize(vTopicName.size());
        for (int i = 0; i < vTopicName.size(); ++i) {
            ROS_INFO("subscribe topic : %s  \n frame : %s",vTopicName[i].c_str(),vLiDARFrame[i].c_str());
            vSubs[i] = n->subscribe<sensor_msgs::PointCloud2>(vTopicName[i],10,boost::bind(&PointCloudProcess::MsgCallBack,this,_1,i));
        }
    }else if(pub.getNumSubscribers()==0){
        ROS_INFO("clear subscriber");
        vSubs.clear();
    }
}
void PointCloudProcess::MsgCallBack(sensor_msgs::PointCloud2::ConstPtr msg, int i){
    static PCXYZ::Ptr pfusedPC(new PCXYZ);
    static double *lasttime = new double[sizeof(vTopicName.size())];

    /* get tf for initial value (rude)*/
    tf::StampedTransform transform_tf;
    try {
        pTFlisten->lookupTransform("base_link", vLiDARFrame[i], ros::Time(0), transform_tf);
        ROS_INFO("find tf %s in base_link", vLiDARFrame[i].c_str());
        TF2Eigen(transform_tf, vTf[i]);
    }
    catch (tf::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
        return;
    }

    /* check validity */
    if (ros::Time::now().toSec() - lasttime[i] >= 0.5) {
        ROS_INFO("topic '%s' is timeout and ignored ", vTopicName[i].c_str());
        lasttime[i] = ros::Time::now().toSec();
        return;
    }
    lasttime[i] = ros::Time::now().toSec();

    /* fusion */
    PCXYZ newFramePC, tfPC;
    pcl::fromROSMsg(*msg, newFramePC);
    pcl::transformPointCloud(newFramePC, tfPC, vTf[i]);
    *pfusedPC += tfPC;

    /* 认为第五帧到来后就融合5个LiDAR数据完毕 */
    if (i == vTopicName.size() - 1) {
        /* downsampling using vaxelgrid and send to topic */
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(0.1f, 0.1f, 0.1f);
        voxel_grid.setInputCloud(pfusedPC);
        voxel_grid.filter(*gCloud);
        /* do something */
        Segment();
        ROS_INFO("down sampling point cloud have %d points", gCloud->size());
        pfusedPC=PCXYZ::Ptr(new PCXYZ);
        /* send  */
//        sensor_msgs::PointCloud2 msgPC;
//        pcl::toROSMsg(*gCloud,msgPC);
//        msgPC.header.frame_id="base_link";
//        pub.publish(msgPC);
    }
}



void PointCloudProcess::Segment() {
    /* 距离滤波(box) */
//    vector<int> inliersindices;
//    pcl::CropBox<pcl::PointXYZ> region(true);
//    region.setMax(Eigen::Vector4f(ROI_R_max,ROI_R_max,100,1));
//    region.setMin(Eigen::Vector4f(-ROI_R_max,-ROI_R_max,0,1));
//    region.setInputCloud(gCloud);
//    region.filter(*gCloud);
//
//    region.setMax(Eigen::Vector4f(ROI_R_min,ROI_R_min,100,1));
//    region.setMin(Eigen::Vector4f(-ROI_R_min,-ROI_R_min,0,1));
//    region.setNegative(true);
//    region.setInputCloud(gCloud);
//    region.filter(*gCloud);
    chrono::steady_clock::time_point t1 = chrono::steady_clock::now();
    /* 半径滤波(Radius) */
    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree (1);
    octree.setInputCloud(gCloud);
    octree.addPointsFromInputCloud();

    std::vector<int> vIndices;
    std::vector<float> vDistance;
    pcl::PointXYZ searchPoint(0,0,0);
    if(octree.radiusSearch (searchPoint, ROI_R_max, vIndices, vDistance)>0){
        ROS_INFO("radius max filter ok : %f",ROI_R_max);
        pcl::copyPointCloud(*gCloud,vIndices,*gCloud);
    }

    octree=pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>(1);
    octree.setInputCloud(gCloud);
    octree.addPointsFromInputCloud();
    if(octree.radiusSearch (searchPoint, ROI_R_min, vIndices, vDistance)>0){
        ROS_INFO("radius min filter ok : %f",ROI_R_min);
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        indices->indices=vIndices;
        pcl::ExtractIndices<pcl::PointXYZ> ex;
        ex.setInputCloud(gCloud);
        ex.setIndices(indices);
        ex.setNegative(true);
        ex.filter(*gCloud);
    }
    /* 平面拟合 */
    pcl::ModelCoefficients::Ptr coeff(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(500);
    seg.setInputCloud(gCloud);
    seg.segment(*inliers, *coeff);
    std::cout << "inliers->indices.size = " << inliers->indices.size() << std::endl;

    /* 分隔出地面 */
    PCXYZ::Ptr goundPC(new PCXYZ);

    pcl::ExtractIndices<pcl::PointXYZ> ex;
    ex.setInputCloud(gCloud);
    ex.setIndices(inliers);
    ex.filter(*goundPC);

    /* 分割出地面物体 */
    PCXYZ::Ptr cutedPC(new PCXYZ);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(gCloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cutedPC);

    /* 输出到topic */
//    sensor_msgs::PointCloud2 output;
//    pcl::toROSMsg(filteredCloud, output);
//    output.header.frame_id = "base_link";
//    pub.publish(output);

    /* 聚类 */
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cutedPC);

    vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cutedPC);
    ec.extract(clusterIndices);
    ROS_INFO("%d clusters",clusterIndices.size());

    sensor_msgs::PointCloud2 msg2sent;
    pcl::PointCloud<pcl::PointXYZL> labelPC; //带标签的PC类型
    pcl::copyPointCloud(*cutedPC,labelPC);

    ROS_INFO("totall %d clusters",clusterIndices.size());
    /* 上聚类label */
    int i=0;
    for (auto it:clusterIndices)
    {
        for (auto pit:it.indices) {
            labelPC[pit].label = i;
            cout<<pit<<endl;
        }
        i++;
    }

    chrono::steady_clock::time_point t2 = chrono::steady_clock::now();
    chrono::duration<double> time_used = chrono::duration_cast<chrono::duration<double>>( t2-t1 );
    cout<<"solve time cost = "<<time_used.count()<<" seconds. "<<endl;
    /* 发送至可视化 */
    pcl::toROSMsg(labelPC,msg2sent);
    msg2sent.header.frame_id="base_link";
    pub.publish(msg2sent);
}







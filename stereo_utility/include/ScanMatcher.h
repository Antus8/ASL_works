#ifndef _SCAN_MATCHER_H_
#define _SCAN_MATCHER_H_

#include <iostream>
#include <ros/ros.h>
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <nav_msgs/Odometry.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/video/tracking.hpp>
#include <opencv2/calib3d/calib3d_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <Eigen/Eigen>
#include <geometry_msgs/Pose.h>


class ScanMatcher {

    public:    
                   
        ScanMatcher();
        ~ScanMatcher();

        bool Initialize(const ros::NodeHandle& n);

    protected: 

        std::string name_;  
        bool b_is_previous_stored_; 

        ros::Subscriber point_cloud_sub_;
        ros::Publisher odometry_pub_; 
        ros::Publisher pose_pub_; 

        bool RegisterCallbacks(const ros::NodeHandle& n); 
        bool CreatePublishers(const ros::NodeHandle& n);
      
        void PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg); 
        void PublishOdometry(float x, float y, float z, ros::Time& timestamp);
        void PublishPose(const tf::Pose& pose,const ros::Time& timestamp);

        pcl::PointCloud<pcl::PointXYZ>::Ptr previous_point_cloud_;
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp_;
        Eigen::Matrix4d transformation_;
        pcl::PointCloud<pcl::PointXYZ> final_; //Used to create the map

};

#endif

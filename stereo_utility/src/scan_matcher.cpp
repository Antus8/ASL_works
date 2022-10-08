#include <ScanMatcher.h>

ScanMatcher::ScanMatcher() {
    ROS_INFO("ScanMatcher Class Constructor");
    b_is_previous_stored_ = false;
    icp_.setMaximumIterations (100);
    icp_.setMaxCorrespondenceDistance (0.05);
    icp_.setTransformationEpsilon (1e-8);
    icp_.setEuclideanFitnessEpsilon (1);
}


ScanMatcher::~ScanMatcher() {
    ROS_INFO("VisualOdometry Class Destructor");
}


bool ScanMatcher::Initialize(const ros::NodeHandle& n) {
    ROS_INFO("ScanMatcher - Initialize");
    name_ = ros::names::append(n.getNamespace(), "ScanMatcher");
    
    if (!RegisterCallbacks(n)) {
        ROS_ERROR("Failed to register callbacks in %s", name_.c_str());
        return false;
    }
    if (!CreatePublishers(n)) {
        ROS_ERROR("Failed to create publishers in %s", name_.c_str());
        return false;
    }   
    return true;     
}



bool ScanMatcher::RegisterCallbacks(const ros::NodeHandle& n) {
    ROS_INFO("Registering online callbacks in %s", name_.c_str());
    ros::NodeHandle nl(n);
    //point_cloud_sub_ = nl.subscribe<sensor_msgs::PointCloud2> ("/arti_robot/points2", 1, &ScanMatcher::PointCloudCallback, this);

    /*
    TESTING WITH POINCLOUD2 ROSBAG casual_walk.bag
    PointCloud2 data is published on /points_raw topic
    frame_id = velodyne
    */

    point_cloud_sub_ = nl.subscribe<sensor_msgs::PointCloud2> ("/points_raw", 1, &ScanMatcher::PointCloudCallback, this);
    return true;
}



bool ScanMatcher::CreatePublishers(const ros::NodeHandle& n) {
    ROS_INFO("ScanMatcher- CreatePublishers");
    ros::NodeHandle nl(n);
    odometry_pub_ = nl.advertise<nav_msgs::Odometry>("odometry", 10, false);
    pose_pub_ = nl.advertise<geometry_msgs::PoseStamped>("robot_pose", 10, false);
    return true;
}

void ScanMatcher::PublishOdometry(float x, float y, float z, ros::Time& timestamp) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = "lidar_odometry_frame";
    odom_msg.pose.pose.position.x = x;
    odom_msg.pose.pose.position.y = y;
    odom_msg.pose.pose.position.z = z;
    odometry_pub_.publish(odom_msg);    
}

void ScanMatcher::PublishPose(const tf::Pose& pose,const ros::Time& timestamp) {
     geometry_msgs::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "lidar_odometry_frame";
    pose_stamped.header.stamp = timestamp;

    pose_stamped.pose.orientation.x = pose.getRotation().getX();
    pose_stamped.pose.orientation.y = pose.getRotation().getY();
    pose_stamped.pose.orientation.z = pose.getRotation().getZ();
    pose_stamped.pose.orientation.w = pose.getRotation().getW();

    pose_stamped.pose.position.x = pose.getOrigin().getX();
    pose_stamped.pose.position.y = pose.getOrigin().getY();
    pose_stamped.pose.position.z = pose.getOrigin().getZ();

    pose_pub_.publish(pose_stamped);  
}

void ScanMatcher::PointCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
    //ROS_INFO("ScanMatcher - CameraCallback"); 

    //I need to CONVERT sensor_msgs::PointCloud2 to pcl::PointCloud<pcl::PointXYZ>
    
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg,pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
    std::vector<int> indices;

    //pcl::removeNaNFromPointCloud(*temp_cloud, *my_cloud, indices);

    if(!b_is_previous_stored_){
        previous_point_cloud_ = temp_cloud;
        b_is_previous_stored_ = true;
        return;
    }
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr current_point_cloud = temp_cloud;
    
    icp_.setInputSource(current_point_cloud);
    icp_.setInputTarget(previous_point_cloud_);
    icp_.align(final_);

    //std::cout << "has converged: " << icp_.hasConverged() << "  score: " <<
    //icp_.getFitnessScore() << std::endl;
    
    transformation_ = icp_.getFinalTransformation().cast<double>();
    tf::Pose pose;
    Eigen::Vector3d trans(transformation_.block<3, 1>(0, 3));
    Eigen::Quaterniond q(transformation_.block<3, 3>(0, 0));
    pose.setRotation(tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
    pose.setOrigin(tf::Vector3(trans.x(), trans.y(), trans.z()));

    PublishPose(pose, msg->header.stamp);
    
    previous_point_cloud_ = current_point_cloud;
}
    


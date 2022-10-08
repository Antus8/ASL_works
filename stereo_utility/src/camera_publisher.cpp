#include <CameraPublisher.h>

using namespace sensor_msgs;
using namespace cv;
using namespace cv_bridge;

CameraPublisher::CameraPublisher(std::string id){
    identifier_ = id;
    ROS_INFO("CameraPublisher Class Constructor");
}

CameraPublisher::~CameraPublisher() {
    ROS_INFO("CameraPublisher Class Destructor");
}

bool CameraPublisher::Initialize(const ros::NodeHandle& n) {
    ROS_INFO("CameraPublisher - Initialize");
    name_ = ros::names::append(n.getNamespace(), "CameraPublisher");
    if (!CreatePublishers(n)) {
        ROS_ERROR("Failed to create publishers in %s", name_.c_str());
        return false;
    }   
    return true;     
}

bool CameraPublisher::CreatePublishers(const ros::NodeHandle& n) {
    ROS_INFO("CameraPublisher - CreatePublishers");
    ros::NodeHandle nl(n);
    image_transport::ImageTransport it(nl);

    if(identifier_ == "left"){
        camera_name_ = "my_left_camera";
        filename_ = "/home/antonello/stereo_ws/src/utility_stereo_aggregator/videos/video_left.avi"; //left.mp4
        calib_file_path_ = "file:///home/antonello/stereo_ws/src/utility_stereo_aggregator/calibration/cam_left.yaml";
        publisher_ = it.advertise("left/image_raw", 1);
        cam_info_publisher_ = nl.advertise<sensor_msgs::CameraInfo>("left/camera_info", 1);
    }
    else {
        camera_name_ = "my_right_camera";
        filename_ = "/home/antonello/stereo_ws/src/utility_stereo_aggregator/videos/video_right.avi"; //right.mp4
        calib_file_path_ = "file:///home/antonello/stereo_ws/src/utility_stereo_aggregator/calibration/cam_right.yaml";
        publisher_ = it.advertise("right/image_raw", 1);
        cam_info_publisher_ = nl.advertise<sensor_msgs::CameraInfo>("right/camera_info", 1);
    }
    return true;
}

void CameraPublisher::Publish(const ros::NodeHandle& n){

    ros::Rate loop_rate(18);
    ros::NodeHandle nl(n);

    cap_.open(filename_);
    
    if(!cap_.isOpened()){
        ROS_ERROR("Error reading input stream");
        //return 1;
    }

    sensor_msgs::ImagePtr msg;
    camera_info_manager::CameraInfoManager cam_info(nl, camera_name_, calib_file_path_);

    time_between_frames = 1/cap_.get(cv::CAP_PROP_FPS);

    
    while (nl.ok()) {
        cap_ >> frame_;
        
        if(!frame_.empty()) {
        msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame_).toImageMsg();
        msg->header.stamp = ros::Time::now();
        msg->header.frame_id = "robot_base_link";

        cam_info_ = cam_info.getCameraInfo();
        cam_info_.header.stamp = msg->header.stamp;
        cam_info_.header.frame_id = msg->header.frame_id;

        std::cout << "Time: " << msg->header.stamp << std::endl;
        std::cout << "Time: " << cam_info_.header.stamp << std::endl;
        std::cout << "Id: " << msg->header.frame_id << std::endl;
        std::cout << "Id: " << cam_info_.header.frame_id << std::endl;
        

        cam_info_publisher_.publish(cam_info_);
        publisher_.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        }
        else{
            cap_.open(filename_);
            cap_ >> frame_;
        }
}
}


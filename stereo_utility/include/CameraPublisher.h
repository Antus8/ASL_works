#ifndef _CAMERA_PUBLISHER_H_
#define _CAMERA_PUBLISHER_H_

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



class CameraPublisher {

    public:    
                   
        CameraPublisher(std::string id);
        ~CameraPublisher();
        bool Initialize(const ros::NodeHandle& n);
        void Publish(const ros::NodeHandle& n);

    protected: 

        std::string name_;
        std::string camera_name_;
        std::string calib_file_path_;

        image_transport::Publisher publisher_; 
        ros::Publisher cam_info_publisher_;
        sensor_msgs::CameraInfo cam_info_;

        std::string identifier_;
        std::string filename_; 
        cv::VideoCapture cap_;
        cv::Mat frame_;

        bool CreatePublishers(const ros::NodeHandle& n);
        int time_between_frames;
};

#endif

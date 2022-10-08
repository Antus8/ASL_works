#include <CameraPublisher.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "camera_publisher");
  ros::NodeHandle n("~");

  CameraPublisher left_publisher_("left");    
  //CameraPublisher right_publisher_("right");
  if(!left_publisher_.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize one of the Publishers!",
              ros::this_node::getName().c_str());
  }
  else {
      left_publisher_.Publish(n);
      //right_publisher_.Publish(n);
  }

  ros::spin();
  return EXIT_SUCCESS;
} 

#include <ScanMatcher.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "scan_matcher");
  ros::NodeHandle n("~");

  ScanMatcher sm;    
  if(!sm.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize VisualOdometry.",
              ros::this_node::getName().c_str());
  }
  ros::spin();
  return EXIT_SUCCESS;
  
}

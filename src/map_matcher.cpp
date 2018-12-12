#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <glog/logging.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

namespace map_matcher {


class BranchAndBoundMatcher {

  public:
    explicit BranchAndBoundMatcher(ros::NodeHandle& n) : nh_(n) {
      // Topic names.
      const std::string kScanTopic = "scan";
      const std::string kCroppedScanTopic = "cropped_scan";

      // Services
      // TODO

      // TF_publisher
      // TODO

      nh_.getParam("acceptance_ratio", kAcceptanceRatio, 0.5);
      nh_.getParam("rotation_downsampling", kRotationDownsampling, 1);
    }
    ~BranchAndBoundMatcher() {}

  protected:
    /// \brief receives scan 1 messages
    void matchCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
      VLOG(3) << "scancallback";


      VLOG(3) << "";
      // publish result.
      match_debug_pub_.publish(TODO);
     }


  private:
    // ROS
    ros::NodeHandle& nh_;
    ros::Subscriber scan_sub_;
    ros::Publisher match_debug_pub_;
    // Params
    double kAcceptanceRatio;
    int kRotationDownsampling;

}; // class BranchAndBoundMatcher

} // namespace map_matcher

using namespace map_matcher;

int main(int argc, char **argv) {

  ros::init(argc, argv, "map_matcher");
  ros::NodeHandle n("~"); // private node handle (~ gets replaced with node name)
  BranchAndBoundMatcher bnb_matcher(n);

  try {
    ros::spin();
  }
  catch (const std::exception& e) {
    ROS_ERROR_STREAM("Exception: " << e.what());
    return 1;
  }
  catch (...) {
    ROS_ERROR_STREAM("Unknown Exception.");
    return 1;
  }

  return 0;
}


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

#include "map_matcher/SetReferenceMap.h"
#include "map_matcher/MatchToReference.h"

namespace map_matcher {

class Node {
  public:
    Node(int height, int angle_index, int pos_i, int pos_j) :
        h(height),
        a(angle_index),
        i(pos_i),
        j(pos_j) {
      score = -1;
    }
    ~Node() {}
    int h;
    int a;
    int i;
    int j;
    int score;
};

class BranchAndBoundMatcher {

  public:
    explicit BranchAndBoundMatcher(ros::NodeHandle& n) : nh_(n) {

      // Services
      // TODO
      nh_.advertiseService("set_reference_map", &BranchAndBoundMatcher::set_reference_map_service, this);
      nh_.advertiseService("match_to_reference", &BranchAndBoundMatcher::match_to_reference_service, this);

      // TF_publisher
      // TODO

      nh_.param("acceptance_ratio", kAcceptanceRatio, 0.5);
      nh_.param("rotation_downsampling", kRotationDownsampling, 1);
    }
    ~BranchAndBoundMatcher() {}

  protected:
    bool set_reference_map_service(map_matcher::SetReferenceMap::Request& req,
                                   map_matcher::SetReferenceMap::Response& res) {
      VLOG(3) << "";


      VLOG(3) << "";
     }

    bool match_to_reference_service(map_matcher::MatchToReference::Request& req,
                                    map_matcher::MatchToReference::Response& res) {
      VLOG(3) << "scancallback";


      VLOG(3) << "";
      // publish result.
//       match_debug_pub_.publish(TODO);
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


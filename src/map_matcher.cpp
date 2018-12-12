#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>

#include <glog/logging.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/OccupancyGrid.h>

#include "map_matcher/SetReferenceMap.h"
#include "map_matcher/MatchToReference.h"

namespace map_matcher {

typedef std::vector<int> Array2D;

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

class BranchAndBoundProblemDef {
  public:
    BranchAndBoundProblemDef() {}
    ~BranchAndBoundProblemDef() {}
}; // BranchAndBoundProblemDef

class BranchAndBoundMatcher {
  public:
    BranchAndBoundMatcher() {}
    BranchAndBoundMatcher(const float& acceptance_ratio,
                          const int& rotation_downsampling) :
        kAcceptanceRatio(acceptance_ratio),
        kRotationDownsampling(rotation_downsampling) {}
    ~BranchAndBoundMatcher() {}

    void setReferenceMap(const nav_msgs::OccupancyGrid& refmap) {
      refmap_ = refmap;
      // Recompute max fields
    }
    void match(const nav_msgs::OccupancyGrid& map) {
      // Initialize problem
    }

    const nav_msgs::OccupancyGrid& refmap() const { return refmap_; }

  private:
    float kAcceptanceRatio;
    int kRotationDownsampling;
    nav_msgs::OccupancyGrid refmap_;
    std::vector<Array2D> precomputed_max_fields_;

}; // class BranchAndBoundMatcher

class MapMatcherNode {

  public:
    MapMatcherNode(ros::NodeHandle& n) : nh_(n) {
      // Services
      // TODO
      nh_.advertiseService("set_reference_map", &MapMatcherNode::set_reference_map_service, this);
      nh_.advertiseService("match_to_reference", &MapMatcherNode::match_to_reference_service, this);

      // Params
      float acceptance_ratio;
      int rotation_downsampling;
      nh_.param<float>("acceptance_ratio", acceptance_ratio, 0.5);
      nh_.param("rotation_downsampling", rotation_downsampling, 1);

      // BranchAndBoundMatcher
      bnb_ = BranchAndBoundMatcher(acceptance_ratio, rotation_downsampling);
    }
    ~MapMatcherNode() {}

  protected:
    bool set_reference_map_service(map_matcher::SetReferenceMap::Request& req,
                                   map_matcher::SetReferenceMap::Response& res) {
      VLOG(3) << "";
      bnb_.setReferenceMap(req.reference_map);


      VLOG(3) << "";
     }

    bool match_to_reference_service(map_matcher::MatchToReference::Request& req,
                                    map_matcher::MatchToReference::Response& res) {
      VLOG(3) << "scancallback";
      bnb_.match(req.source_map);


      VLOG(3) << "";
      // publish result.
      tf::Transform transform; // TODO
      tf_br_.sendTransform(tf::StampedTransform(
        transform,
        ros::Time::now(),
        req.source_map.header.frame_id,
        bnb_.refmap().header.frame_id));
     }


  private:
    // ROS
    ros::NodeHandle& nh_;
    tf::TransformBroadcaster tf_br_;
    ros::Publisher match_debug_pub_;
    ros::Subscriber scan_sub_;

    // State
    BranchAndBoundMatcher bnb_;

}; // class MapMatcherNode


} // namespace map_matcher

using namespace map_matcher;

int main(int argc, char **argv) {

  ros::init(argc, argv, "map_matcher");
  ros::NodeHandle n("~"); // private node handle (~ gets replaced with node name)
  MapMatcherNode map_matcher(n);

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


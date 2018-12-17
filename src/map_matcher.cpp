#include <thread>
#include <chrono>
#include <string>
#include <vector>
#include <cmath>
#include <algorithm>
#include <mutex>
#include <functional>

#include <glog/logging.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/OccupancyGrid.h>

#include "map_matcher/SetReferenceMap.h"
#include "map_matcher/MatchToReference.h"

namespace map_matcher {

typedef int8_t ArrayType;
typedef std::vector<ArrayType> ArrayData;
typedef struct Hit{
  int i;
  int j;
} Hit;
typedef std::vector<Hit> Hits;

template <class T>
class Array2D {
  public:
    Array2D(const ArrayData& msg_data, const nav_msgs::MapMetaData& info) :
      Array2D(info.width, info.height)
    {
      // fill with msg_data
      for ( size_t i = 0; i < shape_i; i++ ) {
        for ( size_t j = 0; j < shape_j; j++ ) {
          data[i][j] = msg_data[i + j * shape_i];
        }
      }
    }
    Array2D(const size_t& size_i, const size_t& size_j) :
      shape_i(size_i), shape_j(size_j) {
      data = new ArrayType*[shape_i];
      for ( size_t i = 0; i < shape_i; i++ ) {
        data[i] = new ArrayType[shape_j];
        // fill with zeros
        for ( size_t j = 0; j < shape_j; j++ ) {
          data[i][j] = 0;
        }
      }
    }
    // Copy constructor
    Array2D(const Array2D& orig) : Array2D(orig.shape_i, orig.shape_j) {
      VLOG(3) << "COPY";
      for ( size_t i = 0; i < shape_i; i++ ) {
        for ( size_t j = 0; j < shape_j; j++ ) {
          data[i][j] = orig.data[i][j];
        }
      }
    }
    const Array2D& operator=(const Array2D& orig) {
      VLOG(3) << "ASSIGN";
      CHECK(orig.shape_i == shape_i);
      CHECK(orig.shape_j == shape_j);
      for ( size_t i = 0; i < shape_i; i++ ) {
        for ( size_t j = 0; j < shape_j; j++ ) {
          data[i][j] = orig.data[i][j];
        }
      }
    }
    ~Array2D() {
      for ( size_t i = 0; i < shape_i; i++ ) {
        delete[] data[i];
      }
      delete[] data;
    }
    ArrayType** data;
    size_t shape_i;
    size_t shape_j;
    T& at(const int& i, const int& j) {
      // chosen to get numpy indexing where also i -> x , j -> y from OccupancyGrid messages
      return data[i][j];
    }
    const T& at_c(const int& i, const int& j) const {
      // chosen to get numpy indexing where also i -> x , j -> y from OccupancyGrid messages
      return data[i][j];
    }
    Hits as_occupied_points_ij() const {
      constexpr T kThreshOccupied = 90;
      Hits result;
      for (size_t i = 0; i < shape_i; i++) {
        for (size_t j = 0; j < shape_j; j++) {
          if ( at_c(i, j) >= kThreshOccupied ) {
            // size_t to int conversion would fail for HUGE index values.
            result.push_back({static_cast<int>(i), static_cast<int>(j)});
          }
        }
      }
      return result;
    }
}; // class Array2D

inline const size_t shape_i(const nav_msgs::MapMetaData& info) { return info.width; };
inline const size_t shape_j(const nav_msgs::MapMetaData& info) { return info.height; };
// The BNB code should not use width and height, only i j and shapes.

Hits as_occupied_points_ij(const nav_msgs::OccupancyGrid& map) {
  return Array2D<ArrayType>(map.data, map.info).as_occupied_points_ij();
}
Hits rotate_hits_around_map_center(const Hits& hits, const float& theta,
    const nav_msgs::MapMetaData& info) {
  const float a = cos(theta);
  const float b = -sin(theta);
  const float c = -b;
  const float d = a;
  Hits rotated;
  for (size_t n = 0; n < hits.size(); n++ ) {
    Hit ij = hits[n];
    // translate
    float i_t = ( ij.i - shape_i(info) * 0.5 );
    float j_t = ( ij.j - shape_j(info) * 0.5 );
    // rotate
    float i_tr = i_t * a + j_t * b;
    float j_tr = i_t * c + j_t * d;
    // translate
    int i_trt = ( i_tr + shape_i(info) * 0.5 );
    int j_trt = ( j_tr + shape_j(info) * 0.5 );
    rotated.push_back({i_trt, j_trt});
  }
  return rotated;
}

class Node {
  public:
    Node(int height, size_t angle_index, int pos_i, int pos_j) :
        h(height),
        a(angle_index),
        i(pos_i),
        j(pos_j) {
      score = -1;
    }
    ~Node() {}
    int h;
    size_t a;
    int i;
    int j;
    int score;
}; // class Node
bool gt(Node a, Node b) { return (a.score < b.score); }
bool gt_angles(Node a, Node b, float th) { return (std::abs(a.a - th) > std::abs(b.a - th)); }

class BranchAndBoundSolution {
  public:
    BranchAndBoundSolution() : score(-1) {}
    ~BranchAndBoundSolution() {}
    int score;
    int pose_i;
    int pose_j;
    float theta;
}; // BranchAndBoundSolution

class BranchAndBoundProblemDef {
  public:
    BranchAndBoundProblemDef() {}
    ~BranchAndBoundProblemDef() {}
    int window_offset;
    int window_size_i;
    int window_size_j;
    int init_height;
    float dr;
    int n_rotations;
    std::vector<Hits> rotated_hits;
    std::vector<float> rotation_angles;

}; // BranchAndBoundProblemDef

class BranchAndBoundMatcher {
  public:
    BranchAndBoundMatcher() : ref_is_set_(false) {}
    BranchAndBoundMatcher(const float& acceptance_ratio,
                          const int& rotation_downsampling,
                          const nav_msgs::OccupancyGrid& refmap) :
        kAcceptanceRatio(acceptance_ratio),
        kRotationDownsampling(rotation_downsampling),
        refmap_(refmap),
        ref_is_set_(true) {
      precomputed_max_fields_.emplace_back(refmap.data, refmap.info);
      ROS_INFO_STREAM("Branch and Bound solver created.");
    }
    ~BranchAndBoundMatcher() {}

    const BranchAndBoundSolution match(const nav_msgs::OccupancyGrid& map,
        const float& theta_prior) {
      VLOG(3) << "";
      BranchAndBoundSolution result;
      if ( !ref_is_set_ ) {
        return result;
      }
      // Initialize problem
      ros::Time tic = ros::Time::now();
      BranchAndBoundProblemDef problem_def;
      // Window characteristics
      int half_length = std::max(shape_i(map.info), shape_j(map.info)) / 2;
      problem_def.window_offset = half_length;
      problem_def.window_size_i = shape_i(refmap_.info) + 2 * half_length;
      problem_def.window_size_j = shape_j(refmap_.info) + 2 * half_length;
      // Height of the first children nodes
      problem_def.init_height = std::ceil(std::log2(std::max(problem_def.window_size_i,
                                                             problem_def.window_size_j))) - 1;
      // Precompute the max fields up to this height
      precompute_max_fields(problem_def.init_height);
      // Rotational resolution
      problem_def.dr = kRotationDownsampling * 1. / sqrt(
          shape_i(map.info) * shape_i(map.info) +
          shape_j(map.info) * shape_j(map.info) );
      problem_def.n_rotations = 2. * M_PI / problem_def.dr - 1;
      // Rotate map accordingly
      Hits occupied_points = as_occupied_points_ij(map);
      ROS_INFO_STREAM("Creating rotations for " << occupied_points.size() << " hits.");
      for (size_t n = 0; n < problem_def.n_rotations; n++) {
        float th = problem_def.dr * n;
        problem_def.rotation_angles.push_back(th);
        problem_def.rotated_hits.push_back(
            rotate_hits_around_map_center(occupied_points, th, map.info));
      }
      ros::Time toc = ros::Time::now();
      ROS_INFO_STREAM("Problem initialization took " << toc - tic << " s");
      ROS_INFO_STREAM("  " << problem_def.window_size_i << "x" << problem_def.window_size_j << " search window");
      ROS_INFO_STREAM("  " << problem_def.n_rotations << " rotations");
      ROS_INFO_STREAM("  precomputed max fields up to height: " << problem_def.init_height);
      //

      // Initialize DFS state
      float best_score = kAcceptanceRatio * problem_def.rotated_hits[0].size();
      ROS_INFO_STREAM("Score threshold: " << best_score);
      Node best_leaf_node(-1, 0, 0, 0);
      std::vector<Node> node_list;


      // Expand rotation nodes
      tic = ros::Time::now();
      for (size_t n = 0; n < problem_def.n_rotations; n++) {
        Node rot_node(problem_def.init_height + 1, n, 0, 0);
        expand_node(rot_node, problem_def, node_list);
      }
      // Sort by closeness to theta_prior if given
      if ( theta_prior ) {
        std::sort(node_list.begin(), node_list.end(), std::bind(gt_angles,
            std::placeholders::_1, std::placeholders::_2, theta_prior));
      } else {
        std::sort(node_list.begin(), node_list.end(), gt);
      }
      toc = ros::Time::now();
      ROS_INFO_STREAM("Expanding rotation nodes: " << toc - tic << " s");

      // Depth-First Search Greedy
      tic = ros::Time::now();
      while ( !node_list.empty() ) {
        Node node = node_list.back();
        node_list.pop_back();
        if ( node.score < best_score ) {
          continue;
        }
        if ( node.h == 0 ) { // leaf node
          best_score = node.score;
          best_leaf_node = node;
          ROS_INFO_STREAM("New best score: " << best_score);
          continue;
        }
        // otherwise, promising inner node -> expand
        expand_node(node, problem_def, node_list);
      }
      toc = ros::Time::now();
      ROS_INFO_STREAM("DFS greedy: " << toc - tic << " s");

      // Output
      if ( best_leaf_node.h != -1 ) { // no solution found
        result.score = best_score;
        result.pose_i = best_leaf_node.i - problem_def.window_offset;
        result.pose_j = best_leaf_node.j - problem_def.window_offset;
        result.theta = best_leaf_node.a * problem_def.dr;
      }
      VLOG(3) << "";
      return result;
    }

    void expand_node(const Node& node, const BranchAndBoundProblemDef& problem_def,
        std::vector<Node>& node_list) const {
      // Split the node window into 4, of size 2^(node_height - 1)
      // if the window size is not a nice power of 2,
      // some children end up covering more of the window
      std::vector<Node> child_nodes;
      constexpr size_t N = 4;
      constexpr int i_offsets[N] = {0, 0, 1, 1};
      constexpr int j_offsets[N] = {0, 1, 0, 1};
      const int child_h = node.h - 1;
      const int w = pow(2, child_h);
      for (size_t n = 0; n < N; n++) {
        int i_offset = i_offsets[n];
        int j_offset = j_offsets[n];
        int child_i = node.i + ( w * i_offset );
        int child_j = node.j + ( w * j_offset );
        if ( child_i >= problem_def.window_size_i ||
             child_j >= problem_def.window_size_j ) {
          continue;
        }
        Node child_node(child_h, node.a, child_i, child_j);
        score_node(child_node, problem_def);
        child_nodes.push_back(child_node);
      }
      // sort by score
      std::sort(child_nodes.begin(), child_nodes.end(), gt);
      // append to node list
      node_list.insert(node_list.end(), child_nodes.begin(), child_nodes.end());
    }


    void score_node(Node& node, const BranchAndBoundProblemDef& problem_def) const {
      const auto& hits = problem_def.rotated_hits[node.a];
      const auto& field = precomputed_max_fields_[node.h];

      int score = 0;
      int o_f = pow(2, node.h) - 1; // max_field offset
      int o_w = problem_def.window_offset; // window offset
      // move hits to node_pos in window frame,
      // then coordinates from window frame to field frame
      int o = - o_w + o_f;
      int fs_i = field.shape_i;
      int fs_j = field.shape_j;
      for (size_t n = 0; n < hits.size(); n++) {
        const auto& ij = hits[n];
        // in occupancy frame
        int i = ij.i;
        int j = ij.j;
        // in field frame
        int i_f = (i + node.i + o);
        int j_f = (j + node.j + o);
        if ( i_f < 0 || j_f < 0 || i_f >= fs_i || j_f >= fs_j ){
          continue;
        }
        score += field.at_c(i_f, j_f);
      }
      node.score = score / 100;
    }

    void precompute_max_fields(const int& up_to_height) {
      int prev_size = precomputed_max_fields_.size();
      for (size_t h = 0; h <= up_to_height; h++) {
        // skip already computed fields
        if (h < prev_size) {
          continue;
        }
        // Compute field for height h
        const Array2D<ArrayType>& prev_field = precomputed_max_fields_.at(h-1);
        Array2D<ArrayType> new_field(shape_i(refmap_.info) + pow(2,h) - 1,
                                     shape_j(refmap_.info) + pow(2,h) - 1);
        int o_prev = pow(2, h-1);
        for (int i = 0; i < new_field.shape_i; i++) {
          for (int j = 0; j < new_field.shape_j; j++) {
            ArrayType max_val = 0;
            // equivalent i j coordinates in the previous map to correct for padding
            // i_previous = i - o_prev
            int i_p = i - o_prev;
            int j_p = j - o_prev;
            // coordinates for 4 samples in the prev_field
            constexpr size_t N = 4;
            int i_p_samples[N] = {i_p, i_p + o_prev, i_p         , i_p + o_prev};
            int j_p_samples[N] = {j_p, j_p         , j_p + o_prev, j_p + o_prev};
            for (size_t n = 0; n < N; n++) {
                int i_p_ = i_p_samples[n];
                int j_p_ = j_p_samples[n];
                if ( i_p_ < 0 ||
                     j_p_ < 0 ||
                     i_p_ >= prev_field.shape_i ||
                     j_p_ >= prev_field.shape_j ) {
                    continue;
                }
                const ArrayType val = prev_field.at_c(i_p_, j_p_);
                if ( val > max_val ) {
                  max_val = val;
                }
            }
            new_field.at(i,j) = max_val;
          }
        }
        // Push back
        precomputed_max_fields_.emplace_back(new_field);

      }
    }

    const nav_msgs::OccupancyGrid& refmap() const { return refmap_; }

  private:
    float kAcceptanceRatio;
    int kRotationDownsampling;
    nav_msgs::OccupancyGrid refmap_;
    std::vector<Array2D<ArrayType>> precomputed_max_fields_;
    bool ref_is_set_;

}; // class BranchAndBoundMatcher

class MapMatcherNode {

  public:
    MapMatcherNode(ros::NodeHandle& n) : nh_(n) {
      // Services
      setref_srv_ = nh_.advertiseService(
          "set_reference_map", &MapMatcherNode::set_reference_map_service, this);
      match_srv_ = nh_.advertiseService(
          "match_to_reference", &MapMatcherNode::match_to_reference_service, this);
    }
    ~MapMatcherNode() {}

  protected:
    bool set_reference_map_service(map_matcher::SetReferenceMap::Request& req,
                                   map_matcher::SetReferenceMap::Response& res) {
      VLOG(3) << "";
      // read params
      float acceptance_ratio;
      int rotation_downsampling;
      nh_.param<float>("acceptance_ratio", acceptance_ratio, 0.5);
      nh_.param("rotation_downsampling", rotation_downsampling, 1);
      // Initialize bnb matcher
      bnb_ = BranchAndBoundMatcher(acceptance_ratio, rotation_downsampling, req.reference_map);
      VLOG(3) << "";
      return true;
     }

    bool match_to_reference_service(map_matcher::MatchToReference::Request& req,
                                    map_matcher::MatchToReference::Response& res) {
      VLOG(3) << "matchcallback";
      const BranchAndBoundSolution solution = bnb_.match(req.source_map, req.theta_prior);
      res.found_valid_match = ( solution.score != -1 );
      res.i_source_origin_in_reference = solution.pose_i;
      res.j_source_origin_in_reference = solution.pose_j;
      res.theta = solution.theta;
      res.score = solution.score;
      VLOG(3) << "";
      return true;
     }


  private:
    // ROS
    ros::NodeHandle& nh_;
    tf::TransformBroadcaster tf_br_;
    ros::Publisher match_debug_pub_;
    ros::Subscriber scan_sub_;
    ros::ServiceServer setref_srv_;
    ros::ServiceServer match_srv_;

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

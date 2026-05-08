// #ifndef _ASTAR_H
// #define _ASTAR_H

// #include <Eigen/Eigen>
// #include <iostream>
// #include <map>
// #include <ros/console.h>
// #include <ros/ros.h>
// #include <string>
// #include <unordered_map>
// // #include "grad_spline/sdf_map.h"
// #include "plan_env/edt_environment.h"
// #include <boost/functional/hash.hpp>
// #include <queue>
// #include <path_searching/matrix_hash.h>
// namespace fast_planner {
// // #define REACH_HORIZON 1
// // #define REACH_END 2
// // #define NO_PATH 3
// #define IN_CLOSE_SET 'a'
// #define IN_OPEN_SET 'b'
// #define NOT_EXPAND 'c'
// #define inf 1 >> 30

// class Node {
// public:
//   /* -------------------- */
//   Eigen::Vector3i index;
//   Eigen::Vector3d position;
//   double g_score, f_score;
//   Node* parent;
//   char node_state;

//   double time;  // dyn
//   int time_idx;

//   /* -------------------- */
//   Node() {
//     parent = NULL;
//     node_state = NOT_EXPAND;
//   }
//   ~Node(){};
// };
// typedef Node* NodePtr;

// class NodeComparator0 {
// public:
//   bool operator()(NodePtr node1, NodePtr node2) {
//     return node1->f_score > node2->f_score;
//   }
// };

// // class NodeHashTable0 {
// // private:
// //   /* data */
// //   std::unordered_map<Eigen::Vector3i, NodePtr, matrix_hash0<Eigen::Vector3i>> data_3d_;
// //   std::unordered_map<Eigen::Vector4i, NodePtr, matrix_hash0<Eigen::Vector4i>> data_4d_;

// // public:
// //   NodeHashTable0(/* args */) {
// //   }
// //   ~NodeHashTable0() {
// //   }
// //   void insert(Eigen::Vector3i idx, NodePtr node) {
// //     data_3d_.insert(make_pair(idx, node));
// //   }
// //   void insert(Eigen::Vector3i idx, int time_idx, NodePtr node) {
// //     data_4d_.insert(make_pair(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx), node));
// //   }

// //   NodePtr find(Eigen::Vector3i idx) {
// //     auto iter = data_3d_.find(idx);
// //     return iter == data_3d_.end() ? NULL : iter->second;
// //   }
// //   NodePtr find(Eigen::Vector3i idx, int time_idx) {
// //     auto iter = data_4d_.find(Eigen::Vector4i(idx(0), idx(1), idx(2), time_idx));
// //     return iter == data_4d_.end() ? NULL : iter->second;
// //   }

// //   void clear() {
// //     data_3d_.clear();
// //     data_4d_.clear();
// //   }
// // };

// class Astar {
// private:
//   /* ---------- main data structure ---------- */
//   vector<NodePtr> path_node_pool_;
//   int use_node_num_, iter_num_;
//   NodeHashTable0 expanded_nodes_;
//   std::priority_queue<NodePtr, std::vector<NodePtr>, NodeComparator0> open_set_;
//   std::vector<NodePtr> path_nodes_;

//   /* ---------- record data ---------- */
//   EDTEnvironment::Ptr edt_environment_;
//   bool has_path_ = false;

//   /* ---------- parameter ---------- */
//   /* search */
//   double lambda_heu_;
//   double margin_;
//   int allocate_num_;
//   double tie_breaker_;
//   /* map */
//   double resolution_, inv_resolution_, time_resolution_, inv_time_resolution_;
//   Eigen::Vector3d origin_, map_size_3d_;
//   double time_origin_;

//   /* helper */
//   Eigen::Vector3i posToIndex(Eigen::Vector3d pt);
//   int timeToIndex(double time);
//   void retrievePath(NodePtr end_node);

//   /* heuristic function */
//   double getDiagHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
//   double getManhHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);
//   double getEuclHeu(Eigen::Vector3d x1, Eigen::Vector3d x2);

// public:
// class AstarNodeComparator {
//   public:
//     bool operator()(AstarNode::Ptr node1, AstarNode::Ptr node2) {
//       return node1->f_score > node2->f_score;
//     }
//   };

//   typedef shared_ptr<Astar> Ptr;
//   typedef shared_ptr<const Astar> ConstPtr;

//   enum class PROFILE { DEFAULT, COARSE, COARSE2, MEDIUM, FINE };
//   enum class MODE {
//     FREE_ONLY,
//     FREE_ONLY_BBOX,
//     FREE_UNKNOWN,
//     FREE_UNKNOWN_BBOX,
//     UNKNOWN_ONLY,
//     UNKNOWN_ONLY_BBOX
//   };
//   enum { REACH_END = 1, NO_PATH = 2 };

//   struct Config {
//     double resolution_, resolution_inverse_; // astar search step
//     double max_search_time_;                 // astar search time limit
//     double lambda_heuristic_;                // weight for heuristic
//     int allocate_num_;                       // number of nodes to allocate in the pool

//     double epsilon_, tie_breaker_;

//     // profile settings
//     double default_resolution_;
//     double default_max_search_time_;
//     double coarse_resolution_;
//     double coarse_max_search_time_;
//     double coarse2_resolution_;
//     double coarse2_max_search_time_;
//     double medium_resolution_;
//     double medium_max_search_time_;
//     double fine_resolution_;
//     double fine_max_search_time_;

//     // map settings
//     Eigen::Vector3d map_origin_;

//     bool verbose_;

//     void print() {
//       std::cout
//           << "|---------------------------------- Astar Config ----------------------------------|"
//           << std::endl;
//       std::cout << "Resolution: " << resolution_ << std::endl;
//       std::cout << "Max search time: " << max_search_time_ << std::endl;
//       std::cout << "Lambda heuristic: " << lambda_heuristic_ << std::endl;
//       std::cout << "Epsilon: " << epsilon_ << std::endl;
//       std::cout << "Tie breaker: " << tie_breaker_ << std::endl;
//       std::cout << "Default resolution: " << default_resolution_ << std::endl;
//       std::cout << "Default max search time: " << default_max_search_time_ << std::endl;
//       std::cout << "Coarse resolution: " << coarse_resolution_ << std::endl;
//       std::cout << "Coarse max search time: " << coarse_max_search_time_ << std::endl;
//       std::cout << "Medium resolution: " << medium_resolution_ << std::endl;
//       std::cout << "Medium max search time: " << medium_max_search_time_ << std::endl;
//       std::cout << "Fine resolution: " << fine_resolution_ << std::endl;
//       std::cout << "Fine max search time: " << fine_max_search_time_ << std::endl;
//       std::cout << "Map origin: " << map_origin_.transpose() << std::endl;
//       std::cout << "Verbose: " << verbose_ << std::endl;
//       std::cout
//           << "|---------------------------------------------------------------------------------|"
//           << std::endl;
//     }
//   };
//   Astar(){};
//   ~Astar();

//   enum { REACH_END = 1, NO_PATH = 2 };

//   /* main API */
//   void setParam(ros::NodeHandle& nh);
//   void init();
//   void reset();
//   int search(Eigen::Vector3d start_pt, Eigen::Vector3d end_pt, bool optimistic = true,
//       bool dynamic = false, double time_start = -1.0);
//   void setProfile(const PROFILE &p);
//   void setEnvironment(const EDTEnvironment::Ptr& env);
//   std::vector<Eigen::Vector3d> getPath();
//   std::vector<Eigen::Vector3d> getVisited();
// };

// }  // namespace fast_planner

// #endif
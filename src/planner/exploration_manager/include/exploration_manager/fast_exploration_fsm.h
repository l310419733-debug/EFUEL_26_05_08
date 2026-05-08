#ifndef _FAST_EXPLORATION_FSM_H_
#define _FAST_EXPLORATION_FSM_H_

#include <Eigen/Eigen>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Empty.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <exploration_manager/DroneState.h>
#include <exploration_manager/PairOpt.h>
#include <exploration_manager/PairOptResponse.h>
#include <bspline/Bspline.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Path.h>
#include <exploration_manager/faster_types.hpp>
#include <exploration_manager/termcolor.hpp>
#include "quadrotor_msgs/PositionCommand.h"
#include <active_perception/perception_utils.h>


#include <mutex>
#include <deque>

#include <algorithm>
#include <iostream>
#include <vector>
#include <memory>
#include <string>
#include <thread>

using Eigen::Vector3d;
using std::shared_ptr;
using std::string;
using std::unique_ptr;
using std::vector;

namespace fast_planner {
class FastPlannerManager;
class FastExplorationManager;
class PlanningVisualization;
struct FSMParam;
struct FSMData;

enum EXPL_STATE { INIT, WAIT_TRIGGER, YAWING, PLAN_TRAJ, PUB_TRAJ, EXEC_TRAJ, FINISH, IDLE };

class FastExplorationFSM {

public:
  FastExplorationFSM(/* args */) {
  }
  ~FastExplorationFSM() {
  }

  void init(ros::NodeHandle& nh);
  void cmdCallback();
  void new_thread();
  bool yawing = false;
  
  bool receive_traj_ = false;
  double last_yaw_;
  bool flightEnd = false;

  // int adaptive_t = 10;

  ros::Time plan_success_time;
  Eigen::Matrix3d R_loop;
  Eigen::Vector3d T_loop;
  bool isLoopCorrection = true;
  state new_state;
  state plan_state;
  std::mutex mtx_plan_;
  std::thread t1;

  double plan_cost_time = 0;
  int plan_loop_num = 0;

  std::deque<state> plan_;
  quadrotor_msgs::PositionCommand cmd;
  //
  double pos_gain[3] = {5.7, 5.7, 6.2};
  double vel_gain[3] = {3.4, 3.4, 4.0};
  ros::Publisher cmd_vis_pub, pos_cmd_pub, traj_pub;
  int traj_id_;
  vector<Eigen::Vector3d> traj_cmd_, traj_real_;
  //flight_time
  double flight_distance = 0;
  double flight_CtrCost = 0;
  double flight_start;
  double flight_time = 0;

  // ros::Time plan_success_time;

  void pgTVioCallback(geometry_msgs::Pose msg);
  bool newPathCallback(vector<state> path, int start_index);
  
  void drawCmd(const Eigen::Vector3d &pos, const Eigen::Vector3d &vec, const int &id, const Eigen::Vector4d &color);
  void drawFOV(const vector<Eigen::Vector3d> &list1, const vector<Eigen::Vector3d> &list2);
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:

  int adaptive_t = 10;
  double flight_finished;
  bool classic_;
  /* helper functions */
  int callExplorationPlanner();
  void transitState(EXPL_STATE new_state, string pos_call);
  void visualize(int content);
  void clearVisMarker();
  int getId();
  void findUnallocated(const vector<int>& actives, vector<int>& missed);

  /* ROS functions */
  void FSMCallback(const ros::TimerEvent& e);
  void safetyCallback(const ros::TimerEvent& e);
  void frontierCallback(const ros::TimerEvent& e);
  void triggerCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void odometryCallback(const nav_msgs::OdometryConstPtr& msg);
  // Swarm.
  //
  void droneKfEstimatePoseCallback(const geometry_msgs::PoseArray::ConstPtr& msg);
  //
  void droneStateTimerCallback(const ros::TimerEvent& e);
  void droneStateMsgCallback(const exploration_manager::DroneStateConstPtr& msg);
  void optTimerCallback(const ros::TimerEvent& e);
  void optMsgCallback(const exploration_manager::PairOptConstPtr& msg);
  void optResMsgCallback(const exploration_manager::PairOptResponseConstPtr& msg);
  void swarmTrajCallback(const bspline::BsplineConstPtr& msg);
  void swarmTrajTimerCallback(const ros::TimerEvent& e);

  /* planning utils */
  shared_ptr<FastPlannerManager> planner_manager_;
  shared_ptr<FastExplorationManager> expl_manager_;
  shared_ptr<PlanningVisualization> visualization_;

  shared_ptr<FSMParam> fp_;
  shared_ptr<FSMData> fd_;
  EXPL_STATE state_;

  /* ROS utils */
  ros::NodeHandle node_;
  ros::Timer exec_timer_, safety_timer_, vis_timer_, frontier_timer_, vis_timer, vis_all_timer_;
  ros::Subscriber trigger_sub_, odom_sub_, pg_T_vio_sub_;
  ros::Publisher replan_pub_, new_pub_, bspline_pub_;

  // Swarm state
  ros::Publisher drone_state_pub_, opt_pub_, opt_res_pub_, swarm_traj_pub_, grid_tour_pub_,
      hgrid_pub_, drone_state_est_pub_, drone_est_path_pub_, drone_est_track_pub_;
  ros::Subscriber drone_state_sub_, opt_sub_, opt_res_sub_, swarm_traj_sub_, drone_state_est_sub_;
  ros::Timer drone_state_timer_, opt_timer_, swarm_traj_timer_;
  // nav_msgs::Path path_msgs_;
  geometry_msgs::PoseArray pose_array;
  nav_msgs::Path path_msgs_;
  nav_msgs::Path track_msgs_;

  shared_ptr<PerceptionUtils> percep_utils_;
};

}  // namespace fast_planner

#endif
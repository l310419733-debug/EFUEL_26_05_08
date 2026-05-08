#include <active_perception/graph_node.h>
#include <path_searching/astar2.h>
#include <plan_env/sdf_map.h>
#include <plan_env/raycast.h>

namespace fast_planner {
// Static data
double ViewNode::vm_;
double ViewNode::am_;
double ViewNode::yd_;
double ViewNode::ydd_;
double ViewNode::w_dir_;
shared_ptr<Astar> ViewNode::astar_;
shared_ptr<RayCaster> ViewNode::caster_;
shared_ptr<SDFMap> ViewNode::map_;

// Graph node for viewpoints planning
ViewNode::ViewNode(const Vector3d& p, const double& y, const bool& smallFrontier) {
  pos_ = p;
  yaw_ = y;
  parent_ = nullptr;
  smallArea = smallFrontier;
  vel_.setZero();  // vel is zero by default, should be set explicitly
}

double ViewNode::costTo(const ViewNode::Ptr& node) {
  vector<Vector3d> path;
  double c = ViewNode::computeCost(pos_, node->pos_, yaw_, node->yaw_, vel_, yaw_dot_, path);
  // std::cout << "cost from " << id_ << " to " << node->id_ << " is: " << c << std::endl;
  return c;
}
//多用于两个视点之间连接
double ViewNode::searchPath(const Vector3d& p1, const Vector3d& p2, vector<Vector3d>& path) {
  // Try connect two points with straight line
  bool safe = true;
  Vector3i idx;
  caster_->input(p1, p2);
  while (caster_->nextId(idx)) {
    if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)|| map_->getOccupancy(idx) == SDFMap::UNKNOWN) {
      // map_->getOccupancy(idx) == SDFMap::UNKNOWN
      safe = false;
      break;
    }
  }
  if (safe) {
    path = { p1, p2 };
    return (p1 - p2).norm();
  }
  // Search a path using decreasing resolution
  vector<double> res = { 0.4 };
  for (int k = 0; k < res.size(); ++k) {
    astar_->reset();
    astar_->setResolution(res[k]);
    if (astar_->search(p1, p2, false) == Astar::REACH_END) {
      path = astar_->getPath();
      return astar_->pathLength(path);
    }
  }
  // Use Astar early termination cost as an estimate
  path = { p1, p2 };
  return 100;
}

double ViewNode::computeCost(const Vector3d& p1, const Vector3d& p2, const double& y1,
    const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {
  // Cost of position change
  Vector3d delta_p = p2 - p1;
  Vector3d dist(0.5, 0.5, 0);
  if(delta_p.norm() < 1e-6)return 0.0;
  
  double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

  // Consider velocity change
  if (v1.norm() > 1e-3) {
    Vector3d dir = (p2 - p1).normalized();
    Vector3d vdir = v1.normalized();
    // double diff = acos(vdir.dot(dir));
    double diff = vdir.dot(dir);
    // diff = std::clamp(diff, -1.0, 1.0);
    double angle_diff = acos(diff);
    pos_cost += w_dir_ * angle_diff;
  }

  // Cost of yaw change
  double yaw_diff = fabs(y2 - y1);
  yaw_diff = min(yaw_diff, 2 * M_PI - yaw_diff);
  double yaw_cost = yaw_diff / yd_;

  // Vector3d direction(cos(y2), sin(y2), 0.0);
  // Vector3d p3 = p2 + direction*dist;
  // caster_->input(p2, p3);
  // Vector3i idx;
  // bool is_occup = false;
  // while (caster_->nextId(idx)) {
  //   if (map_->getInflateOccupancy(idx) == 1 || !map_->isInBox(idx)) {
  //     // map_->getOccupancy(idx) == SDFMap::UNKNOWN
  //     is_occup = true;
  //     break;
  //   }
  // }
  // if(is_occup){
    
  // }
  // // return max(pos_cost, yaw_cost);
  return max(pos_cost, yaw_cost);
}

double ViewNode::computeCostfrontier(const Vector3d& p1, const Vector3d& p2, const double& y1,
  const double& y2, const Vector3d& v1, const double& yd1, vector<Vector3d>& path) {
// Cost of position change
Vector3d delta_p = p2 - p1;
if(delta_p.norm() < 1e-6)return 0.0;

double pos_cost = ViewNode::searchPath(p1, p2, path) / vm_;

// Consider velocity change
if (v1.norm() > 1e-3) {
  Vector3d dir = (p2 - p1).normalized();
  Vector3d vdir = v1.normalized();
  // double diff = acos(vdir.dot(dir));
  double diff = vdir.dot(dir);
  // diff = std::clamp(diff, -1.0, 1.0);
  double angle_diff = acos(diff);
  pos_cost += w_dir_ * angle_diff;
  // double vc = v1.dot(dir);
  // pos_cost += w_dir_ * pow(vm_ - fabs(vc), 2) / (2 * vm_ * am_);
  // if (vc < 0)
  //   pos_cost += w_dir_ * 2 * fabs(vc) / am_;
}

// Cost of yaw change
  double yaw_diff = fabs(y2 - y1);
  yaw_diff = min(yaw_diff, 2 * M_PI - yaw_diff);
  double yaw_cost = yaw_diff / yd_;

  // return max(pos_cost, yaw_cost);
  return max(pos_cost, yaw_cost);
}

}  // namespace fast_planner
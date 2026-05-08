#ifndef _HGRID_H_
#define _HGRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>

using Eigen::Vector3d;
using std::list;
using std::pair;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;

class RayCaster;

namespace fast_planner {

class EDTEnvironment;
class Astar;
class GridInfo;
class UniformGrid;

// struct GridInfo {};
// struct GridCell {
//   EIGEN_MAKE_ALIGNED_OPERATOR_NEW

//   enum class STATE { ACTIVE, EXPLORED, UNKNWON };
//   // The cell ID is also its address in the uniform grid vector
//   // The address is calculated as: id = x + y * num_cells_x + z * num_cells_x * num_cells_y
//   int id_;
//   STATE state_ = STATE::ACTIVE;

//   Position center_, center_free_;

//   // centers_free_: CCL centers of free voxels for each free subregions in the cell
//   // centers_free_active_: centers_free_ with frontier vps in the cell, used for global CP planning
//   // centers_unknown_: CCL centers of unknown voxels for each unknown subregions  in the cell
//   vector<Position> centers_free_, centers_free_active_, centers_unknown_, centers_unknown_active_;
//   vector<int> centers_free_active_idx_, centers_unknown_active_idx_;

//   Position bbox_min_, bbox_max_;
//   VoxelIndex bbox_min_idx_, bbox_max_idx_;
//   std::vector<int> nearby_cells_ids_;

//   int unknown_num_, free_num_;
//   int frontier_num_;

//   // Frontier information for the cell in uniform grid
//   std::vector<int> frontier_ids_;
//   std::vector<Position> frontier_viewpoints_;
//   std::vector<double> frontier_yaws_;

//   // Frontier information for centers_free_active_
//   // a cell in uniform grid may have multiple free subregions
//   // each subregion has its own CCL centers (free and unknown)
//   // each free CCL center has its own frontier information (might be empty or multiple)
//   // frontier_ids_mc_.size() == centers_free_active_.size()
//   std::vector<std::vector<int>> frontier_ids_mc_;
//   std::vector<std::vector<Position>> frontier_viewpoints_mc_;
//   std::vector<std::vector<double>> frontier_yaws_mc_;

//   // Connectivity matrixs for current cell CCL centers and nearby cells' CCL centers
//   // matrix size is (centers_free_.size(), nearby_cell_centers_free.size())
//   // map: nearby cell id -> connectivity matrix
//   std::map<int, std::vector<std::vector<bool>>> connectivity_matrixs_;

//   // Visualization vertices of each side of the grid cell
//   std::vector<Position> vertices1_, vertices2_;

//   void print() {
//     std::cout << "-------------------" << std::endl;
//     std::cout << "Grid cell id: " << id_ << std::endl;
//     std::cout << "Center: " << center_.transpose() << std::endl;
//     std::cout << "Bbox min: " << bbox_min_.transpose() << std::endl;
//     std::cout << "Bbox max: " << bbox_max_.transpose() << std::endl;
//     std::cout << "Unknown num: " << unknown_num_ << std::endl;
//     std::cout << "Frontier num: " << frontier_num_ << std::endl;
//     std::cout << "Frontier ids: ";
//     for (auto it = frontier_ids_.begin(); it != frontier_ids_.end(); ++it) {
//       std::cout << *it << " ";
//     }
//     std::cout << std::endl;
//     std::cout << "-------------------" << std::endl;
//   }
// };
// Hierarchical grid, contains two levels currently
class HGrid {

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  struct Config {
    int num_levels_;                // Number of levels in the hierarchical grid
    double cell_size_max_;          // Size of the largest grid (level 0)
    int cell_size_z_partition_num_; // Number of z partitioning for each grid
    bool use_2d_grids_;             // True if z partitioning is ignored
    bool verbose_;                  // True if print out debug information

    double min_unknown_num_scale_; // Scale of minimum unknown num w.r.t. cell numbers

    double unknown_penalty_factor_; // Penalty factor for unknown cells in astar cost calculation
    double hybrid_search_radius_;   // Radiusc cutoff for hybrid search
    int ccl_step_;                  // Step size for CCL
  };

  HGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh);
  ~HGrid();

  // Position getLayerCellCenters(const int &level, const int &cell_id, const int &center_id) {
  //   return uniform_grids_[level].getCellCenters(cell_id, center_id);
  // }
  // void getLayerCellCenters(const int &level, const int &cell_id, const int &center_id,
  //                          Position &center) {
  //   uniform_grids_[level].getCellCenters(cell_id, center_id, center);
  // }
  void updateGridData(const int& drone_id, vector<int>& grid_ids, bool reallocated,
      const vector<int>& last_grid_ids, vector<int>& first_ids, vector<int>& second_ids, const vector<Eigen::Vector3d>& positions);

  bool updateBaseCoor();
  void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
  void getFrontiersInGrid(const vector<int>& grid_ids, vector<int>& ftr_ids);
  
  
  void getCostMatrix(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<vector<int>>& first_ids,
      const vector<vector<int>>& second_ids, const vector<int>& grid_ids, Eigen::MatrixXd& mat,
      const vector<double>& next_yaws, const vector<Eigen::Vector3d>& start_yaws);
  void getGridTour(const vector<int>& ids, const Eigen::Vector3d& pos,
      vector<Eigen::Vector3d>& tour, vector<Eigen::Vector3d>& tour2);
  
  bool getNextGrid(const vector<int>& grid_ids, Eigen::Vector3d& grid_pos, double& grid_yaw);
  void getConsistentGrid(const vector<int>& last_ids, const vector<int>& cur_ids,
      vector<int>& first_ids, vector<int>& second_ids);
  void ouputGridAccesstime(vector<int>& access_times);
  void getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2);
  void getGridMarker2(vector<Eigen::Vector3d>& pts, vector<std::string>& texts);
  void checkFirstGrid(const int& id);
  int getUnknownCellsNum(const int& grid_id);
  Eigen::Vector3d getCenter(const int& grid_id);
  void getActiveGrids(vector<int>& grid_ids);
  bool isConsistent(const int& id1, const int& id2);
  double getCostDroneToGrid(
      const Eigen::Vector3d& pos, const int& grid_id, const vector<int>& first, const Eigen::Vector3d& velocity,
      const double next_yaw, const Eigen::Vector3d start_yaw);
  double getCostGridToGrid(const int& id1, const int& id2, const vector<vector<int>>& firsts,
      const vector<vector<int>>& seconds, const int& drone_num);
  // double getConsistentGridcost(const Eigen::Vector3d& velocity, const Eigen::Vector3d& position, const Eigen::Vector3d& yaw, );    
  unique_ptr<Astar> path_finder_;
  void activateGridForShort(const std::vector<Eigen::Vector3d>& path, const int& drone_id);

private:
  void coarseToFineId(const int& coarse, vector<int>& fines);
  void fineToCoarseId(const int& fine, int& coarse);
  GridInfo& getGrid(const int& id);

  bool isClose(const int& id1, const int& id2);
  bool inSameLevel1(const int& id1, const int& id2);
  vector<Eigen::Vector3d> generateSamplingPoints(
    std::vector<Eigen::Vector3d>& path, double interval);
  
  unique_ptr<UniformGrid> grid1_;  // Coarse level
  unique_ptr<UniformGrid> grid2_;  // Fine level
  
  unique_ptr<UniformGrid> uniform_grids_;
  shared_ptr<EDTEnvironment> edt_;
  double consistent_cost_;
  double consistent_cost2_;
  // unordered_map<pair<Eigen::Vector3d, Eigen::Vector3d>, double> patch_cache_1_;
  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;
  double w_first_;
};

}  // namespace fast_planner
#endif
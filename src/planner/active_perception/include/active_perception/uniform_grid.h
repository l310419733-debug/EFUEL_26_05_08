#ifndef _UNIFORM_GRID_H_
#define _UNIFORM_GRID_H_

#include <ros/ros.h>
#include <Eigen/Eigen>
#include <memory>
#include <vector>
#include <list>
#include <unordered_map>
#include <utility>
#include <active_perception/hgrid.h>

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
class HGrid;

// struct GridInfo {};

class GridInfo {
public:
  GridInfo() {
  }
  ~GridInfo() {
  }

  int unknown_num_;
  int frontier_num_;
  int access_times_;
  int grid_id_;
  Eigen::Vector3d center_;
  unordered_map<int, int> frontier_cell_nums_;
  unordered_map<int, int> contained_frontier_ids_;
  bool is_updated_;
  bool need_divide_, active_;
  int explored_by_;
  int expl_comple_;
  double last_access_time;
  
  bool is_prev_relevant_;
  bool is_cur_relevant_;
  bool is_passable_;
  double last_accessed_time_;

  // Vertices and their box in xy plane, in current drone's frame
  Eigen::Vector3d vmin_, vmax_;
  vector<Eigen::Vector3d> vertices_;

  // Normals of separating lines in xy plane, associated with vertices_
  vector<Eigen::Vector3d> normals_;
};

class UniformGrid {

public:
  // EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // typedef std::share_ptr<UniformGrid> Ptr;
  // typedef std::shared_ptr<const UniformGrid> ConstPtr;
  // struct Config {
  //   int level_;
  //   int num_cells_x_, num_cells_y_, num_cells_z_, num_cells_;
  //   Eigen::Vector3i num_voxels_per_cell_; // Number of voxels in each cell, x, y, z num may differ
  //   Eigen::Vector3d cell_size_;           // Size of each grid cell
  //   Position bbox_min_, bbox_max_;        // Bounding box of the grid
  //   double map_resolution_;               // Resolution of the map (m/voxel)
  //   bool use_2d_grids_;                   // True if z partitioning is ignored
  //   bool verbose_;                        // True if print out debug information

  //   // Cost matrix calculation parameters
  //   double consistency_gain_;

  //   int min_unknown_num_; // Minimum number of unknown voxels in an active cell

  //   double unknown_penalty_factor_;

  //   double hybrid_search_radius_;

  //   int ccl_step_;

  //   double epsilon_ = 1e-4;

  //   void print() {
  //     std::cout << "-------------------" << std::endl;
  //     std::cout << "Uniform grid level: " << level_ << std::endl;
  //     std::cout << "Number of cells: " << num_cells_ << std::endl;
  //     std::cout << "Number of cells in x: " << num_cells_x_ << std::endl;
  //     std::cout << "Number of cells in y: " << num_cells_y_ << std::endl;
  //     std::cout << "Number of cells in z: " << num_cells_z_ << std::endl;
  //     std::cout << "Number of voxels per cell: " << num_voxels_per_cell_.transpose() << std::endl;
  //     std::cout << "Cell size: " << cell_size_.transpose() << std::endl;
  //     std::cout << "Bounding box min: " << bbox_min_.transpose() << std::endl;
  //     std::cout << "Bounding box max: " << bbox_max_.transpose() << std::endl;
  //     std::cout << "Map resolution: " << map_resolution_ << std::endl;
  //     std::cout << "Use 2d grids: " << use_2d_grids_ << std::endl;
  //     std::cout << "Verbose: " << verbose_ << std::endl;
  //     std::cout << "Consistency gain: " << consistency_gain_ << std::endl;
  //     std::cout << "Minimum unknown num: " << min_unknown_num_ << std::endl;
  //     std::cout << "Unknown penalty factor: " << unknown_penalty_factor_ << std::endl;
  //     std::cout << "Epsilon: " << epsilon_ << std::endl;
  //     std::cout << "-------------------" << std::endl;
  //   }
  // };

  // enum class CENTER_TYPE { UNKNOWN, FREE };
  
  
  UniformGrid(const shared_ptr<EDTEnvironment>& edt, ros::NodeHandle& nh, const int& level);
  ~UniformGrid();

  void initGridData();
  void updateBaseCoor();
  void updateGridData(const int& drone_id, vector<int>& grid_ids, vector<int>& parti_ids,
      vector<int>& parti_ids_all, const vector<Eigen::Vector3d>& positions);
  void activateGrids(const vector<int>& ids);
  
  // void inputFrontiers(const std::vector<Position> &frontier_viewpoints,
  //   const std::vector<double> &frontier_yaws);
  // void calculateCostMatrixSingleThread(
  //     const Position &cur_pos, const Eigen::Vector3d &cur_vel, Eigen::MatrixXd &cost_matrix,
  //     std::map<int, std::pair<int, int>> &cost_mat_id_to_cell_center_id);
  void inputFrontiers(const vector<Eigen::Vector3d>& avgs);
  void getCostMatrix(const vector<Eigen::Vector3d>& positions,
      const vector<Eigen::Vector3d>& velocities, const vector<int>& prev_first_grid,
      const vector<int>& grid_ids, Eigen::MatrixXd& mat);
  void getGridTour(const vector<int>& ids, vector<Eigen::Vector3d>& tour);
  void getFrontiersInGrid(const int& grid_id, vector<int>& ftr_ids);
  void getGridMarker(vector<Eigen::Vector3d>& pts1, vector<Eigen::Vector3d>& pts2);
  void activateGridForShort(const vector<Eigen::Vector3d>& path);
private:
  // Config config_;
  // voxel_mapping::MapServer::Ptr map_server_;

  // std::vector<GridCell> uniform_grid_;
  // std::vector<int> active_cell_ids_;
  // std::unordered_set<int> explored_cell_ids_;
  // std::vector<int> cell_ids_need_update_frontier_;

  // // Cost matrix for all cells
  // std::vector<std::vector<double>> cost_matrix_;
  // //
  // // std::vector<std::vector<std::unordered_set<int>>> ccl_voxels_addr_;
  // std::vector<std::vector<Eigen::Vector3d>> ccl_voxels_color_;
  // std::vector<std::vector<std::pair<int, int>>> ccl_free_unknown_states_and_centers_idx_;

  // Connectivity graph
  // ConnectivityGraph::Ptr connectivity_graph_;

  // double space_decomp_time_, connectivity_graph_time_;

  void updateGridInfo(const Eigen::Vector3i& id);
  // std::vector<Eigen::Vector3d> generateSamplingPoints
  int toAddress(const Eigen::Vector3i& id);
  void adrToIndex(const int& adr, Eigen::Vector3i& idx);
  void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id);
  void indexToPos(const Eigen::Vector3i& id, const double& inc, Eigen::Vector3d& pos);
  bool insideGrid(const Eigen::Vector3i& id);
  bool isRelevant(const GridInfo& grid);

  shared_ptr<EDTEnvironment> edt_;
  unique_ptr<Astar> path_finder_;
  vector<GridInfo> grid_data_;

  vector<int> relevant_id_;
  unordered_map<int, int> relevant_map_;
  bool initialized_;
  vector<int> extra_ids_;

  Eigen::Vector3d resolution_;
  Eigen::Vector3d min_, max_;
  Eigen::Vector3i grid_num_;
  int level_;

  int min_unknown_, min_frontier_, min_free_;
  double consistent_cost_, inside_ratio_;
  double w_unknown_;

  // Swarm tf
  Eigen::Matrix3d rot_sw_;
  Eigen::Vector3d trans_sw_;
  bool use_swarm_tf_;

  friend HGrid;
};

}  // namespace fast_planner
#endif
#ifndef _GRAPH_SEARCH_H_
#define _GRAPH_SEARCH_H_

#include <vector>
#include <unordered_map>
#include <queue>
#include <list>
#include <memory>
#include <iostream>
#include <math.h>
#include <algorithm>
#include <Eigen/Eigen>

using Eigen::Vector3d;
using std::cout;
using std::list;
using std::priority_queue;
using std::queue;
using std::shared_ptr;
using std::unique_ptr;
using std::unordered_map;
using std::vector;
using std::numeric_limits;

namespace fast_planner {
// GraphSearch that operates on different types of node using Dijkstra algorithm
template <typename NodeT>
class GraphSearch {
public:
  GraphSearch() {
    node_num_ = 0;
    edge_num_ = 0;
  }
  ~GraphSearch() {
  }

  void print();
  void addNode(const shared_ptr<NodeT>& node);
  void addEdge(const int& from, const int& to);
  void DijkstraSearch(const int& start, const int& goal, vector<shared_ptr<NodeT>>& path);
  double heuristicForward(const shared_ptr<NodeT>& a, const shared_ptr<NodeT>& b) {
    return (a->pos_ - b->pos_).norm(); // 欧式距离
  }
  double heuristicBackward(const shared_ptr<NodeT>& a, const shared_ptr<NodeT>& b) {
    return (a->pos_ - b->pos_).norm(); // 可设计为对称或非对称
  }
  void BidirectionAstarSearch(const int& start, const int& goal, vector<shared_ptr<NodeT>>& path);
private:
  vector<shared_ptr<NodeT>> nodes_;
  int node_num_;
  int edge_num_;
};

template <typename NodeT>
class NodeCompare {
public:
  bool operator()(const shared_ptr<NodeT>& node1, const shared_ptr<NodeT>& node2) {
    return node1->g_value_ > node2->g_value_;
  }
};

template <typename NodeT>
void GraphSearch<NodeT>::print() {
  for (auto v : nodes_) {
    v->print();
    v->printNeighbors();
  }
}

template <typename NodeT>
void GraphSearch<NodeT>::addNode(const shared_ptr<NodeT>& node) {
  nodes_.push_back(node);
  nodes_.back()->id_ = node_num_++;
}

template <typename NodeT>
void GraphSearch<NodeT>::addEdge(const int& from, const int& to) {
  nodes_[from]->neighbors_.push_back(nodes_[to]);
  ++edge_num_;
}

template <typename NodeT>
void GraphSearch<NodeT>::BidirectionAstarSearch(
  const int& start, const int& goal, vector<shared_ptr<NodeT>>& path
){
  priority_queue<shared_ptr<NodeT>, vector<shared_ptr<NodeT>>, NodeCompare<NodeT>> open_forward, open_backward;
  unordered_map<int, bool> closed_forward, closed_backward;

  shared_ptr<NodeT> start_node = nodes_[start];
  shared_ptr<NodeT> goal_node = nodes_[goal];
  start_node->g_forward_ = 0.0;
  start_node->h_forward_ = heuristicForward(start_node, goal_node);
  open_forward.push(start_node);

  goal_node->g_backward_ = 0.0;
  goal_node->h_backward_ = heuristicBackward(goal_node, start_node);
  open_backward.push(goal_node);

  shared_ptr<NodeT> meet_node = nullptr;
  double min_total_cost = numeric_limits<double>::max();

  while(!open_forward.empty() && !open_backward.empty()){

    if(!open_forward.empty()){
      auto current_forward = open_forward.top();
      open_forward.pop();
      closed_forward[current_forward->id_] = true;

      if(closed_backward.find(current_forward->id_) != closed_backward.end()){
        double total_cost = current_forward->g_forward_ + nodes_[current_forward->id_]->g_backward_;

        if(total_cost < min_total_cost){
          min_total_cost = total_cost;
          meet_node = current_forward;
        }
      }

    for(auto neighbor : current_forward->neighbors_){
      if(closed_forward[neighbor->id_])continue;

      double tentative_g = current_forward->g_forward_ + current_forward->costTo(neighbor);
      if(tentative_g < neighbor->g_forward_){
        neighbor->g_forward_ = tentative_g;
        neighbor->parent_forward_ = current_forward;
        neighbor->h_forward_ = heuristicForward(neighbor, goal_node);
        open_forward.push(neighbor);
      }
    }
  }

  if(!open_backward.empty()){
    auto current_backward = open_backward.top();
    open_backward.pop();
    closed_backward[current_backward->id_] = true;

    if(closed_forward.find(current_backward->id_) != closed_forward.end()){
      double total_cost = current_backward->g_backward_ + nodes_[current_backward->id_]->g_forward_;
      if(total_cost < min_total_cost){
        min_total_cost = total_cost;
        meet_node = nodes_[current_backward->id_];
      }
    }
    
    for(auto neighbor : current_backward->neighbors_){
      if(closed_backward[neighbor->id_])continue;

      double tentative_g = current_backward->g_backward_ + neighbor->costTo(current_backward);
      if(tentative_g < neighbor ->g_backward_){
        neighbor->g_backward_ = tentative_g;
        neighbor->parent_backward_ =current_backward;
        neighbor->h_backward_ =heuristicBackward(neighbor, start_node);
        open_backward.push(neighbor);
      }
    }
  }

  if(meet_node != nullptr &&
    (open_forward.top()->g_forward_ + open_forward.top()->h_forward_ > min_total_cost)&&
    (open_forward.top()->g_backward_ + open_backward.top()->h_backward_> min_total_cost)){

      break;
    }
  }

  if(meet_node != nullptr){
    shared_ptr<NodeT> node =meet_node;
    while(node != nullptr){
      path.push_back(node);
      node = node->parent_forward_;
    }
    reverse(path.begin(), path.end());

    node = meet_node->parent_backward_;
    while(node != nullptr){
      path.push_back(node);
      node = node->parent_backward_;
    }
  }


}

template <typename NodeT>
void GraphSearch<NodeT>::DijkstraSearch(
    const int& start, const int& goal, vector<shared_ptr<NodeT>>& path) {
  // std::cout << "Node: " << node_num_ << ", edge: " << edge_num_ << std::endl;
  // Basic structure used by Dijkstra
  // unordered_map<int, int> close_set;
  priority_queue<shared_ptr<NodeT>, vector<shared_ptr<NodeT>>, NodeCompare<NodeT>> open_set;

  shared_ptr<NodeT> start_v = nodes_[start];
  shared_ptr<NodeT> end_v = nodes_[goal];
  start_v->g_value_ = 0.0;
  open_set.push(start_v);

  while (!open_set.empty()) {
    auto vc = open_set.top();
    open_set.pop();
    vc->closed_ = true;
    // close_set[vc->id_] = 1;

    // Check if reach target
    if (vc == end_v) {
      // std::cout << "Dijkstra reach target ";
      shared_ptr<NodeT> vit = vc;
      while (vit != nullptr) {
        path.push_back(vit);
        vit = vit->parent_;
      }
      reverse(path.begin(), path.end());
      return;
    }

    for (auto vb : vc->neighbors_) {
      // Check if in close set
      if (vb->closed_) continue;

      // Add new node or updated node in open set
      double g_tmp = vc->g_value_ + vc->costTo(vb);
      if (g_tmp < vb->g_value_) {
        vb->g_value_ = g_tmp;
        vb->parent_ = vc;
        open_set.push(vb);
      }
    }
  }
}
}  // namespace fast_planner

#endif
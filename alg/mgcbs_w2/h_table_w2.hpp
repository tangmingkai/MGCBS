#pragma once
#include <algorithm>
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/functional/hash.hpp>
#include <boost/heap/d_ary_heap.hpp>

#include "base/interval.hpp"
namespace mg_cbs_w2 {
struct HeuristicState {
  HeuristicState() = default;
  HeuristicState(int _goal_state, int _target_id)
      : goal_state(_goal_state), target_id(_target_id) {}
  int goal_state;
  int target_id;
  bool operator==(const HeuristicState &other) const {
    return goal_state == other.goal_state && target_id == other.target_id;
  }
};
};  // namespace mg_cbs_w2

template <>
struct std::hash<mg_cbs_w2::HeuristicState> {
  size_t operator()(const mg_cbs_w2::HeuristicState &x) const {
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<int>()(x.goal_state));
    boost::hash_combine(seed, std::hash<int>()(x.target_id));
    return seed;
  }
};



namespace mg_cbs_w2 {
class HTable {
 public:
  void Build(
      const std::vector<std::vector<std::vector<int>>> &full_distance_matrix,
      const std::vector<std::vector<int>> &target_distance_matrix) {
    full_distance_matrix_ = full_distance_matrix;
    Build_MST(target_distance_matrix);
  }
  int GetHeuristicMid(int state, int target_id) const {
    return GetHeuristic_MST(state, target_id);
  }

  const std::vector<std::vector<int>> &GetFullDistanceMatrix(
      int target_id) const {
    return full_distance_matrix_[target_id];
  }

 private:
  void Build_MST(const std::vector<std::vector<int>> &distance_matrix) {
    distance_matrix_ = distance_matrix;
    buffer_.clear();
  }

  int GetHeuristic_MST(int state, int target_id) const {
    auto iter = buffer_.find(HeuristicState(state, target_id));
    if (iter != buffer_.end()) return iter->second;
    int res = 0;
    int goal_num = distance_matrix_.size() - 1;
    std::vector<int> id2vid;
    if (target_id >= 0) id2vid.emplace_back(target_id);
    for (int i = 0; i < goal_num; i++) {
      if (((state) & (1 << i)) != 0) continue;
      id2vid.emplace_back(i + 1);
    }
    if (id2vid.size() <= 1) {
      buffer_[HeuristicState(state, target_id)] = 0;
      return 0;
    }
    std::vector<int> c(id2vid.size(), Interval::oo);
    std::vector<bool> visited(id2vid.size(), false);

    boost::heap::d_ary_heap<MSTNode, boost::heap::arity<2>,
                            boost::heap::mutable_<true>>
        open;
    std::vector<
        boost::heap::d_ary_heap<MSTNode, boost::heap::arity<2>,
                                boost::heap::mutable_<true>>::handle_type>
        handle(c.size());
    c[0] = 0;
    for (uint i = 0; i < c.size(); i++) {
      MSTNode node(c[i], i);
      handle[i] = open.push(node);
    }
    while (!open.empty()) {
      MSTNode n = open.top();
      open.pop();
      int min_c = n.cost();
      int min_id = n.id();
      res += min_c;
      visited[min_id] = true;
      for (uint j = 0; j < c.size(); j++) {
        if (visited[j] == false &&
            c[j] > distance_matrix_[id2vid[min_id]][id2vid[j]]) {
          c[j] = distance_matrix_[id2vid[min_id]][id2vid[j]];
          (*handle[j]).set_cost(c[j]);
          open.update(handle[j]);
        }
      }
    }
    buffer_[HeuristicState(state, target_id)] = res;
    return res;
  }

  class MSTNode {
   public:
    MSTNode(int cost, int id) : cost_(cost), id_(id) {}
    bool operator<(const MSTNode &n) const { return cost_ > n.cost(); }
    int cost() const { return cost_; }
    int id() const { return id_; }
    void set_cost(int cost) { cost_ = cost; }

   private:
    int cost_;
    int id_;
  };
  mutable std::unordered_map<HeuristicState, int> buffer_;
  // table_[i][j]
  // i: a binary number for the state. 0 means the goal haven't been reached. 1
  // means the goal have been reached. The goal of agent staying in is 1. j: the
  // agent is in goal j.
  std::vector<std::vector<int>> table_;

  // For MST
  // distance_matrix_[i][j]: Distance from target i to target j
  std::vector<std::vector<int>> distance_matrix_;
  // full_distance_matrix[i][x][y]: The distance from (x,y) to
  //                                agent.targets[i]
  std::vector<std::vector<std::vector<int>>> full_distance_matrix_;
};
};  // namespace mg_cbs_w2

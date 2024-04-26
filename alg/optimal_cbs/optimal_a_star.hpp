#pragma once
#include <algorithm>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <iostream>
#include "base/contraints.hpp"
#include "base/location.hpp"
#include "base/ts_state.hpp"
#include "alg/optimal_cbs/h_table_o.hpp"

struct OState {
  OState() = default;
  OState(const TSState &_ts_state, const int &_goal_state)
      : ts_state(_ts_state), goal_state(_goal_state) {}
  TSState ts_state;
  int goal_state;
  friend std::ostream &operator<<(std::ostream &os, const OState &c) {
    os << "(" << c.ts_state.time << ", " << c.ts_state.x << ", "
              << c.ts_state.y << ": " << c.goal_state << ")";
    return os;
  }
  bool operator==(const OState& other) const {
    return goal_state == other.goal_state && ts_state == other.ts_state;
  }
};

namespace std {
template <>
struct hash<OState> {
  size_t operator()(const OState& s) const {
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<TSState>()(s.ts_state));
    boost::hash_combine(seed, s.goal_state);
    return seed;
  }
};
}  // namespace std

namespace optimal_cbs {
class OptimalAStar {
 public:
  struct ONode {
    ONode(const OState &_o_state, const int &g, const int &f)
        : o_state(_o_state), g(g), f(f) {}
    OState o_state;
    bool operator<(const ONode &n) const {
      if (f != n.f) return f > n.f;
      return g < n.g;
    }

    friend std::ostream &operator<<(std::ostream &os, const ONode &c) {
      os << c.o_state << " " << c.f << "=" << c.g << "+" << (c.f - c.g)
         << std::endl;
      return os;
    }
    int g;
    int f;
  };

  explicit OptimalAStar(const Environment &env) : env_(env) {}
  bool Search(
      const Agent &agent, const HTable &h_table,
      const Constraints &cons,
      const std::vector<std::vector<std::vector<int>>> &full_distance_matrix,
      PlanResult *plan_result) {
    TSState start_ts_state = TSState(0, agent.start.x, agent.start.y);
    if (cons.IsRestricted(start_ts_state)) {
      return false;
    }

    OpenSet open_set;
    StateToHeap state_to_heap;
    ClosedSet closed_set;
    ComeFrom come_from;
    int init_goal_state = 0;

    int min_dis = Interval::oo;
    for (uint i = 0; i < agent.goals.size(); i++) {
      int dis = full_distance_matrix[i][start_ts_state.x][start_ts_state.y];
      if (dis == 0) init_goal_state |= 1 << i;
      min_dis = std::min(min_dis, dis);
    }

    auto start_o_state = OState(start_ts_state, init_goal_state);

    int heuristic = min_dis + h_table.GetHeuristicMid(0, -1);

    auto handle = open_set.push(ONode(start_o_state, 0, heuristic));
    state_to_heap[start_o_state] = handle;
    std::vector<OState> neighbors;
    neighbors.reserve(10);
    std::vector<int> latest_vc_time(agent.goals.size(), -1);

    for (uint i = 0; i < agent.goals.size(); i++) {
      auto iter_vc = cons.vertex_constraints.find(agent.goals[i]);
      if (iter_vc != cons.vertex_constraints.end()) {
        for (auto &vc : iter_vc->second) {
          latest_vc_time[i] = std::max(latest_vc_time[i], vc.time);
        }
      }
    }
    int final_goal_state = (1 << agent.goals.size()) - 1;
    while (!open_set.empty()) {
      ONode current = open_set.top();
      if ((current.o_state.goal_state == final_goal_state)) {
        bool is_can_stop = false;
        auto &ts_state = current.o_state.ts_state;
        for (uint i = 0; i < agent.goals.size(); i++) {
          if ((ts_state.x == agent.goals[i].x) &&
              (ts_state.y == agent.goals[i].y) &&
              (ts_state.time > latest_vc_time[i])) {
            is_can_stop = true;
          }
        }
        if (is_can_stop) {
          plan_result->states.clear();
          plan_result->states.push_back(current.o_state.ts_state);
          auto iter = come_from.find(current.o_state);
          while (iter != come_from.end()) {
            plan_result->states.push_back(iter->second.ts_state);
            iter = come_from.find(iter->second);
          }
          std::reverse(plan_result->states.begin(), plan_result->states.end());
          plan_result->cost = plan_result->states.size() - 1;
          bool is_pass_check = true;
          if (cons.IsRestricted(plan_result->states[0])) is_pass_check = false;
          for (uint i = 1; i < plan_result->states.size(); i++)
            if (cons.IsRestricted(plan_result->states[i - 1],
                                  plan_result->states[i])) {
              is_pass_check = false;
            }
          if (!is_pass_check) {
            puts("gg");
            exit(0);
          }
          return true;
        }
      }

      open_set.pop();
      state_to_heap.erase(current.o_state);
      closed_set.insert(current.o_state);
      auto &current_ts_state = current.o_state.ts_state;
      int current_goal_state = current.o_state.goal_state;
      for (int k = 0; k < 5; k++) {
        int next_time = current_ts_state.time + 1;
        int next_x = current_ts_state.x + dirx[k];
        int next_y = current_ts_state.y + diry[k];
        if (!env_.LocationVaild(Location(next_x, next_y))) continue;
        auto next_ts_state = TSState(next_time, next_x, next_y);
        if (cons.IsRestricted(current_ts_state, next_ts_state)) continue;
        int min_dis = Interval::oo;
        std::vector<int> on_goal_ids;
        for (uint i = 0; i < agent.goals.size(); i++) {
          if (((1 << i) & current_goal_state) != 0) continue;
          int dis = full_distance_matrix[i][next_x][next_y];
          if (dis == 0) on_goal_ids.emplace_back(i);
          min_dis = std::min(min_dis, dis);
        }
        if (min_dis == Interval::oo) {
          for (uint i = 0; i < agent.goals.size(); i++) {
            int dis = full_distance_matrix[i][next_x][next_y];
            min_dis = std::min(min_dis, dis);
          }
        }
        int next_goal_state = current_goal_state;
        for (int on_goal_id : on_goal_ids) next_goal_state |= 1 << on_goal_id;
        auto next_o_state = OState(next_ts_state, next_goal_state);
        if (closed_set.find(next_o_state) == closed_set.end()) {
          int g = current.g + 1;
          auto iter = state_to_heap.find(next_o_state);
          if (iter == state_to_heap.end()) {
            int heuristic =
                min_dis + h_table.GetHeuristicMid(next_goal_state, -1);
            int f = g + heuristic;
            auto handle = open_set.push(ONode(next_o_state, g, f));
            state_to_heap[next_o_state] = handle;
          } else {
            auto handle = iter->second;
            if (g >= (*handle).g) {
              continue;
            }
            int delta = (*handle).g - g;
            (*handle).g = g;
            (*handle).f -= delta;
            open_set.update(handle);
          }
          come_from[next_o_state] = current.o_state;
        }
      }
    }
    return false;
  }
  typedef boost::heap::d_ary_heap<ONode, boost::heap::arity<2>,
                                  boost::heap::mutable_<true>>
      OpenSet;
  typedef OpenSet::handle_type HeapHandle_t;

  typedef std::unordered_map<OState, HeapHandle_t> StateToHeap;
  typedef std::unordered_set<OState> ClosedSet;
  typedef std::unordered_map<OState, OState> ComeFrom;
  const Environment &env_;
  const int dirx[5] = {0, 1, 0, -1, 0};
  const int diry[5] = {1, 0, -1, 0, 0};
};
};  // namespace optimal_cbs

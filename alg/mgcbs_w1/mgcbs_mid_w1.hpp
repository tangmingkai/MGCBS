#pragma once
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/heap/d_ary_heap.hpp>

#include "base/agent.hpp"
#include "base/plan_result.hpp"
#include "env/environment.hpp"

#include "alg/mgcbs_w1/h_table_w1.hpp"
#include "alg/mgcbs_w1/mgcbs_low_w1.hpp"
namespace mg_cbs_w1 {
struct MiddleState {
  MiddleState() = default;
  MiddleState(int _goal_state, int _target_id, int _si_id)
      : goal_state(_goal_state), target_id(_target_id), si_id(_si_id) {}
  int goal_state;
  int target_id;
  int si_id;
  bool operator==(const MiddleState &other) const {
    return goal_state == other.goal_state && target_id == other.target_id &&
           si_id == other.si_id;
  }
};
};  // namespace mg_cbs_w1

template <>
struct std::hash<mg_cbs_w1::MiddleState> {
  size_t operator()(const mg_cbs_w1::MiddleState &x) const {
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<int>()(x.goal_state));
    boost::hash_combine(seed, std::hash<int>()(x.target_id));
    boost::hash_combine(seed, std::hash<int>()(x.si_id));
    return seed;
  }
};

namespace mg_cbs_w1 {
class MGCBSMid {
 public:
  struct MiddleLevelNode {
    MiddleState middle_state;
    int g, f;
    MiddleLevelNode() = default;
    MiddleLevelNode(int state, int target_id, int si_id, int g, int f)
        : middle_state(state, target_id, si_id), g(g), f(f) {}
    bool operator<(const MiddleLevelNode &n) const {
      if (f != n.f) return f > n.f;
      return g < n.g;
    }

    friend std::ostream &operator<<(std::ostream &os,
                                    const MiddleLevelNode &c) {
      os << "state: (" << c.middle_state.goal_state << ", (" << c.g << ", "
         << c.f << ")" << std::endl;
      return os;
    }
  };
  typedef boost::heap::d_ary_heap<MiddleLevelNode, boost::heap::arity<2>,
                                  boost::heap::mutable_<true>>
      OpenSet;
  typedef OpenSet::handle_type HeapHandle_t;

  typedef std::unordered_map<MiddleState, HeapHandle_t> StateToHeap;
  typedef std::unordered_map<MiddleState, int> ClosedSet;
  typedef std::unordered_map<MiddleState,
                             std::pair<MiddleState, std::vector<TSState>>>
      ComeFrom;
  explicit MGCBSMid(const Environment &env)
      : env_(env), low_level_search_(env) {}

  bool Search(const Agent &agent, const HTable &h_table,
              const Constraints &cons, PlanResult *plan_result) {
    OpenSet open_set;
    StateToHeap state_to_heap;
    ClosedSet closed_set;
    ComeFrom come_from;

    closed_set.clear();
    double log_2 = log(2);

    auto handle = open_set.push(
        MiddleLevelNode(0, 0, 0, 0, h_table.GetHeuristicMid(0, 0)));
    state_to_heap[MiddleState(0, 0, 0)] = handle;
    int goal_num = agent.goals.size();
    int all_goal_state = (1 << goal_num) - 1;
    // segments[i][j]: The j TIS state at target[i].
    std::vector<std::vector<TISState>> segments;
    BuildSegments(agent, cons, &segments);
    while (!open_set.empty()) {
      const auto current = open_set.top();
      open_set.pop();
      state_to_heap.erase(current.middle_state);
      closed_set[current.middle_state] = current.g;
      int now_state = current.middle_state.goal_state;
      int now_target_id = current.middle_state.target_id;
      if (now_state == all_goal_state) {
        GeneratePlanningResult(come_from, current.middle_state, agent, cons,
                               plan_result);
        return true;
      }
      int remain_state = all_goal_state ^ now_state;
      auto current_ts_state = TSState(current.g, agent.targets[now_target_id].x,
                                      agent.targets[now_target_id].y);
      while (remain_state > 0) {
        int p_next_goal_id = remain_state - (remain_state & (remain_state - 1));
        int next_goal_id = static_cast<int>(log(p_next_goal_id) / log_2 + 0.5);
        int next_target_id = next_goal_id + 1;
        int next_state = now_state ^ p_next_goal_id;
        remain_state = remain_state - p_next_goal_id;
        int si_size = segments[next_target_id].size();
        int next_si_id = 0;
        if (next_state == all_goal_state) {
          next_si_id = si_size - 1;
        }
        for (; next_si_id < si_size; next_si_id++) {
          auto np = MiddleState(next_state, next_target_id, next_si_id);
          {
            auto iter = closed_set.find(np);
            if (iter != closed_set.end()) continue;
          }
          {
            auto iter = state_to_heap.find(np);
            int next_g;
            std::vector<TSState> path;
            next_g = low_level_search_.Search(
                current_ts_state, segments[next_target_id][next_si_id], cons,
                h_table.GetFullDistanceMatrix(next_target_id), &path);
            if (next_g == Interval::oo) continue;
            if (iter == state_to_heap.end()) {
              auto handle = open_set.push(MiddleLevelNode(
                  next_state, next_target_id, next_si_id, next_g,
                  next_g +
                      h_table.GetHeuristicMid(next_state, next_target_id)));
              state_to_heap[np] = handle;
              come_from[np] = std::make_pair(current.middle_state, path);
            } else {
              auto handle = iter->second;
              if (next_g >= (*handle).g) {
                continue;
              }
              int delta = (*handle).g - next_g;
              (*handle).g = next_g;
              (*handle).f = (*handle).f - delta;
              open_set.update(handle);
              come_from[np] = std::make_pair(current.middle_state, path);
            }
          }
        }
      }
    }
    // std::cout << "Cannot search a path in middle level" << std::endl;
    return false;
  }

 private:
  void BuildSegments(const Agent &agent, const Constraints &cons,
                     std::vector<std::vector<TISState>> *segments) {
    segments->clear();
    segments->resize(agent.targets.size());
    for (int i = 0; i < static_cast<int>(agent.targets.size()); i++) {
      auto con_iter = cons.vertex_constraints.find(agent.targets[i]);
      int l = 0;
      if (con_iter != cons.vertex_constraints.end()) {
        for (auto &iter2 : con_iter->second) {
          int r = iter2.time - 1;
          if (r >= l) {
            (*segments)[i].emplace_back(TISState(
                Interval(l, r), agent.targets[i].x, agent.targets[i].y));
          }
          l = iter2.time + 1;
        }
      }
      (*segments)[i].emplace_back(TISState(
          Interval(l, Interval::oo), agent.targets[i].x, agent.targets[i].y));
    }
  }

  void GeneratePlanningResult(const ComeFrom &come_from,
                              const MiddleState &final_middle_state,
                              const Agent &agent, const Constraints &cons,
                              PlanResult *plan_result) {
    plan_result->Clear();
    std::vector<std::vector<TSState>> results;
    auto now_state = final_middle_state;
    while (true) {
      auto iter = come_from.find(now_state);
      if (iter == come_from.end()) {
        break;
      }
      now_state = iter->second.first;
      results.emplace_back(iter->second.second);
    }

    plan_result->states.emplace_back(agent.start);
    for (int i = static_cast<int>(results.size()) - 1; i >= 0; i--)
      for (int j = 0; j < static_cast<int>(results[i].size()); j++)
        plan_result->states.emplace_back(results[i][j]);
    plan_result->cost = plan_result->states.size() - 1;
  }
  const Environment &env_;
  MGCBSLow low_level_search_;
};
};  // namespace mg_cbs_w1



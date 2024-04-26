#pragma once
#include <unordered_map>
#include <utility>
#include <vector>
#include <boost/heap/d_ary_heap.hpp>

#include "base/agent.hpp"
#include "base/plan_result.hpp"
#include "env/environment.hpp"

#include "alg/mgcbs/h_table.hpp"
#include "alg/mgcbs/siforest.hpp"

namespace mg_cbs {
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
};  // namespace mg_cbs

template <>
struct std::hash<mg_cbs::MiddleState> {
  size_t operator()(const mg_cbs::MiddleState &x) const {
    size_t seed = 0;
    boost::hash_combine(seed, std::hash<int>()(x.goal_state));
    boost::hash_combine(seed, std::hash<int>()(x.target_id));
    boost::hash_combine(seed, std::hash<int>()(x.si_id));
    return seed;
  }
};

namespace mg_cbs {
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
  typedef std::unordered_map<MiddleState, MiddleState> ComeFrom;
  explicit MGCBSMid(const Environment &env) : env_(env) {}

  bool Search(const Agent &agent, const SIForest &si_forest,
              const HTable &h_table, const Constraints &cons,
              PlanResult *plan_result) {
    OpenSet open_set;
    StateToHeap state_to_heap;
    ClosedSet closed_set;
    ComeFrom come_from;

    closed_set.clear();
    double log_2 = log(2);

    // MiddleLevelNode(int state, int target_id, int si_id, int g, int f)
    auto handle =
        open_set.push(MiddleLevelNode(0, 0, 0, 0, h_table.GetHeuristic(0, 0)));
    state_to_heap[MiddleState(0, 0, 0)] = handle;
    int goal_num = agent.goals.size();
    int all_goal_state = (1 << goal_num) - 1;
    while (!open_set.empty()) {
      const auto current = open_set.top();
      open_set.pop();
      state_to_heap.erase(current.middle_state);
      closed_set[current.middle_state] = current.g;
      int now_state = current.middle_state.goal_state;
      int now_target_id = current.middle_state.target_id;
      if (now_state == all_goal_state) {
        GeneratePlanningResult(come_from, current.middle_state, si_forest,
                               agent, cons, plan_result);
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
        int si_size = si_forest.GetSize(next_target_id);
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
            next_g = si_forest.GetNextTargetReachTime(
                current_ts_state, next_target_id, next_si_id);
            if (next_g == Interval::oo) continue;
            if (iter == state_to_heap.end()) {
              auto handle = open_set.push(MiddleLevelNode(
                  next_state, next_target_id, next_si_id, next_g,
                  next_g + h_table.GetHeuristic(next_state, next_target_id)));
              state_to_heap[np] = handle;
              come_from[np] = current.middle_state;
            } else {
              auto handle = iter->second;
              if (next_g >= (*handle).g) {
                continue;
              }
              int delta = (*handle).g - next_g;
              (*handle).g = next_g;
              (*handle).f = (*handle).f - delta;
              open_set.update(handle);
              come_from[np] = current.middle_state;
            }
          }
        }
      }
    }
    // std::cout << "Cannot search a path in middle level" << std::endl;
    return false;
  }

 private:
  void GeneratePlanningResult(const ComeFrom &come_from,
                              const MiddleState &final_middle_state,
                              const SIForest &si_forest, const Agent &agent,
                              const Constraints &cons,
                              PlanResult *plan_result) {
    plan_result->Clear();
    std::vector<std::pair<int, int>> ids;
    auto now_state = final_middle_state;
    ids.emplace_back(std::make_pair(now_state.target_id, now_state.si_id));
    while (true) {
      auto iter = come_from.find(now_state);
      if (iter == come_from.end()) {
        break;
      }
      now_state = iter->second;
      ids.emplace_back(std::make_pair(now_state.target_id, now_state.si_id));
    }
    plan_result->states.emplace_back(agent.start);
    for (int i = static_cast<int>(ids.size()) - 2; i >= 0; i--) {
      auto now_ts_state = plan_result->states.back();
      si_forest.AppendPath(now_ts_state, ids[i].first, ids[i].second,
                           plan_result);
    }
    plan_result->cost = plan_result->states.size() - 1;
  }
  const Environment &env_;
};
};  // namespace mg_cbs


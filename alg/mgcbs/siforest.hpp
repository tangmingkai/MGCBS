#pragma once
#include <list>
#include <memory>
#include <unordered_map>
#include <vector>

#include "alg/mgcbs/sitree.hpp"
#include "base/agent.hpp"
#include "base/location.hpp"
#include "base/ts_state.hpp"
#include "env/environment.hpp"
namespace mg_cbs {
// The TIS Forest in the paper
class SIForest {
 public:
  explicit SIForest(const Environment &env) : env_(env) {}

  void Build(const Agent &agent,
             const Constraints &constraints) {
    sitrees_.clear();
    auto &targets = agent.targets;
    for (uint i = 1; i < targets.size(); i++) {
      sitrees_.emplace_back();
      auto &sitrees = sitrees_.back();
      auto iter1 = constraints.vertex_constraints.find(targets[i]);
      int l = 0;
      if (iter1 != constraints.vertex_constraints.end()) {
        for (auto &iter2 : iter1->second) {
          int r = iter2.time - 1;
          if (r >= l) {
            sitrees.emplace_back(SITree(env_));
            sitrees.back().Build(
                TISState(Interval(l, r), targets[i].x, targets[i].y),
                constraints);
          }
          l = iter2.time + 1;
        }
      }
      sitrees.emplace_back(SITree(env_));
      sitrees.back().Build(
          TISState(Interval(l, Interval::oo), targets[i].x, targets[i].y),
          constraints);
    }
  }
  int GetNextTargetReachTime(const TSState &from,
                             const int &to_target_id,
                             const int &to_si_id) const {
    int cost = sitrees_[to_target_id-1][to_si_id].GetCost(from);
    if (cost == Interval::oo) return Interval::oo;
    return from.time + cost;
  }
  bool AppendPath(const TSState &ts_state, const int &to_target_id,
                  const int &to_si_id, PlanResult *plan_result) const {
    return sitrees_[to_target_id-1][to_si_id].AppendPath(ts_state, plan_result);
  }
  int GetSize(int target_id) const {
    return sitrees_[target_id-1].size();
  }

 private:
  std::vector<std::vector<SITree>> sitrees_;
  const Environment &env_;
};
using SIForestPtr = std::shared_ptr<SIForest>;
};  // namespace mg_cbs

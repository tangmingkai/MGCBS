#pragma once
#include <memory>
#include <vector>
#include <string>

#include "alg/mgcbs/mgcbs_high.hpp"
#include "alg/mgcbs_w1/mgcbs_high_w1.hpp"
#include "alg/mgcbs_w2/mgcbs_high_w2.hpp"
#include "alg/optimal_cbs/optimal_cbs.hpp"
#include "env/environment.hpp"

const std::string alg_list =
    "[MGCBS, MGCBS_w1, MGCBS_w2, OptimalCBS]";

// template <typename State, typename Action, typename Cost, typename Conflict,
//           typename Constraints, typename Environment, typename Agent,
//           typename StateHasher = std::hash<State>>
class Solver {
 public:
  Solver(const Environment& env, const std::string& alg_name) : env_(env) {
    init_succ_ = true;
    if (alg_name == "MGCBS")
      alg_ = std::make_shared<mg_cbs::MGCBSHigh>(env);
    else if (alg_name == "MGCBS_w1")
      alg_ = std::make_shared<mg_cbs_w1::MGCBSHigh>(env);
    else if (alg_name == "MGCBS_w2")
      alg_ = std::make_shared<mg_cbs_w2::MGCBSHigh>(env);
    else if (alg_name == "OptimalCBS")
      alg_ = std::make_shared<optimal_cbs::OptimalCBS>(env);
    else
      init_succ_ = false;
  }
  bool Solve(const std::vector<Agent>& agents,
             std::vector<PlanResult>* solution) {
    if (alg_->Search(agents, solution) == false) return false;
    for (uint i = 0; i < agents.size(); i++) {
      (*solution)[i].agent_name = agents[i].agent_name;
    }
    return true;
  }

  bool init_succ() const {return init_succ_;}

 private:
  std::shared_ptr<BaseAlgorithm> alg_;
  const Environment& env_;
  bool init_succ_;
};

